// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
#define CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_

#include <memory>
#include <string>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/srv/led_execute.hpp"
#include "low_power_consumption/low_power_consumption.hpp"
#include "protocol/msg/state_switch_status.hpp"

namespace cyberdog
{
namespace manager
{
enum class PowerMachineState : uint8_t
{
  PMS_ACTIVE    = 0,    // 正常
  PMS_PROTECT   = 1,    // 保护
  PMS_LOWPOWER  = 2,    // 低功耗
  PMS_OTA       = 3,    // OTA
  PMS_SHUTDOWN  = 4,    // 关机
  PMS_UNKOWN    = 255,  // 未知
};

class PowerConsumptionInfoNode final
{
  using PCIN_CALLBACK = std::function<void (uint8_t)>;

public:
  PowerConsumptionInfoNode(rclcpp::Node::SharedPtr node_ptr, PCIN_CALLBACK callback)
  : power_consumption_info_node_(node_ptr),
    machine_state_switch_handler(callback)
  {
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
    power_consumption_callback_group_ =
      power_consumption_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    reboot_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::Trigger>(
      "reboot", std::bind(
        &PowerConsumptionInfoNode::RebootCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    power_off_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::Trigger>(
      "poweroff", std::bind(
        &PowerConsumptionInfoNode::ShutdownCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    led_excute_client_ =
      power_consumption_info_node_->create_client<protocol::srv::LedExecute>(
      "led_execute", rmw_qos_profile_services_default, power_consumption_callback_group_);

    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    motion_status_sub_ =
      power_consumption_info_node_->create_subscription<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_mostion_status_callback, this,
        std::placeholders::_1), options);

    state_switch_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::StateSwitchStatus>(
      "state_switch_status",
      rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::machine_state_switch_callback,
        this, std::placeholders::_1), options);

    wake_up_sub_ =
      power_consumption_info_node_->create_subscription<std_msgs::msg::Bool>(
      "dog_wakeup", rclcpp::SystemDefaultsQoS(),
      std::bind(&PowerConsumptionInfoNode::DogWakeup, this, std::placeholders::_1),
      options);
  }

public:
  // 后续使用toml文件配置

  void Init()
  {
    toml::value value_table;
    auto local_config_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_config_dir + std::string("/toml_config/manager/lowpower.toml");
    if (access(local_config_dir.c_str(), F_OK) != 0) {
      ERROR(" %s do not exist!", local_config_dir.c_str());
      return;
    } else {
      INFO("load lowpoer toml file successfully");
    }
    if (!cyberdog::common::CyberdogToml::ParseFile(local_config_dir, value_table)) {
      ERROR("fail to read data from lowpoer_priority toml");
      return;
    }
    if (!value_table.is_table()) {
      ERROR("Toml format error");
      return;
    }
    cyberdog::common::CyberdogToml::Get(value_table, "enable_lowpower", enable_lowpower_);
    cyberdog::common::CyberdogToml::Get(value_table, "enter_lp_time", enter_lp_times_);
    cyberdog::common::CyberdogToml::Get(value_table, "re-enter_lp_time", renter_lp_times_);
  }

  void SwitchMachinsState(PowerMachineState machine_state)
  {
    uint8_t ms = static_cast<uint8_t>(machine_state);
    machine_state_switch_handler(ms);
  }

  void BatteryChangeUpdata(int soc, bool is_charging)
  {
    battery_soc_ = soc;
    battery_is_charging_ = is_charging;
    static uint8_t pre_soc = battery_soc_;

    if (battery_soc_ == 0 && !battery_is_charging_) {
      ShutdownHandler();
      return;
    }

    if (battery_soc_ < 5) {
      if (battery_soc_ < pre_soc) {
        INFO("[Text]: soc less than 5 before lowpower");
        LowPowerHandler();
        INFO("[Text]: soc less than 5 after lowpower");
      }
    } else if (battery_soc_ < 20) {
      if (battery_soc_ < pre_soc) {
        INFO("[Text]: soc less than 20 before protect");
        ProtectHandler();
        INFO("[Text]: soc less than 20 after protect");
      }
    } else if (battery_soc_ < 30) {
      if (battery_soc_ < pre_soc) {
        INFO("[Text]: soc less than 30 before act");
        ActivedHandler();
        INFO("[Text]: soc less than 30 before act");
      }
    } else {}
    pre_soc = battery_soc_;
  }

  void ActivedHandler()
  {
    std::lock_guard<std::mutex> lck(switch_mtx);
    if (machine_state_ == PowerMachineState::PMS_ACTIVE ||
      machine_state_ == PowerMachineState::PMS_LOWPOWER) {return;}
    SwitchMachinsState(PowerMachineState::PMS_ACTIVE);
  }

  void ProtectHandler()
  {
    std::lock_guard<std::mutex> lck(switch_mtx);
    if (machine_state_ == PowerMachineState::PMS_PROTECT ||
      machine_state_ == PowerMachineState::PMS_LOWPOWER) {return;}
    SwitchMachinsState(PowerMachineState::PMS_PROTECT);
  }

  void LowPowerHandler()
  {
    std::lock_guard<std::mutex> lck(switch_mtx);
    if (machine_state_ == PowerMachineState::PMS_LOWPOWER) {return;}
    SwitchMachinsState(PowerMachineState::PMS_LOWPOWER);
    SysLowpowerEntry(true);
    TailLedControl(true);
  }

  void ShutdownHandler()
  {
    std::lock_guard<std::mutex> lck(switch_mtx);
    SwitchMachinsState(PowerMachineState::PMS_SHUTDOWN);
    poweroff();
  }

  void RebootCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    INFO("reboot......");
    PM_SYS pd = PM_SYS_REBOOT;
    int code = -1;
    code = lpc_ptr_->LpcSysRequest(pd);
    if (code != 0) {
      ERROR("reboot failed, LpcRequest error code is %d", code);
    } else {
      INFO("reboot successfully");
    }
    response->success = (code == 0 ? true : false);
  }

  void ShutdownCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    INFO("poweroff......");
    PM_SYS pd = PM_SYS_SHUTDOWN;
    int code = -1;
    code = lpc_ptr_->LpcSysRequest(pd);
    if (code != 0) {
      INFO("Shutdown failed, LpcRequest error code is %d", code);
    } else {
      INFO("Shut down successfully");
    }
    response->success = (code == 0 ? true : false);
  }

  void SysLowpowerEntry(bool is_enter)
  {
    // PM_DEV pd = PM_CAM_ALL;
    PM_DEV pd = PM_ALL_NO_TOF;
    static int r_count = 0;
    unsigned int err;
    int code = -1;
    INFO(
      "[LowPower]: [%d]EnterLowPower %s:start", (r_count + 1),
      (is_enter ? "true" : "false"));
    if (is_enter) {
      code = lpc_ptr_->LpcRelease(pd, &err);
      ++r_count;
    } else {
      code = lpc_ptr_->LpcRequest(pd, &err);
      ++r_count;
    }
    if (code != 0) {
      ERROR(
        "[LowPower]: %s lowpoer false, Lpc error code is %d",
        (is_enter ? "enter" : "exit"), code);
    }
    INFO(
      "[LowPower]: [%d]EnterLowPower %s:stop", (r_count + 1),
      (is_enter ? "true" : "false"));
  }

  void poweroff()
  {
    INFO("cyberdog will poweroff when the battery soc is 0.");
    PM_SYS pd = PM_SYS_SHUTDOWN;
    int code = -1;
    code = lpc_ptr_->LpcSysRequest(pd);
    if (code != 0) {
      INFO("poweroff failed, LpcSysRequest error code is %d", code);
    } else {
      INFO("poweroff successfully");
    }
  }

  void sub_mostion_status_callback(const protocol::msg::MotionStatus::SharedPtr msg)
  {
    if (!enable_lowpower_) {
      return;
    }

    if (machine_state_ == PowerMachineState::PMS_OTA) {
      INFO_MILLSECONDS(3000, "[LowPower]: in ota state return.");
      return;
    }

    if (machine_state_ == PowerMachineState::PMS_LOWPOWER) {
      // lay_count = 0;
      return;
    }

    // motion_id: 趴下(101)、站立(111)
    int motion_id = msg->motion_id;
    static u_int32_t lay_count = 0;
    static int times = 0;
    INFO_MILLSECONDS(5000, "[Text]: lay_count is %d", lay_count);

    if (motion_id == 0) {
      ++lay_count;
      if (lay_count == renter_lp_times_ * 10) {
        INFO("[LowPower]: call low power consumption when the dog lies down for 2min");
        lay_count = 0;
        LowPowerHandler();
        ++times;
      }
    } else if (motion_id == 101) {
      ++lay_count;
      if (times == 0) {
        if (lay_count == enter_lp_times_ * 10) {
          INFO("[LowPower]: call low power consumption when the dog lies down for 30s");
          lay_count = 0;
          LowPowerHandler();
          ++times;
        }
      } else {
        if (lay_count == renter_lp_times_ * 10) {
          lay_count = 0;
          INFO("[LowPower]: call low power consumption when the dog lies down for 2min");
          LowPowerHandler();
        }
      }
    } else {
      times = 0;
      lay_count = 0;
    }
  }

  void DogWakeup(const std_msgs::msg::Bool::SharedPtr msg)
  {
    INFO("msg->data = ", msg->data);
    INFO("[Text]: dog wakeup");
    if (is_exiting_lowpower_) {
      INFO("[LowPower]: wakeup rejected, now in exiting lowpower");
      return;
    }

    if (machine_state_ != PowerMachineState::PMS_LOWPOWER || battery_soc_ < 5) {
      return;
    }
    ExitLowpower();
  }

  void ExitLowpower()
  {
    is_exiting_lowpower_ = true;
    SysLowpowerEntry(false);
    if (battery_soc_ < 20) {
      INFO("[Text]: dog wakeup, soc less than 20");
      SwitchMachinsState(PowerMachineState::PMS_PROTECT);
    } else {
      INFO("[Text]: dog wakeup, soc more than 20");
      SwitchMachinsState(PowerMachineState::PMS_ACTIVE);
    }
    TailLedControl(false);
    is_exiting_lowpower_ = false;
  }

  void TailLedControl(bool is_light_off)
  {
    if (!led_excute_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call led_excute server not avalible");
      return;
    }
    auto request_led = std::make_shared<protocol::srv::LedExecute::Request>();
    request_led->occupation = is_light_off;
    request_led->client = "lowpower";
    request_led->target = 2;
    request_led->mode = 0x01;
    request_led->effect = 0xA0;
    auto future_result_tail = led_excute_client_->async_send_request(request_led);
    std::future_status status_tail = future_result_tail.wait_for(std::chrono::seconds(2));

    if (status_tail != std::future_status::ready) {
      ERROR("call led_execute service failed");
    }

    if (future_result_tail.get()->code == 0) {
      INFO("call led service successed");
    } else {
      ERROR(
        "release tail led fialed, error code is:%d", future_result_tail.get()->code);
    }
  }

  void machine_state_switch_callback(const protocol::msg::StateSwitchStatus::SharedPtr msg)
  {
    // 0-active；1-低电量；2-低功耗；3-OTA；4-关机
    machine_state_ = static_cast<PowerMachineState>(msg->state);
  }

private:
  rclcpp::Node::SharedPtr power_consumption_info_node_ {nullptr};
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr power_consumption_callback_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_ {nullptr};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_ {nullptr};
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_excute_client_;
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  rclcpp::Subscription<protocol::msg::StateSwitchStatus>::SharedPtr state_switch_status_sub_ {
    nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wake_up_sub_ {nullptr};
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex switch_mtx;
  PCIN_CALLBACK machine_state_switch_handler {[](uint8_t) {}};
  PowerMachineState machine_state_ {PowerMachineState::PMS_UNKOWN};
  u_int32_t enter_lp_times_ {30};
  u_int32_t renter_lp_times_ {120};
  uint8_t battery_soc_ {100};
  bool battery_is_charging_ {false};
  bool enable_lowpower_ {true};
  bool is_exiting_lowpower_ {false};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
