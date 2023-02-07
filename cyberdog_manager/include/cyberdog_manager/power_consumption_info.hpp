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
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/msg/motion_status.hpp"
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
   :power_consumption_info_node_(node_ptr), machine_state_switch_handler(callback)
  {
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
    power_consumption_callback_group_ = 
      power_consumption_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    low_power_consumption_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::SetBool>(
      "low_power_consumption",
      std::bind(
        &PowerConsumptionInfoNode::EnterLowPower, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    reboot_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::Trigger>(
      "reboot", std::bind(&PowerConsumptionInfoNode::RebootCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    power_off_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::Trigger>(
      "poweroff", std::bind(&PowerConsumptionInfoNode::ShutdownCallback, this,
      std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);
    
    low_power_client_ =
      power_consumption_info_node_->create_client<std_srvs::srv::SetBool>(
      "low_power_consumption",
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    power_off_client_ =
      power_consumption_info_node_->create_client<std_srvs::srv::Trigger>(
      "poweroff",
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    motion_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_mostion_status_callback,
        this, std::placeholders::_1), options);

    state_switch_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::machine_state_switch_callback,
        this, std::placeholders::_1), options);
  }

public:
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
    is_switching_ms_ = true;
    machine_state_switch_handler(ms);
    is_switching_ms_ = false;
  }

  void ActivedHandler() {
    if (machine_state_ == PowerMachineState::PMS_ACTIVE ||
        machine_state_ == PowerMachineState::PMS_LOWPOWER) return;
    SwitchMachinsState(PowerMachineState::PMS_ACTIVE);
  }

  void ProtectHandler() {
    if (machine_state_ == PowerMachineState::PMS_PROTECT ||
        machine_state_ == PowerMachineState::PMS_LOWPOWER) return;
    SwitchMachinsState(PowerMachineState::PMS_PROTECT);
  }

  void LowPowerHandler() {
    if (machine_state_ == PowerMachineState::PMS_LOWPOWER) return;
    SwitchMachinsState(PowerMachineState::PMS_LOWPOWER);
    SysLowpowerEntry(true);
  }
  
  void ShutdownHandler() {
    SwitchMachinsState(PowerMachineState::PMS_SHUTDOWN);
    poweroff();
  }

  // void SetMachine(PCIN_CALLBACK callback)
  // {
  //   machine_state_switch_handle = callback;
  // }

  void UpdataBatterySoc(uint8_t soc) {
    battery_soc_ = soc;
  }

private:
  void EnterLowPower(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    // PM_DEV pd = PM_CAM_ALL;
    PM_DEV pd = PM_ALL_NO_TOF;
    static int r_count = 0;
    unsigned int err;
    int code = -1;
    INFO("[LowPower]: [%d]EnterLowPower %s:start", (r_count + 1),
      (request->data ? "true" : "false"));
    if (request->data) {
      code = lpc_ptr_->LpcRelease(pd, &err);
      ++r_count;
    } else {
      code = lpc_ptr_->LpcRequest(pd, &err);
      ++r_count;
    }
    response->success = (code == 0 ? true : false);

    if (!response->success) {
      ERROR("[LowPower]: %s lowpoer false, Lpc error code is %d",
        (request->data ? "enter" : "exit"), code);
    }
    INFO(
      "[LowPower]: [%d]EnterLowPower %s:stop", (r_count + 1),
      (request->data ? "true" : "false"));
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
    INFO("[LowPower]: now dog %s low-power model.", (is_enter ? "enter" : "exit"));
    if (!low_power_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("[LowPower]: call low-power service not avalible");
    } else {
      std::chrono::seconds timeout(10);
      auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
      req->data = is_enter;
      auto future_result = low_power_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO("[LowPower]: call low-power service success.");
      } else {
        ERROR("[LowPower]: call low-power service failed!");
      }
    }
  }

  void poweroff()
  {
    INFO("cyberdog start poweroff when the battery soc is 0.");
    if (!power_off_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call poweroff service not avalible");
    } else {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future_result = power_off_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO("call poweroff service success.");
      } else {
        ERROR("call poweroff service failed!");
      }
    }
  }


  void sub_mostion_status_callback(const protocol::msg::MotionStatus::SharedPtr msg)
  {
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

    if (motion_id == 0) {
      ++lay_count;
      if (lay_count == renter_lp_times_ * 10) {
        INFO("[LowPower]: call low power consumption when the dog lies down for 2min");
        LowPowerHandler();
        ++times;
      }
    } else if (motion_id == 101) {
      ++lay_count;
      if (times == 0) {
        if (lay_count == enter_lp_times_ * 10) {
          INFO("[LowPower]: call low power consumption when the dog lies down for 30s");
          LowPowerHandler();
          ++times;
        }
      } else {
        if (lay_count == renter_lp_times_ * 10) {
          INFO("[LowPower]: call low power consumption when the dog lies down for 2min");
          LowPowerHandler();
        }
      }
    } else {
      times = 0;
      lay_count = 0;
    }
    
  }

  void AudioWakeUp()
  {
    if (is_switching_ms_) {
      INFO("[LowPower]: wakeup rejected, now switching machine state");
      return;
    }

    if (machine_state_ != PowerMachineState::PMS_LOWPOWER || battery_soc_ < 5) {
      return;
    }

    if (battery_soc_ < 20) {
      SysLowpowerEntry(false);
      SwitchMachinsState(PowerMachineState::PMS_PROTECT);
    } else {
      if (machine_state_ == PowerMachineState::PMS_LOWPOWER) {
        SysLowpowerEntry(false);
      }
      SwitchMachinsState(PowerMachineState::PMS_ACTIVE);
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
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_power_consumption_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_ {nullptr};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr low_power_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  rclcpp::Subscription<protocol::msg::StateSwitchStatus>::SharedPtr state_switch_status_sub_ {nullptr};
  PCIN_CALLBACK machine_state_switch_handler {[](uint8_t) {}};
  PowerMachineState machine_state_ {PowerMachineState::PMS_UNKOWN};
  u_int32_t enter_lp_times_ {30};
  u_int32_t renter_lp_times_ {120};
  uint8_t battery_soc_ {100};
  bool enable_lowpower_ {true};
  bool is_switching_ms_ {false};
};
}
}

#endif // CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_