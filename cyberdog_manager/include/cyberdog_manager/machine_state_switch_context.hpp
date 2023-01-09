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
#ifndef CYBERDOG_MANAGER__MACHINE_STATE_SWITCH_CONTEXT_HPP_
#define CYBERDOG_MANAGER__MACHINE_STATE_SWITCH_CONTEXT_HPP_

#include <string>
#include <memory>
#include <map>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/msg/state_switch_status.hpp"

namespace cyberdog
{
namespace manager
{
enum class MsscMachineState : uint8_t
{
  MSSC_ACTIVE    = 0,    // 正常
  MSSC_PROTECT   = 1,    // 保护
  MSSC_LOWPOWER  = 2,    // 低功耗
  MSSC_OTA       = 3,    // OTA
  MSSC_SHUTDOWN  = 4,    // 关机
  MSSC_UNKOWN    = 255,  // 未知
};

class MachineStateSwitchContext final
{
  using BSSC_CALLBACK = std::function<void ()>;

public:
  explicit MachineStateSwitchContext(rclcpp::Node::SharedPtr node_ptr)
  : mssc_node_(node_ptr)
  {
    mssc_callback_group_ =
      mssc_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = mssc_callback_group_;
    state_set_sub_ = mssc_node_->create_subscription<std_msgs::msg::UInt8>(
      "machine_state_switch", rclcpp::SystemDefaultsQoS(),
      std::bind(&MachineStateSwitchContext::MachineStateSwitch, this, std::placeholders::_1),
      sub_options);
    state_valget_srv_ =
      mssc_node_->create_service<std_srvs::srv::Trigger>(
      "machine_state_valget",
      std::bind(
        &MachineStateSwitchContext::MachineStateGet, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    switch_ota_state_srv_ =
      mssc_node_->create_service<std_srvs::srv::SetBool>(
      "ota_state_switch",
      std::bind(
        &MachineStateSwitchContext::OtaMachineState, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    power_off_client_ =
      mssc_node_->create_client<std_srvs::srv::Trigger>(
      "poweroff",
      rmw_qos_profile_services_default, mssc_callback_group_);
    low_power_client_ =
      mssc_node_->create_client<std_srvs::srv::SetBool>(
      "low_power_consumption",
      rmw_qos_profile_services_default, mssc_callback_group_);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = mssc_callback_group_;
    state_swith_status_pub_ =
      mssc_node_->create_publisher<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), pub_options);
  }
  void Init()
  {
    mssc_machine_state = MsscMachineState::MSSC_ACTIVE;
  }
  void SetActive(BSSC_CALLBACK callback)
  {
    active_handler = callback;
  }
  void SetProtect(BSSC_CALLBACK callback)
  {
    protect_handler = callback;
  }
  void SetLowpower(BSSC_CALLBACK callback)
  {
    lowpower_handler = callback;
  }
  void SetOta(BSSC_CALLBACK callback)
  {
    ota_handler = callback;
  }
  void SetShutdown(BSSC_CALLBACK callback)
  {
    shutdown_handler = callback;
  }
  void AudioWakeUp()
  {
    if (is_switching_ms_) {
      INFO("[LowPower]: wakeup rejected, now switching machine state");
      return;
    }

    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return;
    } else if (battery_charge_val < 5) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        return;
      } else {
        ERROR(
          "[LowPower]: switch state error1:wake up, battery charge val:%d, current state:%d",
          battery_charge_val, mssc_machine_state);
      }
    } else if (battery_charge_val < 20) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        lowpower(false);
        SwitchState(MsscMachineState::MSSC_PROTECT);
        INFO("[LowPower]: wakeup from lowpower to protect");
      } else {
        ERROR(
          "[LowPower]: switch state error2:wakeup, battery charge val:%d, current state:%d",
          battery_charge_val, mssc_machine_state);
      }
    } else {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        lowpower(false);
        SwitchState(MsscMachineState::MSSC_ACTIVE);
        INFO("[LowPower]: wakeup from lowpower to active");
      } else if (mssc_machine_state == MsscMachineState::MSSC_PROTECT) {
        ERROR(
          "[LowPower]: switch state error3:wake up, battery charge val:%d, current state:%d",
          battery_charge_val, mssc_machine_state);
        SwitchState(MsscMachineState::MSSC_ACTIVE);
        INFO("[LowPower]: wakeup from protect to active");
      } else if (mssc_machine_state == MsscMachineState::MSSC_ACTIVE) {
        return;
      }
    }
  }
  void BatteryChargeUpdate(uint8_t bc, bool is_charging)
  {
    battery_charge_val = bc;
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return;
    } else if (battery_charge_val <= 0 && (!is_charging)) {
      // 关机
      SwitchState(MsscMachineState::MSSC_SHUTDOWN);
    } else if (battery_charge_val < 5) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER || is_charging) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_PROTECT) {
        // 切换到低功耗模式
        SwitchState(MsscMachineState::MSSC_LOWPOWER);
        INFO("[LowPower]: call low power consumption when battery soc is less than 5");

      } else if (mssc_machine_state == MsscMachineState::MSSC_ACTIVE) {
        // 切换到低功耗模式
        SwitchState(MsscMachineState::MSSC_LOWPOWER);
        INFO("[LowPower]: call low power consumption when battery soc is less than 5");
      }
    } else if (battery_charge_val < 20) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_PROTECT) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_ACTIVE) {
        // 切换到保护模式
        SwitchState(MsscMachineState::MSSC_PROTECT);
      }
    } else {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_PROTECT) {
        SwitchState(MsscMachineState::MSSC_ACTIVE);
      } else if (mssc_machine_state == MsscMachineState::MSSC_ACTIVE) {
        return;
      }
    }
  }
  void ContinueKeepDown()
  {
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return;
    } else if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
      return;
    }
    SwitchState(MsscMachineState::MSSC_LOWPOWER);
  }

private:
  void MachineStateSwitch(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    SwitchState(static_cast<MsscMachineState>(msg->data));
  }
  void MachineStateGet(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    // SwitchState(static_cast<MsscMachineState>(msg->data));
    auto iter = mssc_machine_state_code_map.find(mssc_machine_state);
    if (iter == mssc_machine_state_code_map.end()) {
      response->message = "error-state";
    } else {
      response->message = mssc_machine_state_code_map.at(mssc_machine_state);
    }
    response->success = true;
  }
  void OtaMachineState(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        lowpower(false);
        INFO("[LowPower]: exit lowpower to ota");
      } else if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
        response->success = true;
        return;
      }
      SwitchState(MsscMachineState::MSSC_OTA);
    } else {
      SwitchState(MsscMachineState::MSSC_ACTIVE);
    }
    response->success = true;
  }
  void SwitchState(MsscMachineState mm)
  {
    std::lock_guard<std::mutex> lck(switch_mtx);
    mssc_machine_state = mm;
    is_switching_ms_ = true;
    switch (mssc_machine_state) {
      case MsscMachineState::MSSC_ACTIVE:
        {
          INFO("^^^ switch state:active ^^^");
          active_handler();
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 0;
          state_swith_status_pub_->publish(sss);
        }
        break;
      case MsscMachineState::MSSC_PROTECT:
        {
          INFO("^^^ switch state:protected ^^^");
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 1;
          state_swith_status_pub_->publish(sss);
          protect_handler();
        }
        break;
      case MsscMachineState::MSSC_LOWPOWER:
        {
          INFO("^^^ switch state:low-power ^^^");
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 2;
          state_swith_status_pub_->publish(sss);
          lowpower_handler();
          lowpower(true);
        }
        break;
      case MsscMachineState::MSSC_OTA:
        {
          INFO("^^^ switch state:ota ^^^");
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 3;
          state_swith_status_pub_->publish(sss);
          ota_handler();
        }
        break;
      case MsscMachineState::MSSC_SHUTDOWN:
        {
          INFO("^^^ switch state:shutdown ^^^");
          // need modify
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 4;
          state_swith_status_pub_->publish(sss);
          poweroff();
          shutdown_handler();
        }
        break;
      default:
        {
          INFO("^^^ switch state:unkown ^^^");
        }
        break;
    }
    is_switching_ms_ = false;
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

  bool lowpower(bool is_enter)
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
        return future_result.get()->success;
      } else {
        ERROR("[LowPower]: call low-power service failed!");
      }
    }
    return false;
  }

private:
  rclcpp::Node::SharedPtr mssc_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr mssc_callback_group_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_set_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_valget_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_ota_state_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr low_power_client_;
  rclcpp::Publisher<protocol::msg::StateSwitchStatus>::SharedPtr state_swith_status_pub_;
  MsscMachineState mssc_machine_state {MsscMachineState::MSSC_UNKOWN};
  std::mutex switch_mtx;
  BSSC_CALLBACK active_handler {[](void) {}};
  BSSC_CALLBACK protect_handler {[](void) {}};
  BSSC_CALLBACK lowpower_handler {[](void) {}};
  BSSC_CALLBACK ota_handler {[](void) {}};
  BSSC_CALLBACK shutdown_handler {[](void) {}};
  uint8_t battery_charge_val {100};
  const std::map<MsscMachineState, std::string> mssc_machine_state_code_map = {
    {MsscMachineState::MSSC_ACTIVE, "active"},
    {MsscMachineState::MSSC_PROTECT, "protected"},
    {MsscMachineState::MSSC_LOWPOWER, "low-power"},
    {MsscMachineState::MSSC_SHUTDOWN, "shutdown"},
    {MsscMachineState::MSSC_UNKOWN, "unkown"}
  };
  bool is_switching_ms_ {false};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__MACHINE_STATE_SWITCH_CONTEXT_HPP_
