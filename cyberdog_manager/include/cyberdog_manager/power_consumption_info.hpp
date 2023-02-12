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
  PMS_NORMAL    = 0,    // 正常
  PMS_PROTECT   = 1,    // 保护
  PMS_LOWPOWER  = 2,    // 低功耗
  PMS_UNKOWN    = 255,  // 未知
};

class PowerConsumptionInfoNode final
{
  using PCIN_CALLBACK = std::function<void ()>;

public:
  explicit PowerConsumptionInfoNode(rclcpp::Node::SharedPtr node_ptr, PCIN_CALLBACK callback)
  : enter_lowpower_handler(callback)
  {
    power_consumption_info_node_ = node_ptr;
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
    power_consumption_callback_group_ =
      power_consumption_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    power_off_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::Trigger>(
      "poweroff",
      std::bind(
        &PowerConsumptionInfoNode::ShutdownCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    reboot_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::Trigger>(
      "reboot",
      std::bind(
        &PowerConsumptionInfoNode::RebootCallback, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    motion_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_mostion_status_callback,
        this, std::placeholders::_1), options);
    state_swith_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_state_switch_status_callback,
        this, std::placeholders::_1), options);
  }

  bool EnterLowPower(bool is_enter)
  {
    static int r_count = 0;
    PM_DEV pd = PM_ALL_NO_TOF;
    unsigned int err;
    int code = -1;
    bool result = false;
    if (is_enter) {
      INFO("[LowPower]: [%d]LowPower enter inner-call:start", (r_count + 1));
      code = lpc_ptr_->LpcRelease(pd, &err);
      if (code == 0) {
        is_lowpower_ = true;
        INFO("[LowPower]: [%d]LowPower enter inner-call:success", (r_count + 1));
        result = true;
      } else {
        INFO("[LowPower]: [%d]LowPower enter inner-call:failed!", (r_count + 1));
      }
      ++r_count;
    } else {
      INFO("[LowPower]: [%d]LowPower exit inner-call:start", (r_count + 1));
      code = lpc_ptr_->LpcRequest(pd, &err);
      if (code == 0) {
        is_lowpower_ = false;
        INFO("[LowPower]: [%d]LowPower exit inner-call:success", (r_count + 1));
        result = true;
      } else {
        INFO("[LowPower]: [%d]LowPower exit inner-call:failed!", (r_count + 1));
      }
      ++r_count;
    }
    return result;
  }

private:
  void sub_mostion_status_callback(const protocol::msg::MotionStatus::SharedPtr msg)
  {
    if (is_ota_) {
      INFO_MILLSECONDS(10000, "[LowPower]: in ota state return.");
      return;
    }
    // motion_id: 趴下(101)、站立(111)
    int motion_id = msg->motion_id;
    static int lay_count = 0;
    static int times = 0;

    if (!is_lowpower_) {
      if (motion_id == 0) {
        ++lay_count;
        if (lay_count == 1200) {
          INFO("[LowPower]: enter lowpower, start up time is greater than 2min");
          enter_lowpower_handler();
          ++times;
        }
      } else if (motion_id == 101) {
        ++lay_count;
        if (times == 0) {
          if (lay_count == 300) {
            INFO("[LowPower]: call low power consumption when the dog lies down for 30s");
            enter_lowpower_handler();
            ++times;
          }
        } else {
          if (lay_count == 1200) {
            INFO("[LowPower]: call low power consumption when the dog lies down for 2min");
            enter_lowpower_handler();
          }
        }
      } else {
        times = 0;
        lay_count = 0;
      }
    } else {
      lay_count = 0;
    }
  }

  void sub_state_switch_status_callback(const protocol::msg::StateSwitchStatus::SharedPtr msg)
  {
    if (msg->state == 3) {
      is_ota_ = true;
    } else {
      is_ota_ = false;
    }
  }

  void RebootCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    INFO("[PowerConsumption]: reboot......");
    PM_SYS pd = PM_SYS_REBOOT;
    int code = -1;
    code = lpc_ptr_->LpcSysRequest(pd);
    if (code != 0) {
      INFO("[PowerConsumption]: reboot failed, function LpcRequest call faild");
    } else {
      INFO("[PowerConsumption]: reboot successfully");
    }
    response->success = (code == 0 ? true : false);
  }

  void ShutdownCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    INFO("[PowerConsumption]: poweroff......");
    PM_SYS pd = PM_SYS_SHUTDOWN;
    int code = -1;
    code = lpc_ptr_->LpcSysRequest(pd);
    if (code != 0) {
      INFO("[PowerConsumption]: Shutdown failed, function LpcRequest call faild");
    } else {
      INFO("[PowerConsumption]: Shutdown successfully");
    }
    response->success = (code == 0 ? true : false);
  }

private:
  rclcpp::Node::SharedPtr power_consumption_info_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr power_consumption_callback_group_;
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_power_consumption_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_ {nullptr};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_ {nullptr};
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_ {nullptr};
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  rclcpp::Subscription<protocol::msg::StateSwitchStatus>::SharedPtr \
    state_swith_status_sub_ {nullptr};
  PCIN_CALLBACK request_handler;
  PCIN_CALLBACK release_handler;
  PowerMachineState pms {PowerMachineState::PMS_UNKOWN};
  PCIN_CALLBACK enter_lowpower_handler;
  bool is_lowpower_ {false};
  bool is_ota_ {false};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
