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

#include <string>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/led_execute.hpp"
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

    led_excute_client_ =
      power_consumption_info_node_->create_client<protocol::srv::LedExecute>(
      "led_execute",
      rmw_qos_profile_services_default, power_consumption_callback_group_);

    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    state_swith_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_state_switch_status_callback,
        this, std::placeholders::_1), options);
  }

  void Init()
  {
    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    motion_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_mostion_status_callback,
        this, std::placeholders::_1), options);
    start = std::chrono::steady_clock::now();
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
      TailLedControl(true);
      code = lpc_ptr_->LpcRelease(pd, &err);
      if (code == 0) {
        is_lowpower_ = true;
        INFO("[LowPower]: [%d]LowPower enter inner-call:success", (r_count + 1));
        result = true;
      } else {
        INFO(
          "[LowPower]: [%d]LowPower enter inner-call:failed! error code is %d",
          (r_count + 1), code);
      }
      ++r_count;
    } else {
      INFO("[LowPower]: [%d]LowPower exit inner-call:start", (r_count + 1));
      code = lpc_ptr_->LpcRequest(pd, &err);
      if (code == 0) {
        start = std::chrono::steady_clock::now();
        is_lowpower_ = false;
        TailLedControl(false);
        INFO("[LowPower]: [%d]LowPower exit inner-call:success", (r_count + 1));
        result = true;
      } else {
        INFO(
          "[LowPower]: [%d]LowPower exit inner-call:failed! error code is %d",
          (r_count + 1), code);
      }
      ++r_count;
    }
    return result;
  }

private:
  void sub_mostion_status_callback(const protocol::msg::MotionStatus::SharedPtr msg)
  {
    // 状态机切换也有判断
    if (is_ota_) {
      INFO_MILLSECONDS(10000, "[LowPower]: in ota state return.");
      return;
    }
    // motion_id: 趴下(101)、站立(111)
    int motion_id = msg->motion_id;
    if (is_lowpower_) {
      return;
    }

    // static auto start = std::chrono::steady_clock::now();
    if (motion_id == 0 || motion_id == 101) {
      auto end = std::chrono::steady_clock::now();
      auto diff = std::chrono::duration_cast<std::chrono::seconds>(end - start);
      if (diff.count() >= enter_lowpower_time_) {
        // INFO("[LowPower]: enter lowpower, get down time is greater than 2min");
        enter_lowpower_handler();
        start = std::chrono::steady_clock::now();
      }
    } else {
      start = std::chrono::steady_clock::now();
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
      return;
    }
    if (future_result_tail.get()->code == 0) {
      INFO("call led service successed");
    } else {
      ERROR(
        "control tail led fialed, error code is:%d", future_result_tail.get()->code);
    }
  }

private:
  rclcpp::Node::SharedPtr power_consumption_info_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr power_consumption_callback_group_;
  // rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_power_consumption_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reboot_srv_ {nullptr};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_off_srv_ {nullptr};
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_excute_client_;
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
  int enter_lowpower_time_ {30};
  std::chrono::steady_clock::time_point start;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
