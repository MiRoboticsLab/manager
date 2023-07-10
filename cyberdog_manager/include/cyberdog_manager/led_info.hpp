// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#ifndef CYBERDOG_MANAGER__LED_INFO_HPP_
#define CYBERDOG_MANAGER__LED_INFO_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/motion_status.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{
struct LedMode
{
  bool occupation;
  std::string client;
  uint8_t target;
  uint8_t mode;
  uint8_t effect;
  uint8_t r_value;
  uint8_t g_value;
  uint8_t b_value;
};

class LedInfoNode final
{
public:
  explicit LedInfoNode(rclcpp::Node::SharedPtr node_ptr)
  : led_info_node_(node_ptr)
  {
    led_callback_group_ =
      led_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    led_excute_client_ =
      led_info_node_->create_client<protocol::srv::LedExecute>(
      "led_execute",
      rmw_qos_profile_services_default, led_callback_group_);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = led_callback_group_;
  }

  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    power_charging_ = msg->power_wired_charging;
    battery_soc_ = msg->batt_soc;
    static int start_battery_soc = battery_soc_;
    int charging_status_switch = false;

    // 充电状态切换flag
    if (pre_charging_status_ != power_charging_) {
      charging_status_switch = true;
    }
    // 非充电状态，Bms释放灯效
    if (battery_soc_ > 20) {
      if (!power_charging_ && !bms_occupied_led_) {
        bms_occupied_led_ = true;
        LedMode head{false, "bms", 1, 0x02, 0x09, 0xFF, 0x32, 0x32};
        LedMode tail{false, "bms", 2, 0x02, 0x09, 0xFF, 0x32, 0x32};
        LedMode mini{false, "bms", 3, 0x02, 0x30, 0xFF, 0x32, 0x32};
        bool result = ReqLedService(head, tail, mini);
        INFO("Bms %s to release led occupation ", result ? "successed" : "failed");
      }
      if (power_charging_) {
        bms_occupied_led_ = false;
      }
    }

    if ((pre_battery_soc_ == 1 && battery_soc_ == 0) || start_battery_soc == 0) {
      if (!power_charging_) {
        LedMode poweroff_head{true, "bms", 1, 0x01, 0xA3, 0x00, 0x00, 0x00};
        LedMode poweroff_tail{true, "bms", 2, 0x01, 0xA3, 0x00, 0x00, 0x00};
        LedMode poweroff_mini{true, "bms", 3, 0x02, 0x31, 0xFF, 0x00, 0x00};
        bool result = ReqLedService(poweroff_head, poweroff_tail, poweroff_mini);
        INFO("%s set led when the soc is 0", result ? "successed" : "failed");
      }
    }

    if ((pre_battery_soc_ == 21 && battery_soc_ == 20) ||
      (start_battery_soc > 0 && start_battery_soc <= 20) ||
      (charging_status_switch && battery_soc_ <= 20))
    {
      if (power_charging_) {
        LedMode head{true, "bms", 1, 0x02, 0x06, 0xFF, 0x32, 0x32};
        LedMode tail{true, "bms", 2, 0x02, 0x06, 0xFF, 0x32, 0x32};
        LedMode mini{true, "bms", 3, 0x02, 0x30, 0xFF, 0x00, 0x00};
        bool result = ReqLedService(head, tail, mini);
        INFO("%s set the charging led when the soc less than 20", result ? "successed" : "failed");
      } else {
        LedMode head{true, "bms", 1, 0x02, 0x09, 0xFF, 0x32, 0x32};
        LedMode tail{true, "bms", 2, 0x02, 0x09, 0xFF, 0x32, 0x32};
        LedMode mini{true, "bms", 3, 0x02, 0x30, 0xFF, 0x00, 0x00};
        bool result = ReqLedService(head, tail, mini);
        INFO("%s set led when the soc less than 20", result ? "successed" : "failed");
      }
    }

    if ((pre_battery_soc_ == 20 && battery_soc_ == 21) ||
      (pre_battery_soc_ == 80 && battery_soc_ == 79) ||
      (start_battery_soc > 20 && start_battery_soc < 80) ||
      (charging_status_switch && battery_soc_ > 20 && battery_soc_ < 80))
    {
      if (power_charging_) {
        LedMode head{true, "bms", 1, 0x02, 0x06, 0x75, 0xFC, 0xF6};
        LedMode tail{true, "bms", 2, 0x02, 0x06, 0x75, 0xFC, 0xF6};
        LedMode mini{true, "bms", 3, 0x02, 0x30, 0x75, 0xFC, 0xF6};
        bool result = ReqLedService(head, tail, mini);
        INFO(
          "%s set the charging less, the soc more than 20 and less than 80 with charing",
          result ? "successed" : "failed");
      } else {
        // 释放Bms灯效
        LedMode bringup_head{false, "bms", 1, 0x02, 0x09, 0xFF, 0x32, 0x32};
        LedMode bringup_tail{false, "bms", 2, 0x02, 0x09, 0xFF, 0x32, 0x32};
        LedMode bringup_mini{false, "bms", 3, 0x02, 0x30, 0xFF, 0x32, 0x32};
        bool result = ReqLedService(bringup_head, bringup_tail, bringup_mini);
        INFO("bms %s release led when the soc less than 80", result ? "successed" : "failed");
      }
      // 更改系统灯效
      LedMode sys_bringup_head{true, "system", 1, 0x02, 0x09, 0x75, 0xFC, 0xF6};
      LedMode sys_bringup_tail{true, "system", 2, 0x02, 0x09, 0x75, 0xFC, 0xF6};
      LedMode sys_bringup_mini{true, "system", 3, 0x02, 0x30, 0x75, 0xFC, 0xF6};
      bool result2 = ReqLedService(sys_bringup_head, sys_bringup_tail, sys_bringup_mini);
      INFO(
        "%s set sys led, the soc more than 20 an less than 80",
        result2 ? "successed" : "failed");
    }

    if ((pre_battery_soc_ == 79 && battery_soc_ == 80) ||
      start_battery_soc >= 80 || (charging_status_switch && battery_soc_ >= 80))
    {
      // 更改系统灯效
      LedMode bringup_head{true, "system", 1, 0x02, 0x09, 0x06, 0x21, 0xE2};
      LedMode bringup_tail{true, "system", 2, 0x02, 0x09, 0x06, 0x21, 0xE2};
      LedMode bringup_mini{true, "system", 3, 0x02, 0x30, 0x06, 0x21, 0xE2};
      bool result = ReqLedService(bringup_head, bringup_tail, bringup_mini);
      INFO("%s set sys led, the soc more than 80", result ? "successed" : "failed");
      if (power_charging_) {
        LedMode bringup_head{true, "bms", 1, 0x02, 0x06, 0x06, 0x21, 0xE2};
        LedMode bringup_tail{true, "bms", 2, 0x02, 0x06, 0x06, 0x21, 0xE2};
        LedMode bringup_mini{true, "bms", 3, 0x02, 0x30, 0x06, 0x21, 0xE2};
        bool result = ReqLedService(bringup_head, bringup_tail, bringup_mini);
        INFO(
          "%s set the charging, the soc more than 80 with charing",
          result ? "successed" : "failed");
      }
    }
    start_battery_soc = -1;
    pre_battery_soc_ = battery_soc_;
    pre_charging_status_ = power_charging_;
  }

private:
  void ReqAssignment(std::shared_ptr<protocol::srv::LedExecute::Request> req, LedMode & data)
  {
    req->occupation = data.occupation;
    req->client = data.client;
    req->target = data.target;
    req->mode = data.mode;
    req->effect = data.effect;
    req->r_value = data.r_value;
    req->g_value = data.g_value;
    req->b_value = data.b_value;
  }

  bool ReqLedService(LedMode & head, LedMode & tail, LedMode & mini)
  {
    if (!led_excute_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call led_excute server not avalible");
      return false;
    }

    auto request_led = std::make_shared<protocol::srv::LedExecute::Request>();
    ReqAssignment(request_led, head);
    auto future_result_head = led_excute_client_->async_send_request(request_led);
    std::future_status status_head = future_result_head.wait_for(std::chrono::seconds(2));

    ReqAssignment(request_led, tail);
    auto future_result_tail = led_excute_client_->async_send_request(request_led);
    std::future_status status_tail = future_result_head.wait_for(std::chrono::seconds(2));

    ReqAssignment(request_led, mini);
    auto future_result_mini = led_excute_client_->async_send_request(request_led);
    std::future_status status_mini = future_result_mini.wait_for(std::chrono::seconds(2));

    if (status_head != std::future_status::ready ||
      status_tail != std::future_status::ready ||
      status_mini != std::future_status::ready)
    {
      INFO("call led_execute service failed");
      return false;
    }

    if (future_result_head.get()->code == 0 &&
      future_result_tail.get()->code == 0 &&
      future_result_mini.get()->code == 0)
    {
      INFO("call led service successed");
      return true;
    } else {
      INFO(
        "call led service fialed, error code[head, tail, mini] is:%d %d %d",
        future_result_head.get()->code,
        future_result_tail.get()->code,
        future_result_mini.get()->code);
      return false;
    }
  }

  bool ReqLedService(LedMode & tail)
  {
    if (!led_excute_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call led_excute server not avalible");
      return false;
    }
    auto request_led = std::make_shared<protocol::srv::LedExecute::Request>();
    ReqAssignment(request_led, tail);
    auto future_result_tail = led_excute_client_->async_send_request(request_led);
    std::future_status status_tail = future_result_tail.wait_for(std::chrono::seconds(2));

    if (status_tail != std::future_status::ready) {
      INFO("call led_execute service failed");
      return false;
    }

    if (future_result_tail.get()->code == 0) {
      INFO("call led service successed");
      return true;
    } else {
      INFO(
        "call led service fialed, error code[tail] is: %d",
        future_result_tail.get()->code);
      return false;
    }
  }

private:
  rclcpp::Node::SharedPtr led_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr led_callback_group_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wake_up_sub_ {nullptr};
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_excute_client_ {nullptr};
  uint8_t battery_soc_ {100};
  uint8_t pre_battery_soc_ {0};
  bool power_charging_ {false};
  bool pre_charging_status_ {false};
  bool bms_occupied_led_ {false};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__LED_INFO_HPP_
