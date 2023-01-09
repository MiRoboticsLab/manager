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
// using SHINE_CALLBACK = std::function<void (uint8_t id)>;

public:
  // explicit LedInfoNode(rclcpp::Node::SharedPtr node_ptr, SHINE_CALLBACK callback)
  explicit LedInfoNode(rclcpp::Node::SharedPtr node_ptr)
  : led_info_node_(node_ptr)
    // ,light_shine_handler(callback)
  {
    led_callback_group_ =
      led_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    led_excute_client_ =
      led_info_node_->create_client<protocol::srv::LedExecute>(
      "led_execute",
      rmw_qos_profile_services_default, led_callback_group_);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = led_callback_group_;
    bms_status_sub_ = led_info_node_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&LedInfoNode::BmsStatus, this, std::placeholders::_1),
      sub_options);

    wake_up_sub_ = led_info_node_->create_subscription<std_msgs::msg::Bool>(
      "dog_wakeup", rclcpp::SystemDefaultsQoS(),
      std::bind(&LedInfoNode::WakeUp, this, std::placeholders::_1),
      sub_options);

    // motion_status sub
    motion_status_sub_ = led_info_node_->create_subscription<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&LedInfoNode::Motion_status, this, std::placeholders::_1),
      sub_options);
  }

private:
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    battery_soc_ = msg->batt_soc;
    static bool is_set_led_zero = false;
    static bool is_set_led_five = false;
    static bool is_set_led_twenty = false;

    if (!is_set_led_zero && battery_soc_ <= 0) {
      if (!msg->power_wired_charging) {
        is_set_led_zero = true;
        LedMode poweroff_head{true, "bms", 1, 0x01, 0xA3, 0x00, 0x00, 0x00};
        LedMode poweroff_tail{true, "bms", 2, 0x01, 0xA3, 0x00, 0x00, 0x00};
        LedMode poweroff_mini{true, "bms", 3, 0x01, 0x31, 0xFF, 0x32, 0x32};
        bool result = ReqLedService(poweroff_head, poweroff_tail, poweroff_mini);
        INFO("%s set led when the soc is 0", result ? "successed" : "failed");
      }
    } else if (!is_set_led_five && battery_soc_ < 5) {
      if (!msg->power_wired_charging) {
        is_low_power_ = true;
        is_set_led_five = true;
        is_set_led_twenty = false;
        LedMode low_power_tail{true, "lowpower", 2, 0x01, 0xA0, 0x00, 0x00, 0x00};
        bool result = ReqLedService(low_power_tail);
        INFO("[LowPower]: %s set led when the soc is less than 5", result ? "successed" : "failed");
      }
    } else if (!is_set_led_twenty && battery_soc_ >= 5 && battery_soc_ <= 20) {
      is_set_led_five = false;
      is_set_led_twenty = true;
      LedMode bringup_head{true, "bms", 1, 0x02, 0x09, 0xFF, 0x32, 0x32};
      LedMode bringup_tail{true, "bms", 2, 0x02, 0x09, 0xFF, 0x32, 0x32};
      LedMode bringup_mini{true, "bms", 3, 0x02, 0x30, 0xFF, 0x32, 0x32};
      bool result = ReqLedService(bringup_head, bringup_tail, bringup_mini);
      INFO("%s set led when the soc more than 30", result ? "successed" : "failed");
    }
  }

  void Motion_status(const protocol::msg::MotionStatus::SharedPtr msg)
  {
    // motion_id: 趴下(101)、站立(111)
    // static bool is_low_power {false};
    static bool is_convert_motion_status {false};
    static int lay_count {0};

    // 运动状态转换至趴下，30s后启动低功耗, 关闭尾灯
    if (is_convert_motion_status == true && msg->motion_id == 101) {
      ++lay_count;
      if (lay_count == 300) {
        INFO("[LowPower]: start to trun off tail led when dog lies down for 30s");
        LedMode low_power_tail{true, "lowpower", 2, 0x01, 0xA0, 0x00, 0x00, 0x00};
        bool result = ReqLedService(low_power_tail);
        INFO(
          "[LowPower]: %s trun off tail led when enter low power ",
          result ? "successed" : "failed");
        is_convert_motion_status = false;
        lay_count = 0;
        is_low_power_ = true;
      }
    } else {
      lay_count = 0;
    }

    // 状态切换到站立
    if (is_convert_motion_status == false && msg->motion_id == 111) {
      is_convert_motion_status = true;
      if (is_low_power_) {
        is_low_power_ = false;
      }
    }
  }

  void WakeUp(const std_msgs::msg::Bool msg)
  {
    (void)msg;
    // 刷新灯效
    if (is_low_power_) {
      if (battery_soc_ < 5) {
        INFO("[LowPower]:turn on tail led rejected, batter soc less than 5");
        return;
      }

      LedMode low_power_tail{false, "lowpower", 2, 0x01, 0xA0, 0x00, 0x00, 0x00};
      bool result = ReqLedService(low_power_tail);
      INFO("[LowPower]: %s turn on tail led when wakeup", result ? "successed" : "failed");
      is_low_power_ = false;
    }
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
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_ {nullptr};
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wake_up_sub_ {nullptr};
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_excute_client_ {nullptr};
  uint8_t battery_soc_ {100};
  bool is_low_power_ {false};

  // SHINE_CALLBACK light_shine_handler {[](uint8_t) {}};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__LED_INFO_HPP_
