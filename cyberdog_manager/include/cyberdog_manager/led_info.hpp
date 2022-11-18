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

#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/msg/bms_status.hpp"

namespace cyberdog
{
namespace manager
{
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
  }

private:
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
  }

private:
  rclcpp::Node::SharedPtr led_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr led_callback_group_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_excute_client_;
  // SHINE_CALLBACK light_shine_handler {[](uint8_t) {}};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__LED_INFO_HPP_
