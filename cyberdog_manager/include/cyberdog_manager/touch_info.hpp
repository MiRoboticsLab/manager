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
#ifndef CYBERDOG_MANAGER__TOUCH_INFO_HPP_
#define CYBERDOG_MANAGER__TOUCH_INFO_HPP_

#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/msg/touch_status.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/audio_play_extend.hpp"

namespace cyberdog
{
namespace manager
{
class TouchInfoNode final
{
public:
  explicit TouchInfoNode(rclcpp::Node::SharedPtr node_ptr)
  {
    touch_info_node_ = node_ptr;
    touch_callback_group_ =
      touch_info_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;

    sub_options.callback_group = touch_callback_group_;
    touch_status_sub_ = touch_info_node_->create_subscription<protocol::msg::TouchStatus>(
      "touch_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&TouchInfoNode::ReportPower, this, std::placeholders::_1), sub_options);

    bms_status_sub_ = touch_info_node_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&TouchInfoNode::BmsStatus, this, std::placeholders::_1),
      sub_options);

    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = touch_callback_group_;
    audio_play_extend_pub =
      touch_info_node_->create_publisher<protocol::msg::AudioPlayExtend>(
      "speech_play_extend", rclcpp::SystemDefaultsQoS(), pub_options);
  }

private:
  void ReportPower(const protocol::msg::TouchStatus::SharedPtr msg)
  {
    INFO_MILLSECONDS(30000, "[cyberdog_manager:] touch_info-->> Enter TouchStatus callback!!");
    std::chrono::milliseconds report_previous_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
    INFO("report_previous_time is %d", report_previous_time.count());
    if (msg->touch_state == 3 &&
      abs(reporting_time.count() - report_previous_time.count()) >= 3000)
    {
      INFO(
        "reporting_time - report_previous_time is %d",
        abs(reporting_time.count() - report_previous_time.count()));
      reporting_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
      INFO("[cyberdog_manager:] touch_info-->> the batt_soc is %d", battery_percent);
      protocol::msg::AudioPlayExtend msg;
      msg.is_online = true;
      msg.module_name = touch_info_node_->get_name();
      msg.text = "剩余电量为百分之" + std::to_string(battery_percent);
      INFO("语言播报：%s", msg.text.c_str());
      audio_play_extend_pub->publish(msg);
    }
  }

  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    battery_percent = msg->batt_soc;
    INFO_MILLSECONDS(
      30000,
      "[cyberdog_manager:] touch_info-->> Enter BMS callback.the batt_soc is %d.",
      battery_percent);
  }

private:
  rclcpp::Node::SharedPtr touch_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr touch_callback_group_;
  rclcpp::Subscription<protocol::msg::TouchStatus>::SharedPtr touch_status_sub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_extend_pub;
  int battery_percent;
  std::chrono::milliseconds reporting_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch());
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__TOUCH_INFO_HPP_
