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
#ifndef CYBERDOG_MANAGER__BATTERY_CAPACITY_INFO_HPP_
#define CYBERDOG_MANAGER__BATTERY_CAPACITY_INFO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{
// enum class BatteryMachineState : uint8_t
// {
//   BMS_NORMAL    = 0,    // 正常
//   BMS_PROTECT   = 1,    // 保护
//   BMS_LOWPOWER  = 2,    // 低功耗
//   BMS_UNKOWN    = 255,  // 未知
// };

class BatteryCapacityInfoNode final
{
  using BCIN_CALLBACK = std::function<void ()>;
  using BCINSOC_CALLBACK = std::function<void (uint8_t val, bool val2)>;

public:
  explicit BatteryCapacityInfoNode(
    rclcpp::Node::SharedPtr node_ptr,
    BCINSOC_CALLBACK bcin_soc)
  : battery_capacity_info_node_(node_ptr),
    batsoc_notify_handler(bcin_soc),
    is_protected(false), is_reported_charging(false)
  {
  }

public:
  void Init()
  {
    bc_callback_group_ =
      battery_capacity_info_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = bc_callback_group_;
    bms_status_sub_ = battery_capacity_info_node_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&BatteryCapacityInfoNode::BmsStatus, this, std::placeholders::_1),
      sub_options);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = bc_callback_group_;
    audio_play_extend_pub =
      battery_capacity_info_node_->create_publisher<protocol::msg::AudioPlayExtend>(
      "speech_play_extend",
      rclcpp::SystemDefaultsQoS(),
      pub_options
      );
  }

private:
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    static bool is_battary_zero = false;
    static bool is_set_count = false;
    bms_status_ = *msg;
    batsoc_notify_handler(bms_status_.batt_soc, bms_status_.power_wired_charging);
    if (bms_status_.power_wired_charging) {
      is_set_count = false;
      is_reported_charging = false;
      return;
    }
    INFO_MILLSECONDS(
      30000, "Battery Capacity Info:%d",
      bms_status_.batt_soc);
    if (!is_battary_zero && bms_status_.batt_soc <= 0) {
      is_battary_zero = true;
      protocol::msg::AudioPlayExtend msg;
      msg.is_online = true;
      msg.module_name = battery_capacity_info_node_->get_name();
      // msg.text = "电量低于5%,进入低电保护模式!请尽快充电";
      msg.text = "电量为0,关机中!";
      audio_play_extend_pub->publish(msg);
      // shutdown_handler();
    } else if (bms_status_.batt_soc <= 5) {
      if (!is_protected) {
        is_protected = true;
        // protect_handler();
        INFO(
          "Battery Capacity Info capacity:%d, %s lowpower mode.",
          bms_status_.batt_soc,
          (is_protected ? "enter" : "exit"));
        protocol::msg::AudioPlayExtend msg;
        msg.is_online = true;
        msg.module_name = battery_capacity_info_node_->get_name();
        // msg.text = "电量低于5%,进入低电保护模式!请尽快充电";
        msg.text = "电量低于5%，电池即将耗尽，请尽快充电!";
        audio_play_extend_pub->publish(msg);
      }
    } else if (bms_status_.batt_soc > 20) {
      if (is_protected) {
        is_protected = false;
        // active_handler();
        INFO(
          "Battery Capacity Info capacity:%d, %s protect mode.",
          bms_status_.batt_soc,
          (is_protected ? "enter" : "exit"));
        // protocol::msg::AudioPlayExtend msg;
        // msg.is_online = true;
        // msg.module_name = battery_capacity_info_node_->get_name();
        // msg.text = "电量高于20%,退出低电保护模式";
        // audio_play_extend_pub->publish(msg);
      }
    }
    if (!is_protected) {
      if (!is_reported_charging && (bms_status_.batt_soc < 20)) {
        is_reported_charging = true;
        protocol::msg::AudioPlayExtend msg;
        msg.is_online = true;
        msg.module_name = battery_capacity_info_node_->get_name();
        msg.text = "电量低于20%，部分功能受限";
        audio_play_extend_pub->publish(msg);
      } else if (bms_status_.batt_soc < 30) {
        if (!is_reported_charging) {
          is_reported_charging = true;
          protocol::msg::AudioPlayExtend msg;
          msg.is_online = true;
          msg.module_name = battery_capacity_info_node_->get_name();
          msg.text = "电量低于30%，请尽快充电";
          audio_play_extend_pub->publish(msg);
        }
        if (bms_status_.batt_soc < 20) {
          if (!is_set_count && is_reported_charging) {
            is_set_count = true;
            protocol::msg::AudioPlayExtend msg;
            msg.is_online = true;
            msg.module_name = battery_capacity_info_node_->get_name();
            msg.text = "电量低于20%，部分功能受限";
            audio_play_extend_pub->publish(msg);
          }
        }
      } else if (bms_status_.batt_soc > 35) {
        is_reported_charging = false;
        is_set_count = false;
      }
    }
  }

private:
  rclcpp::Node::SharedPtr battery_capacity_info_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr bc_callback_group_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_extend_pub;
  protocol::msg::BmsStatus bms_status_;
  // BCIN_CALLBACK protect_handler;
  // BCIN_CALLBACK lowpower_handler;
  // BCIN_CALLBACK active_handler;
  // BCIN_CALLBACK shutdown_handler;
  BCINSOC_CALLBACK batsoc_notify_handler;
  bool is_protected;
  bool is_reported_charging;
  // BatteryMachineState bms;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__BATTERY_CAPACITY_INFO_HPP_
