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

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/srv/audio_text_play.hpp"
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
  using BCINBMS_CALLBACK = std::function<void (const protocol::msg::BmsStatus::SharedPtr)>;

public:
  explicit BatteryCapacityInfoNode(
    rclcpp::Node::SharedPtr node_ptr,
    BCINSOC_CALLBACK bcin_soc,
    BCINBMS_CALLBACK bcin_bms)
  : battery_capacity_info_node_(node_ptr),
    batsoc_notify_handler(bcin_soc),
    bms_notify_handler(bcin_bms),
    is_soc_zero_(false), is_soc_five_(false), is_soc_twenty_(false), is_soc_thirty_(false)
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
      rclcpp::SystemDefaultsQoS(), pub_options);
    audio_play_client_ = battery_capacity_info_node_->create_client<protocol::srv::AudioTextPlay>(
      "speech_text_play",
      rmw_qos_profile_services_default, bc_callback_group_);
  }

private:
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    bms_status_ = *msg;
    bms_notify_handler(msg);
    INFO_MILLSECONDS(
      30000, "Battery Capacity Info:%d",
      bms_status_.batt_soc);
    if (!bms_status_.power_wired_charging) {
      if (bms_status_.batt_soc <= 0) {
        if (!is_soc_zero_) {
          is_soc_zero_ = true;
          // is_soc_five_ = false;
          PlayAudioService("电量为0,关机中!");
        }
      } else if (bms_status_.batt_soc <= 5) {
        if (!is_soc_five_) {
          is_soc_five_ = true;
          is_soc_twenty_ = false;
          PlayAudio("电量低于5%，电池即将耗尽，请尽快充电!");
        }
      } else if (bms_status_.batt_soc <= 20) {
        if (!is_soc_twenty_) {
          is_soc_twenty_ = true;
          is_soc_five_ = false;
          is_soc_thirty_ = false;
          PlayAudio("电量低于20%，部分功能受限!");
        }
      } else if (bms_status_.batt_soc <= 30) {
        if (!is_soc_thirty_) {
          is_soc_thirty_ = true;
          is_soc_twenty_ = false;
          PlayAudio("电量低于30%，请尽快充电!");
        }
      }
    } else {
      is_soc_twenty_ = false;
      is_soc_thirty_ = false;
      is_soc_five_ = false;
    }
    batsoc_notify_handler(bms_status_.batt_soc, bms_status_.power_wired_charging);
  }

  void PlayAudio(const std::string & text)
  {
    protocol::msg::AudioPlayExtend msg;
    msg.is_online = true;
    msg.module_name = battery_capacity_info_node_->get_name();
    msg.text = text;
    audio_play_extend_pub->publish(msg);
  }

  void PlayAudioService(const std::string & text)
  {
    auto request_audio = std::make_shared<protocol::srv::AudioTextPlay::Request>();
    request_audio->module_name = battery_capacity_info_node_->get_name();
    request_audio->is_online = true;
    request_audio->text = text;
    auto callback = [](rclcpp::Client<protocol::srv::AudioTextPlay>::SharedFuture future) {
        INFO("Audio play result: %s", future.get()->status == 0 ? "success" : "failed");
      };
    auto future_result_audio = audio_play_client_->async_send_request(request_audio, callback);
    if (future_result_audio.wait_for(std::chrono::milliseconds(3000)) ==
      std::future_status::timeout)
    {
      ERROR("Cannot get response from AudioPlay");
    }
  }

private:
  rclcpp::Node::SharedPtr battery_capacity_info_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr bc_callback_group_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_extend_pub;
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_ {nullptr};
  protocol::msg::BmsStatus bms_status_;
  BCINSOC_CALLBACK batsoc_notify_handler;
  BCINBMS_CALLBACK bms_notify_handler;
  bool is_soc_zero_;
  bool is_soc_five_;
  bool is_soc_twenty_;
  bool is_soc_thirty_;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__BATTERY_CAPACITY_INFO_HPP_
