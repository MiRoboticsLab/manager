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
#ifndef CYBERDOG_MANAGER__AUDIO_INFO_HPP_
#define CYBERDOG_MANAGER__AUDIO_INFO_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{

class AudioInfoNode final
{
  using AudioMsg = protocol::msg::AudioPlayExtend;
  using WAKEUP_CALLBACK = std::function<void ()>;

public:
  explicit AudioInfoNode(rclcpp::Node::SharedPtr node_ptr)
  : audio_info_node_(node_ptr), wakeup_handler(callback)
  {
    audio_callback_group_ =
      audio_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = audio_callback_group_;
    audio_play_pub_ =
      audio_info_node_->create_publisher<AudioMsg>(
      "speech_play_extend", rclcpp::SystemDefaultsQoS(), pub_options);
    audio_text_play_client_ =
      audio_info_node_->create_client<protocol::srv::AudioTextPlay>(
      "speech_text_play",
      rmw_qos_profile_services_default, audio_callback_group_);
  }

  void Init()
  {
    if (!init_) {
      // AudioMsg msg;
      // msg.module_name = audio_info_node_->get_name();
      // msg.is_online = false;
      // msg.speech.play_id = protocol::msg::AudioPlay::PID_SOUND_EFFECT_READY;
      // audio_play_pub_->publish(msg);
      INFO("...now play sound effect ready...");
      std::chrono::seconds timeout(6);
      auto req = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      req->module_name = audio_info_node_->get_name();
      req->is_online = false;
      req->speech.play_id = protocol::msg::AudioPlay::PID_SOUND_EFFECT_READY;
      auto future_result = audio_text_play_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
      } else {
        ERROR("call play sound service failed!");
      }
      AudioMsg text_msg;
      text_msg.module_name = audio_info_node_->get_name();
      text_msg.is_online = false;
      text_msg.speech.play_id = protocol::msg::AudioPlay::PID_SELF_CHECK_SUCCESS;
      text_msg.text = "自检完成,状态就绪!";
      audio_play_pub_->publish(text_msg);
      rclcpp::SubscriptionOptions sub_options;
      sub_options.callback_group = audio_callback_group_;
      wake_up_sub_ =
        audio_info_node_->create_subscription<std_msgs::msg::Bool>(
        "dog_wakeup", rclcpp::SystemDefaultsQoS(),
        std::bind(&AudioInfoNode::DogWakeup, this, std::placeholders::_1),
        sub_options);
      init_ = true;
    }
  }

  void Error(std::string error_text)
  {
    AudioMsg text_msg;
    text_msg.module_name = audio_info_node_->get_name();
    text_msg.is_online = false;
    text_msg.speech.play_id = protocol::msg::AudioPlay::PID_SELF_CHECK_FAILED;
    text_msg.text = error_text;
    audio_play_pub_->publish(text_msg);
  }

private:
  void DogWakeup(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      INFO("[LowPower]: dog wakeup...");
      // wakeup_handler();
    }
  }

private:
  rclcpp::Node::SharedPtr audio_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr audio_callback_group_;
  rclcpp::Publisher<AudioMsg>::SharedPtr audio_play_pub_ {nullptr};
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_text_play_client_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wake_up_sub_ {nullptr};
  // WAKEUP_CALLBACK wakeup_handler {[](void) {}};
  bool init_ {false};
};

}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__AUDIO_INFO_HPP_
