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

#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/audio_play.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{

class AudioInfoNode final
{
  using AudioMsg = protocol::msg::AudioPlay;

public:
  explicit AudioInfoNode(rclcpp::Node::SharedPtr node_ptr)
  {
    audio_info_node_ = node_ptr;
    audio_callback_group_ =
      audio_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = audio_callback_group_;
    audio_play_pub_ =
      audio_info_node_->create_publisher<AudioMsg>(
      "speech_play", rclcpp::SystemDefaultsQoS(), pub_options);
  }

  void Init()
  {
    AudioMsg msg;
    msg.module_name = audio_info_node_->get_name();
    msg.play_id = AudioMsg::PID_SOUND_EFFECT_READY;
    audio_play_pub_->publish(msg);
    INFO("...now play sound effect ready...");
  }

private:
  rclcpp::Node::SharedPtr audio_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr audio_callback_group_;
  rclcpp::Publisher<AudioMsg>::SharedPtr audio_play_pub_ {nullptr};
};

}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__AUDIO_INFO_HPP_
