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
#ifndef CYBERDOG_MANAGER__AUDIO_INFO_HPP_
#define CYBERDOG_MANAGER__AUDIO_INFO_HPP_

#include <memory>
#include <string>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/srv/sdcard_play_id_query.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{

class AudioInfoNode final
{
  using AudioMsg = protocol::msg::AudioPlayExtend;

public:
  explicit AudioInfoNode(rclcpp::Node::SharedPtr node_ptr)
  : audio_info_node_(node_ptr)
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
    sdcard_playid_query_client_ =
      audio_info_node_->create_client<protocol::srv::SdcardPlayIdQuery>(
      "SdcardPlayIdQuery",
      rmw_qos_profile_services_default, audio_callback_group_);
  }

  void Init()
  {
    if (!init_) {
      INFO("...now play sound effect ready...");
      std::chrono::seconds timeout(6);
      auto req = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      req->module_name = audio_info_node_->get_name();
      req->is_online = false;
      req->speech.play_id = protocol::msg::AudioPlay::PID_SOUND_EFFECT_READY;
      auto future_result = audio_text_play_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        ERROR("call play sound service success.");
      } else {
        ERROR("call play sound service failed!");
      }
      AudioMsg text_msg;
      text_msg.module_name = audio_info_node_->get_name();
      text_msg.is_online = false;
      text_msg.speech.play_id = protocol::msg::AudioPlay::PID_SELF_CHECK_SUCCESS;
      text_msg.text = "自检完成,状态就绪!";
      audio_play_pub_->publish(text_msg);
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

  void SpeechNotify(int32_t code)
  {
    auto mcpim_iter = module_code_play_id_map.find(code);
    if (mcpim_iter != module_code_play_id_map.end()) {
      uint16_t play_id = mcpim_iter->second;
      bool is_self_check = ((code % 10) == 9) ? true : false;
      if (is_self_check) {
        std::chrono::seconds timeout(2);
        auto req = std::make_shared<protocol::srv::SdcardPlayIdQuery::Request>();
        req->play_id = play_id;
        auto future_result = sdcard_playid_query_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
        } else {
          play_id = protocol::msg::AudioPlay::PID_SELF_CHECK_FAILED;
        }
      }
      std::chrono::seconds timeout(6);
      auto req = std::make_shared<protocol::srv::AudioTextPlay::Request>();
      req->module_name = audio_info_node_->get_name();
      req->is_online = false;
      req->speech.play_id = play_id;
      auto future_result = audio_text_play_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        ERROR("call play sound service success. play id:%d", play_id);
      } else {
        ERROR("call play sound service failed! play id:%d", play_id);
      }
    }
  }

private:
  rclcpp::Node::SharedPtr audio_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr audio_callback_group_;
  rclcpp::Publisher<AudioMsg>::SharedPtr audio_play_pub_ {nullptr};
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_text_play_client_ {nullptr};
  rclcpp::Client<protocol::srv::SdcardPlayIdQuery>::SharedPtr sdcard_playid_query_client_ {nullptr};
  bool init_ {false};
  const std::map<int32_t, uint16_t> module_code_play_id_map = {
    {2409, 31006},    // GPS自检失败
    {2109, 31007},    // 雷达自检失败
    {2209, 31008},    // TOF自检失败
    {2309, 31009},    // 超声自检失败
    {1409, 31010},    // Bms自检失败
    {1109, 31011},    // Led自检失败
    {1209, 31011},    // MiniLed自检失败
    {1509, 31012},    // Touch自检失败
    {1609, 31013},    // Uwb自检失败
    {5109, 31014},    // Audio自检失败
    {3009, 31015},    // Motion自检失败
    {2403, 31016},    // GPS切换工作模式失败
    {2103, 31017},    // 雷达切换工作模式失败
    {2203, 31018},    // TOF切换工作模式失败
    {2303, 31019},    // 超声切换工作模式失败
    {1403, 31020},    // Bms切换工作模式失败
    {1103, 31021},    // Led切换工作模式失败
    {1203, 31021},    // MiniLed切换工作模式失败
    {1503, 31022},    // Touch切换工作模式失败
    {1603, 31023},    // Uwb切换工作模式失败
    {5103, 31024},    // Audio切换工作模式失败
    {3003, 31025},    // Motion切换工作模式失败
    {2003, 31024},    // Sensor切换工作模式失败
    {1003, 31025},    // Device切换工作模式失败
    {2401, 31016},    // GPS切换工作模式失败
    {2101, 31017},    // 雷达切换工作模式失败
    {2201, 31018},    // TOF切换工作模式失败
    {2301, 31019},    // 超声切换工作模式失败
    {1401, 31020},    // Bms切换工作模式失败
    {1101, 31021},    // Led切换工作模式失败
    {1201, 31021},    // MiniLed切换工作模式失败
    {1501, 31022},    // Touch切换工作模式失败
    {1601, 31023},    // Uwb切换工作模式失败
    {5101, 31024},    // Audio切换工作模式失败
    {3001, 31025},    // Motion切换工作模式失败
    {2001, 31024},    // Sensor切换工作模式失败
    {1001, 31025},    // Device切换工作模式失败
  };
};

}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__AUDIO_INFO_HPP_
