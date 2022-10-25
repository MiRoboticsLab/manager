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
#ifndef CYBERDOG_MANAGER__HEART_CONTEXT_HPP_
#define CYBERDOG_MANAGER__HEART_CONTEXT_HPP_

#include <string>
#include <memory>
#include <utility>
#include <vector>
#include <map>
#include "protocol/msg/heartbeats.hpp"
#include "manager_base/manager_base.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{
struct HeartbeatsRecorder
{
  // std::string name;
  int64_t timestamp;
  // int counter = 0;
  bool lost = false;
};  // struct HeartbeatsRecorder

class HeartContext final
{
  using ManagerHeartbeatsMsg = protocol::msg::Heartbeats;

public:
  explicit HeartContext(
    rclcpp::Node::SharedPtr node_ptr,
    std::function<bool(int8_t, std::string)> func)
  : heart_beats_node_(node_ptr),
    set_state_func_(func)
  {
    heart_beats_callback_group_ =
      heart_beats_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // manager_vec_.emplace_back("device");
    // manager_vec_.emplace_back("sensor");
    // manager_vec_.emplace_back("motion");
    // manager_vec_.emplace_back("perception");
    // manager_vec_.emplace_back("audio");
    HeartbeatsNodeInit();
    heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeats>(500, 5);
  }
  ~HeartContext() = default;
  void Init()
  {
    std::for_each(
      manager_vec_.cbegin(), manager_vec_.cend(),
      [this](const std::string & name) {
        HeartbeatsRecorder heartbeats_recorder;
        heartbeats_recorder.timestamp = GetMsTime();
        // heartbeats_recorder.counter = 0;
        heartbeats_recorder.lost = false;
        this->heartbeats_map_.insert(
          std::make_pair(name, heartbeats_recorder)
        );
      }
    );
    heart_beats_ptr_->HeartConfig(
      manager_vec_,
      std::bind(&cyberdog::manager::HeartContext::HeartbeatsStateNotify, this), 6);
    heart_beats_ptr_->RegisterLostCallback(
      std::bind(
        &cyberdog::manager::HeartContext::NodeStateConfirm, this,
        std::placeholders::_1, std::placeholders::_2)
    );
    rclcpp::SubscriptionOptions options;
    options.callback_group = heart_beats_callback_group_;
    heartbeats_sub_ = heart_beats_node_->create_subscription<ManagerHeartbeatsMsg>(
      "manager_heartbeats",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&HeartContext::HeartbeatsCallback, this, std::placeholders::_1),
      options);
    // start heartbeats check work
    heart_beats_ptr_->HeartRun();
    set_state_func_((int8_t)system::ManagerState::kActive, "{}");
  }

private:
  void HeartbeatsStateNotify()
  {
    Document json_document(kObjectType);
    for (auto & elem : heartbeats_map_) {
      Document node_document(kObjectType);
      CyberdogJson::Add(node_document, "alive", !elem.second.lost);
      CyberdogJson::Add(json_document, elem.first, node_document);
    }
    std::string json_string("{}");
    if (!CyberdogJson::Document2String(json_document, json_string)) {
      ERROR("error while encoding to json of heart beats state notify");
    }
    has_error_ == true ?
    set_state_func_((int8_t)system::ManagerState::kError, std::move(json_string)) :
    set_state_func_((int8_t)system::ManagerState::kOK, std::move(json_string));
    has_error_ = false;
  }

  void NodeStateConfirm(const std::string & name, bool lost)
  {
    auto iter = heartbeats_map_.find(name);
    if (iter == heartbeats_map_.end()) {
      return;
    }
    has_error_ = has_error_ | lost;
    if (has_error_) {
      ERROR("%s node lost", iter->first.c_str());
    }
    iter->second.lost = lost;
    iter->second.timestamp = GetMsTime();
  }

  void HeartbeatsCallback(const ManagerHeartbeatsMsg::SharedPtr msg)
  {
    auto iter = heartbeats_map_.find(msg->name);
    if (iter == heartbeats_map_.end()) {
      return;
    }
    heart_beats_ptr_->HeartUpdate(iter->first);
    // iter->second.timestamp = msg->timestamp;
    // // iter->second.counter = 0;
  }

  bool HeartbeatsNodeInit()
  {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/heart_beat_config.toml");
    toml::value config;
    if (!common::CyberdogToml::ParseFile(path, config)) {
      ERROR("Parse Heart Beat config file failed, toml file is invalid!");
      return false;
    }
    toml::value heartbeat;
    if (!common::CyberdogToml::Get(config, "heartbeat", heartbeat)) {
      ERROR("Heart Beat init failed, parse heartbeat config failed!");
      return false;
    }
    // toml::array actuator_array;
    if (!common::CyberdogToml::Get(heartbeat, "node", manager_vec_)) {
      ERROR("Heart Beat init failed, parse node array failed!");
      return false;
    }
    return true;
  }

private:
  bool has_error_ {false};
  rclcpp::Node::SharedPtr heart_beats_node_ {nullptr};
  std::function<bool(int8_t, std::string)> set_state_func_;
  rclcpp::CallbackGroup::SharedPtr heart_beats_callback_group_;
  std::vector<std::string> manager_vec_;
  std::map<std::string, HeartbeatsRecorder> heartbeats_map_;
  std::unique_ptr<cyberdog::machine::HeartBeats> heart_beats_ptr_ {nullptr};
  rclcpp::Subscription<ManagerHeartbeatsMsg>::SharedPtr heartbeats_sub_{nullptr};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__HEART_CONTEXT_HPP_
