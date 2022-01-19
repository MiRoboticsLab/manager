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
#ifndef CYBERDOG_MANAGER__CYBERDOG_MANAGER_HPP_
#define CYBERDOG_MANAGER__CYBERDOG_MANAGER_HPP_
#include <vector>
#include "manager_base/manager_base.hpp"

namespace cyberdog
{
namespace manager
{
struct HeartbeatsRecorder
{
  // std::string name;
  int64_t timestamp;
  int counter = 0;
};  // struct HeartbeatsRecorder

class CyberdogManager : public ManagerBase
{
  using ManagerHeartbeatsMsg = protocol::msg::Heartbeats;
public:
  CyberdogManager(const std::string& name);
  ~CyberdogManager();

  void Config() override;
  bool Init() override;
  void Run() override;
  bool SelfCheck() override;
  void HeartbeatsCheck();
  void HeartbeatsCallback(const ManagerHeartbeatsMsg::SharedPtr msg);

public:
  void OnError() override;
  void OnLowPower() override;
  void OnSuspend() override;
  void OnProtected() override;
  void OnActive() override;

private:
  std::string name_;
  std::vector<std::string> manager_vec_;
  // std::vector<std::string> heartbeats_vec_;
  std::map<std::string, HeartbeatsRecorder> heartbeats_map_;
  rclcpp::Node::SharedPtr node_ptr {nullptr};
  rclcpp::Subscription<ManagerHeartbeatsMsg>::SharedPtr heartbeats_sub_{nullptr};
  rclcpp::TimerBase::SharedPtr heartbeats_timer_;
// TODO: black box handler
};  // class CyberdogManager
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__CYBERDOG_MANAGER_HPP_