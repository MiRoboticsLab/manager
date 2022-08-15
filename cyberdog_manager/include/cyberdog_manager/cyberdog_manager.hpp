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
#include <map>
#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <string>
#include "manager_base/manager_base.hpp"
#include "black_box/black_box.hpp"
#include "protocol/srv/device_info.hpp"
#include "protocol/srv/audio_volume_get.hpp"
#include "protocol/srv/audio_execute.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "protocol/msg/connector_status.hpp"
#include "protocol/msg/bms_status.hpp"
#include "cyberdog_machine/cyberdog_heartbeats.hpp"

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

struct WifiInfo
{
  std::string ssid;
  std::string ip;
  std::string mac;
  uint8_t strength;
};  // struct WifiInfo

class CyberdogManager : public ManagerBase
{
  using ManagerHeartbeatsMsg = protocol::msg::Heartbeats;

public:
  explicit CyberdogManager(const std::string & name);
  ~CyberdogManager();

  void Config() override;
  bool Init() override;
  void Run() override;
  bool SelfCheck() override;
  // void HeartbeatsCheck();
  void HeartbeatsCallback(const ManagerHeartbeatsMsg::SharedPtr msg);

public:
  void OnError() override;
  void OnLowPower() override;
  void OnSuspend() override;
  void OnProtected() override;
  void OnActive() override;

private:
  void NodeStateConfirm(const std::string & name, bool lost);
  void HeartbeatsStateNotify();
  void QueryDeviceInfo(
    const protocol::srv::DeviceInfo::Request::SharedPtr request,
    protocol::srv::DeviceInfo::Response::SharedPtr);
  void UidCallback(const std_msgs::msg::String::SharedPtr msg);
  void DogInfoUpdate(const std_msgs::msg::Bool::SharedPtr msg);
  void ConnectStatus(const protocol::msg::ConnectorStatus::SharedPtr msg);
  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dog_info_update_sub_;
  std::string name_;
  bool has_error_;
  std::string sn_;
  std::string uid_;
  bool name_switch_;
  std::string default_name_;
  std::string nick_name_;
  std::vector<std::string> manager_vec_;
  // std::vector<std::string> heartbeats_vec_;
  std::map<std::string, HeartbeatsRecorder> heartbeats_map_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Node::SharedPtr query_node_ptr_ {nullptr};
  rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::Subscription<ManagerHeartbeatsMsg>::SharedPtr heartbeats_sub_{nullptr};
  rclcpp::Subscription<protocol::msg::ConnectorStatus>::SharedPtr connect_status_sub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  // rclcpp::TimerBase::SharedPtr heartbeats_timer_;
  rclcpp::Service<protocol::srv::DeviceInfo>::SharedPtr device_info_get_srv_;
  rclcpp::Client<protocol::srv::AudioVolumeGet>::SharedPtr audio_volume_get_client_;
  rclcpp::Client<protocol::srv::AudioExecute>::SharedPtr audio_execute_client_;

  std::shared_ptr<BlackBox> black_box_ptr_ {nullptr};
  std::unique_ptr<cyberdog::machine::HeartBeats> heart_beats_ptr_ {nullptr};
  std::unique_ptr<cyberdog::manager::WifiInfo> wifi_info_ptr_ {nullptr};
  protocol::msg::BmsStatus bms_status_;
};  // class CyberdogManager
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__CYBERDOG_MANAGER_HPP_
