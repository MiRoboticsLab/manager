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
#include "protocol/srv/audio_volume_get.hpp"
#include "protocol/srv/audio_execute.hpp"
#include "protocol/srv/motor_temp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cyberdog_manager/env_contex.hpp"
#include "cyberdog_manager/query_info.hpp"
#include "cyberdog_manager/account_info.hpp"
#include "cyberdog_manager/power_consumption_info.hpp"
#include "cyberdog_manager/ready_info.hpp"
#include "cyberdog_manager/battery_capacity_info.hpp"
#include "cyberdog_manager/touch_info.hpp"
#include "cyberdog_manager/audio_info.hpp"
#include "cyberdog_manager/heart_context.hpp"
#include "cyberdog_manager/error_context.hpp"
#include "cyberdog_manager/machine_state_switch_context.hpp"
#include "cyberdog_manager/led_info.hpp"
#include "cyberdog_manager/power_brd_info.hpp"

namespace cyberdog
{
namespace manager
{

class CyberdogManager /* : public ManagerBase */
{
public:
  explicit CyberdogManager(const std::string & name);
  ~CyberdogManager();

  void Config();
  bool Init();
  void Run();

private:
  void OnActive();
  bool SetState(int8_t state, std::string json_data = "{}");

private:
  std::string name_;
  std::map<std::string, int32_t> selfcheck_status_;
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::unique_ptr<EnvContex> env_node_ptr_ {nullptr};
  std::unique_ptr<QueryInfoNode> query_node_ptr_ {nullptr};
  std::unique_ptr<AccountInfoNode> account_node_ptr_ {nullptr};
  std::shared_ptr<PowerConsumptionInfoNode> power_consumption_node_ptr {nullptr};
  std::unique_ptr<ReadyNotifyNode> ready_node_ptr {nullptr};
  std::unique_ptr<BatteryCapacityInfoNode> bcin_node_ptr {nullptr};
  std::unique_ptr<TouchInfoNode> touch_node_ptr {nullptr};
  std::shared_ptr<AudioInfoNode> audio_node_ptr {nullptr};
  std::shared_ptr<LedInfoNode> led_node_ptr {nullptr};
  std::unique_ptr<cyberdog::manager::HeartContext> heart_beat_ptr_ {nullptr};
  std::unique_ptr<cyberdog::manager::ErrorContext> error_context_ptr_ {nullptr};
  std::shared_ptr<cyberdog::manager::MachineStateSwitchContext> mssc_context_ptr_ {nullptr};
  std::shared_ptr<BlackBox> black_box_ptr_ {nullptr};
  std::shared_ptr<cyberdog::manager::PowerboardIndoNode> power_brd_node_ptr_ {nullptr};
};  // class CyberdogManager
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__CYBERDOG_MANAGER_HPP_
