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
#include <vector>
#include <map>
#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/ota_server_cmd.hpp"
#include "cyberdog_manager/cyberdog_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "low_power_consumption/pm_if.h"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

cyberdog::manager::CyberdogManager::CyberdogManager(const std::string & name)
: ManagerBase(name),
  name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  machine_state_ptr_ = std::make_unique<cyberdog::manager::StateContext>(name_ + "_machine");
  query_node_ptr_ = std::make_unique<QueryInfoNode>(node_ptr_);
  account_node_ptr_ = std::make_unique<AccountInfoNode>(node_ptr_);
  mssc_context_ptr_ = std::make_shared<cyberdog::manager::MachineStateSwitchContext>(node_ptr_);
  power_consumption_node_ptr = std::make_shared<PowerConsumptionInfoNode>(
    node_ptr_,
    std::bind(&MachineStateSwitchContext::MachineStateHandler, mssc_context_ptr_, std::placeholders::_1));
  ready_node_ptr = std::make_unique<ReadyNotifyNode>(name_ + "_ready");
  heart_beat_ptr_ = std::make_unique<cyberdog::manager::HeartContext>(
    node_ptr_,
    std::bind(&CyberdogManager::SetState, this, std::placeholders::_1, std::placeholders::_2));
  error_context_ptr_ = std::make_unique<cyberdog::manager::ErrorContext>(name_ + "_error");
  bcin_node_ptr = std::make_unique<BatteryCapacityInfoNode>(
    node_ptr_,
    std::bind(&PowerConsumptionInfoNode::UpdataBatterySoc, power_consumption_node_ptr, std::placeholders::_1));
  touch_node_ptr = std::make_unique<TouchInfoNode>(node_ptr_);
  audio_node_ptr = std::make_unique<AudioInfoNode>(node_ptr_);
  // led_node_ptr = std::make_unique<LedInfoNode>(node_ptr_);
  // executor_.add_node(node_ptr_);
  // black_box_ptr_ = std::make_shared<BlackBox>(node_ptr_);
}

cyberdog::manager::CyberdogManager::~CyberdogManager()
{
  node_ptr_ = nullptr;
}

void cyberdog::manager::CyberdogManager::Config()
{
  INFO("config state handler");
  // power_consumption_node_ptr->SetActive(std::bind(&CyberdogManager::OnActive, this));
  // power_consumption_node_ptr->SetDeactive(std::bind(&CyberdogManager::OnDeactive, this));
  // bcin_node_ptr->SetProtect(std::bind(&CyberdogManager::OnProtected, this));
  // bcin_node_ptr->SetLowpower(std::bind(&CyberdogManager::OnLowPower, this));
  // bcin_node_ptr->SetActive(std::bind(&CyberdogManager::OnActive, this));
  // bcin_node_ptr->SetShutdown(std::bind(&CyberdogManager::OnTearDown, this));
  mssc_context_ptr_->SetActive(std::bind(&CyberdogManager::OnActive, this));
  mssc_context_ptr_->SetProtect(std::bind(&CyberdogManager::OnProtected, this));
  mssc_context_ptr_->SetLowpower(std::bind(&CyberdogManager::OnLowPower, this));
  mssc_context_ptr_->SetOta(std::bind(&CyberdogManager::OnOta, this));
  mssc_context_ptr_->SetShutdown(std::bind(&CyberdogManager::OnTearDown, this));
}

bool cyberdog::manager::CyberdogManager::Init()
{
  ready_node_ptr->SelfCheck(1);
  error_context_ptr_->Init();
  if (!machine_state_ptr_->Init()) {
    ERROR(">>>XXXXX---machine state init error!");
  }
  mssc_context_ptr_->SetStateHandler(machine_state_ptr_->GetAchieveStates());
  error_context_ptr_->ClearError();
  Config();
  if (!RegisterStateHandler(node_ptr_)) {
    return false;
  }
  if (!SelfCheck() ) {
    // if (false) {
    ERROR(">>>XXXXX---machine state self check error!");
    audio_node_ptr->Error("自检失败!自检失败!自检失败!");
    ready_node_ptr->SelfCheck(2);
    return false;
  } else {
    // heart_beat_ptr_->Init();
  }

  // heart_beat_ptr_->Init();

  // if (!black_box_ptr_->Init()) {
  if (true) {
    // error msg
    // send msg to app ?
  }

  query_node_ptr_->Init();
  // bcin_node_ptr->Init();
  mssc_context_ptr_->Init();

  OnActive();

  heart_beat_ptr_->Init();

  return true;
}

bool cyberdog::manager::CyberdogManager::SelfCheck()
{
  return machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_SelfCheck);
}

// void cyberdog::manager::CyberdogManager::HeartbeatsCheck()
// {
//   auto current_time = GetMsTime();
//   std::for_each(
//     heartbeats_map_.begin(), heartbeats_map_.end(),
//     [this, &current_time](
//       std::map<std::string, HeartbeatsRecorder>::reference recorder) {
//       if (current_time - recorder.second.timestamp > 500) {
//         if (++recorder.second.counter > 5) {
//           // error msg
//           this->SetState((int8_t)system::ManagerState::kError);
//         } else {
//           // error msg
//         }
//       } else {
//         recorder.second.counter = 0;
//       }
//     }
//   );
// }

void cyberdog::manager::CyberdogManager::Run()
{
  executor_.spin();
  rclcpp::shutdown();
}

void cyberdog::manager::CyberdogManager::OnError()
{
  ERROR("on error");
}

void cyberdog::manager::CyberdogManager::OnLowPower()
{
  INFO("on lowpower");
  // bcin_node_ptr->SetBms(BatteryMachineState::BMS_LOWPOWER);
  machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_LowPower);
}

void cyberdog::manager::CyberdogManager::OnSuspend()
{
  ERROR("on suspend");
}

void cyberdog::manager::CyberdogManager::OnProtected()
{
  ERROR("on protect");
  // bcin_node_ptr->SetBms(BatteryMachineState::BMS_PROTECT);
  machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_Protected);
}

void cyberdog::manager::CyberdogManager::OnActive()
{
  INFO("trigger state:on active");
  bool result = machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_Active);
  query_node_ptr_->Report(true);
  if (result) {
    INFO("!!! All node in detectedc machine state is acitve ok !!!");
    audio_node_ptr->Init();
    ready_node_ptr->Ready(true);
    ready_node_ptr->MachineState(0);
    ready_node_ptr->SelfCheck(0);
  } else {
    audio_node_ptr->Error("自检失败!自检失败!自检失败!");
    ready_node_ptr->Ready(false);
    ready_node_ptr->MachineState(-2);
    ready_node_ptr->SelfCheck(2);
  }
  // bcin_node_ptr->SetBms(BatteryMachineState::BMS_NORMAL);
}

void cyberdog::manager::CyberdogManager::OnDeactive()
{
  INFO("trigger state:on deactive");
  machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_DeActive);
  query_node_ptr_->Report(false);
  ready_node_ptr->Ready(false);
}

void cyberdog::manager::CyberdogManager::OnTearDown()
{
  INFO("trigger state:on teardown");
  machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_TearDown);
}

void cyberdog::manager::CyberdogManager::OnOta()
{
  INFO("trigger state:on ota");
  // machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_OTA);
}
