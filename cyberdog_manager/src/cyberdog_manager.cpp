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
: name_(name)
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  env_node_ptr_ = std::make_unique<EnvContex>(node_ptr_);
  query_node_ptr_ = std::make_unique<QueryInfoNode>(node_ptr_);
  account_node_ptr_ = std::make_unique<AccountInfoNode>(node_ptr_);
  mssc_context_ptr_ = std::make_shared<cyberdog::manager::MachineStateSwitchContext>(node_ptr_);
  power_consumption_node_ptr = std::make_shared<PowerConsumptionInfoNode>(
    node_ptr_,
    std::bind(&MachineStateSwitchContext::KeepDownOverTime, mssc_context_ptr_));
  // ready_node_ptr = std::make_unique<ReadyNotifyNode>(name_ + "_ready");
  ready_node_ptr = std::make_unique<ReadyNotifyNode>(node_ptr_);
  heart_beat_ptr_ = std::make_unique<cyberdog::manager::HeartContext>(
    node_ptr_,
    std::bind(&CyberdogManager::SetState, this, std::placeholders::_1, std::placeholders::_2));
  error_context_ptr_ = std::make_unique<cyberdog::manager::ErrorContext>(name_ + "_error");
  touch_node_ptr = std::make_unique<TouchInfoNode>(node_ptr_);
  audio_node_ptr = std::make_shared<AudioInfoNode>(
    node_ptr_);
  led_node_ptr = std::make_shared<LedInfoNode>(node_ptr_);
  bcin_node_ptr = std::make_unique<BatteryCapacityInfoNode>(
    node_ptr_,
    std::bind(
      &MachineStateSwitchContext::BatteryChargeUpdate, mssc_context_ptr_,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&LedInfoNode::BmsStatus, led_node_ptr, std::placeholders::_1));
  executor_.add_node(node_ptr_);
}

cyberdog::manager::CyberdogManager::~CyberdogManager()
{
  node_ptr_ = nullptr;
}

void cyberdog::manager::CyberdogManager::Config()
{
  INFO("config state handler");
}

bool cyberdog::manager::CyberdogManager::Init()
{
  bool exit_lowpower = false;
  int exit_times = 0;
  while (!exit_lowpower && exit_times < 3) {
    if (power_consumption_node_ptr->QueryLowPower()) {
      exit_lowpower = power_consumption_node_ptr->EnterLowPower(false);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    } else {
      exit_lowpower = true;
    }
    INFO("exit lowpower:%s", (exit_lowpower ? "true" : "false"));
    ++exit_times;
  }
  query_node_ptr_->Init();
  bcin_node_ptr->Init();
  // 开始自检
  ready_node_ptr->SelfCheck(1, selfcheck_status_);
  error_context_ptr_->Init();
  // 硬件setup
  mssc_context_ptr_->ExecuteSetUp(false);
  mssc_context_ptr_->SetLowpowerEnterAndExitCallback(
    std::bind(
      &PowerConsumptionInfoNode::EnterLowPower, power_consumption_node_ptr,
      std::placeholders::_1));
  mssc_context_ptr_->SetExceptionPlaySoundCallback(
    std::bind(
      &AudioInfoNode::SpeechNotify, audio_node_ptr,
      std::placeholders::_1));
  mssc_context_ptr_->SetShutdownRebootCallback(
    std::bind(
      &PowerConsumptionInfoNode::ShutdownOrReboot, power_consumption_node_ptr,
      std::placeholders::_1));
  mssc_context_ptr_->SetControlTailLedCallback(
    std::bind(
      &PowerConsumptionInfoNode::TailLedControl, power_consumption_node_ptr,
      std::placeholders::_1, std::placeholders::_2));
  error_context_ptr_->ClearError();
  Config();
  if (!mssc_context_ptr_->ExecuteSelfCheck(selfcheck_status_)) {
    ERROR(">>>XXXXX--- hardware machine state self check error!");
    ready_node_ptr->SelfCheck(2, selfcheck_status_);
  } else {
    audio_node_ptr->Init();
    ready_node_ptr->SelfCheck(3, selfcheck_status_);
    OnActive();
    // 软件setup
    mssc_context_ptr_->ExecuteSetUp(true);
    OnActive();
    ready_node_ptr->SelfCheck(0, selfcheck_status_);
  }
  power_consumption_node_ptr->Init();
  mssc_context_ptr_->Init();
  heart_beat_ptr_->Init();
  audio_node_ptr->Init();
  audio_node_ptr->SpeechNotify(5300);
  return true;
}

bool cyberdog::manager::CyberdogManager::SetState(int8_t state, std::string json_data)
{
  (void) state;
  (void) json_data;
  return true;
}

void cyberdog::manager::CyberdogManager::Run()
{
  executor_.spin();
  rclcpp::shutdown();
}

void cyberdog::manager::CyberdogManager::OnActive()
{
  INFO("trigger state:on active");
  bool result = mssc_context_ptr_->ExecuteActive();
  query_node_ptr_->Report(true);
  mssc_context_ptr_->KeepMsState();
  if (result) {
    INFO("!!! All node in detectedc machine state is acitve ok !!!");
    ready_node_ptr->Ready(true);
    ready_node_ptr->MachineState(0);
  } else {
    // audio_node_ptr->Error("自检失败!自检失败!自检失败!");
    ready_node_ptr->Ready(false);
    ready_node_ptr->MachineState(-2);
  }
  // bcin_node_ptr->SetBms(BatteryMachineState::BMS_NORMAL);
}
