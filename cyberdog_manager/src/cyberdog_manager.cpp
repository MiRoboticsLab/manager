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
  power_consumption_node_ptr = std::make_unique<PowerConsumptionInfoNode>(node_ptr_);
  heart_beat_ptr_ = std::make_unique<cyberdog::manager::HeartContext>(
    node_ptr_,
    std::bind(&CyberdogManager::SetState, this, std::placeholders::_1, std::placeholders::_2));
  executor_.add_node(node_ptr_);
  // black_box_ptr_ = std::make_shared<BlackBox>(node_ptr_);
}

cyberdog::manager::CyberdogManager::~CyberdogManager()
{
  node_ptr_ = nullptr;
}

void cyberdog::manager::CyberdogManager::Config()
{
  INFO("config");
}

bool cyberdog::manager::CyberdogManager::Init()
{
  if (!machine_state_ptr_->Init()) {
    ERROR("machine state init error!");
  }
  if (!RegisterStateHandler(node_ptr_)) {
    return false;
  }
  if (!SelfCheck() ) {
    return false;
  } else {
    heart_beat_ptr_->Init();
  }

  // if (!black_box_ptr_->Init()) {
  if (true) {
    // error msg
    // send msg to app ?
  }

  query_node_ptr_->Init();

  return true;
}

bool cyberdog::manager::CyberdogManager::SelfCheck()
{
  return machine_state_ptr_->SetState(cyberdog::machine::MachineState::MS_SelfCheck);
  // std::vector<std::string> manager_vec_;

  // std::vector<rclcpp::Client<protocol::srv::ManagerInit>::SharedPtr> manager_client;
  // auto wait_result = std::all_of(
  //   manager_vec_.cbegin(), manager_vec_.cend(),
  //   [this, &manager_client](const std::string & manager_name) {
  //     std::string server_name = std::string("manager_init_") + manager_name;
  //     auto client = this->node_ptr_->create_client<protocol::srv::ManagerInit>(server_name);
  //     if (!client->wait_for_service(std::chrono::nanoseconds(1000 * 1000 * 1000)) ) {
  //       // error msg or other executing
  //       return false;
  //     } else {
  //       manager_client.emplace_back(client);
  //     }
  //     return true;
  //   }
  // );

  // if (!wait_result) {
  //   // error msg
  //   manager_client.clear();
  //   return false;
  // }

  // auto check_result = std::all_of(
  //   manager_client.cbegin(), manager_client.cend(),
  //   [this](const rclcpp::Client<protocol::srv::ManagerInit>::SharedPtr client_ptr) {
  //     auto request_ptr = std::make_shared<protocol::srv::ManagerInit::Request>();
  //     auto result = client_ptr->async_send_request(request_ptr);
  //     if (rclcpp::spin_until_future_complete(
  //       this->node_ptr_,
  //       result) == rclcpp::FutureReturnCode::SUCCESS)
  //     {
  //       if (result.get()->res_code != (int32_t)system::KeyCode::kOK) {
  //         // error msg or other executing
  //         return false;
  //       }
  //     } else {
  //       // error msg or other executing
  //       return false;
  //     }
  //     return true;
  //   }
  // );

  // if (!check_result) {
  //   // error msg
  //   return false;
  // }

  // return true;
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
  ERROR("on lowpower");
}

void cyberdog::manager::CyberdogManager::OnSuspend()
{
  ERROR("on suspend");
}

void cyberdog::manager::CyberdogManager::OnProtected()
{
  ERROR("on protect");
}

void cyberdog::manager::CyberdogManager::OnActive()
{
  ERROR("on active");
}
