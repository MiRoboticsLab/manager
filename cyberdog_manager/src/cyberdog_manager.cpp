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
#include <algorithm>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include "cyberdog_manager/cyberdog_manager.hpp"

cyberdog::manager::CyberdogManager::CyberdogManager(const std::string& name)
  : ManagerBase(name),
  name_(name)
  
{
  node_ptr = rclcpp::Node::make_shared(name_);
  manager_vec_.emplace_back("device");
  manager_vec_.emplace_back("sensor");
  manager_vec_.emplace_back("motion");
  manager_vec_.emplace_back("perception");
}

cyberdog::manager::CyberdogManager::~CyberdogManager()
{
  node_ptr = nullptr;
}

void cyberdog::manager::CyberdogManager::Config()
{
  std::cout << "config\n";
}

bool cyberdog::manager::CyberdogManager::Init()
{
  std::cout << "init\n";
  if( !RegisterStateHandler(node_ptr))
  {
    return false;
  }
  if( !SelfCheck() ) {
    return false;
  } else {
    std::for_each(manager_vec_.cbegin(), manager_vec_.cend(),
    [this](const std::string& name) {
      HeartbeatsRecorder heartbeats_recorder;
      heartbeats_recorder.timestamp = GetMsTime();
      heartbeats_recorder.counter = 0;
      this->heartbeats_map_.insert(
        std::make_pair(name, heartbeats_recorder)
      );
    }
    );
    heartbeats_sub_ = node_ptr->create_subscription<ManagerHeartbeatsMsg>(
      "manager_heartbeats",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogManager::HeartbeatsCallback, this, std::placeholders::_1));
  }

  // start heartbeats check work
  heartbeats_timer_ = node_ptr->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&CyberdogManager::HeartbeatsCheck, this)
    );
  SetState((int8_t)system::ManagerState::kActive);
  return true;
}

bool cyberdog::manager::CyberdogManager::SelfCheck()
{
  // std::vector<std::string> manager_vec;

  std::vector<rclcpp::Client<protocol::srv::ManagerInit>::SharedPtr> manager_client;
  auto wait_result = std::all_of(manager_vec_.cbegin(), manager_vec_.cend(),
    [this, &manager_client](const std::string& manager_name) {
      std::string server_name = std::string("manager_init_") + manager_name;
      auto client = this->node_ptr->create_client<protocol::srv::ManagerInit>(server_name);
      if( !client->wait_for_service(std::chrono::nanoseconds(1000 * 1000 * 1000)) ) {
        // error msg or other executing
        return false;
      } else {
        manager_client.emplace_back(client);
      }
      return true;
    }
  );

  if( !wait_result ) {
    // error msg
    manager_client.clear();
    return false;
  }

  auto check_result = std::all_of(manager_client.cbegin(), manager_client.cend(), 
    [this](const rclcpp::Client<protocol::srv::ManagerInit>::SharedPtr client_ptr) {
      auto request_ptr = std::make_shared<protocol::srv::ManagerInit::Request>();
      auto result = client_ptr->async_send_request(request_ptr);
      if( rclcpp::spin_until_future_complete(this->node_ptr, result) == rclcpp::FutureReturnCode::SUCCESS ) {
        if( result.get()->res_code != (int32_t)system::KeyCode::kOK ) {
          // error msg or other executing
          return false;
        }
      } else {
        // error msg or other executing
        return false;
      }
      return true;
    }
  );
  
  if( !check_result ) {
    // error msg
    return false;
  }

  return true;
}

void cyberdog::manager::CyberdogManager::HeartbeatsCallback(
  const ManagerHeartbeatsMsg::SharedPtr msg)
{
  auto iter = heartbeats_map_.find(msg->name);
  if(iter == heartbeats_map_.end())
    return;
  iter->second.timestamp = msg->timestamp;
  // iter->second.counter = 0;
}

void cyberdog::manager::CyberdogManager::HeartbeatsCheck()
{
  auto current_time = GetMsTime();
  std::for_each(heartbeats_map_.begin(), heartbeats_map_.end(),
    [this, &current_time](
      std::map<std::string, HeartbeatsRecorder>::reference recorder) {
        if(current_time - recorder.second.timestamp > 500) {
          if(++recorder.second.counter > 5) {
            // error msg
            this->SetState((int8_t)system::ManagerState::kError);
          } else {
            // error msg
          }
        } else {
          recorder.second.counter = 0;
        }
    }
  );
}

void cyberdog::manager::CyberdogManager::Run()
{
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
}

void cyberdog::manager::CyberdogManager::OnError()
{
  std::cout << "on error\n";
}

void cyberdog::manager::CyberdogManager::OnLowPower()
{
  std::cout << "on lowpower\n";
}

void cyberdog::manager::CyberdogManager::OnSuspend()
{
  std::cout << "on suspend\n";
}

void cyberdog::manager::CyberdogManager::OnProtected()
{
  std::cout << "on protect\n";
}

void cyberdog::manager::CyberdogManager::OnActive()
{
  std::cout << "on active\n";
}
