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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/ota_server_cmd.hpp"
#include "cyberdog_manager/cyberdog_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

cyberdog::manager::CyberdogManager::CyberdogManager(const std::string & name)
: ManagerBase(name),
  name_(name), has_error_(false), sn_("")
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  query_node_ptr_ = rclcpp::Node::make_shared(name_ + "_query");
  executor_.add_node(node_ptr_);
  executor_.add_node(query_node_ptr_);
  device_info_get_srv_ =
    query_node_ptr_->create_service<protocol::srv::DeviceInfo>(
    "query_divice_info",
    std::bind(
      &CyberdogManager::QueryDeviceInfo, this, std::placeholders::_1,
      std::placeholders::_2));
  // manager_vec_.emplace_back("device");
  // manager_vec_.emplace_back("sensor");
  // manager_vec_.emplace_back("motion");
  // manager_vec_.emplace_back("perception");
  manager_vec_.emplace_back("audio");

  // black_box_ptr_ = std::make_shared<BlackBox>(node_ptr_);
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeats>(500, 5);
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
  if (!RegisterStateHandler(node_ptr_)) {
    return false;
  }
  // if (!SelfCheck() ) {
  if (false) {
    return false;
  } else {
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
      std::bind(&cyberdog::manager::CyberdogManager::HeartbeatsStateNotify, this), 6);
    heart_beats_ptr_->RegisterLostCallback(
      std::bind(
        &cyberdog::manager::CyberdogManager::NodeStateConfirm, this,
        std::placeholders::_1, std::placeholders::_2)
    );
    heartbeats_sub_ = node_ptr_->create_subscription<ManagerHeartbeatsMsg>(
      "manager_heartbeats",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogManager::HeartbeatsCallback, this, std::placeholders::_1));
  }

  // start heartbeats check work
  heart_beats_ptr_->HeartRun();
  // heartbeats_timer_ = node_ptr_->create_wall_timer(
  //   std::chrono::seconds(2),
  //   std::bind(&CyberdogManager::HeartbeatsCheck, this)
  // );
  SetState((int8_t)system::ManagerState::kActive);

  // if (!black_box_ptr_->Init()) {
  if (true) {
    // error msg
    // send msg to app ?
  }

  return true;
}

bool cyberdog::manager::CyberdogManager::SelfCheck()
{
  // std::vector<std::string> manager_vec;

  std::vector<rclcpp::Client<protocol::srv::ManagerInit>::SharedPtr> manager_client;
  auto wait_result = std::all_of(
    manager_vec_.cbegin(), manager_vec_.cend(),
    [this, &manager_client](const std::string & manager_name) {
      std::string server_name = std::string("manager_init_") + manager_name;
      auto client = this->node_ptr_->create_client<protocol::srv::ManagerInit>(server_name);
      if (!client->wait_for_service(std::chrono::nanoseconds(1000 * 1000 * 1000)) ) {
        // error msg or other executing
        return false;
      } else {
        manager_client.emplace_back(client);
      }
      return true;
    }
  );

  if (!wait_result) {
    // error msg
    manager_client.clear();
    return false;
  }

  auto check_result = std::all_of(
    manager_client.cbegin(), manager_client.cend(),
    [this](const rclcpp::Client<protocol::srv::ManagerInit>::SharedPtr client_ptr) {
      auto request_ptr = std::make_shared<protocol::srv::ManagerInit::Request>();
      auto result = client_ptr->async_send_request(request_ptr);
      if (rclcpp::spin_until_future_complete(
        this->node_ptr_,
        result) == rclcpp::FutureReturnCode::SUCCESS)
      {
        if (result.get()->res_code != (int32_t)system::KeyCode::kOK) {
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

  if (!check_result) {
    // error msg
    return false;
  }

  return true;
}

void cyberdog::manager::CyberdogManager::HeartbeatsCallback(
  const ManagerHeartbeatsMsg::SharedPtr msg)
{
  auto iter = heartbeats_map_.find(msg->name);
  if (iter == heartbeats_map_.end()) {
    return;
  }
  heart_beats_ptr_->HeartUpdate(iter->first);
  // iter->second.timestamp = msg->timestamp;
  // // iter->second.counter = 0;
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

void cyberdog::manager::CyberdogManager::NodeStateConfirm(
  const std::string & name, bool lost)
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

void cyberdog::manager::CyberdogManager::HeartbeatsStateNotify()
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
  this->SetState((int8_t)system::ManagerState::kError, std::move(json_string)) :
  this->SetState((int8_t)system::ManagerState::kOK, std::move(json_string));
  has_error_ = false;
}

void cyberdog::manager::CyberdogManager::QueryDeviceInfo(
  const protocol::srv::DeviceInfo::Request::SharedPtr request,
  protocol::srv::DeviceInfo::Response::SharedPtr response)
{
  bool is_sn = request->enables[0];
  bool is_version = request->enables[1];
  std::string info;
  info = "{";
  if (is_sn) {
    if (sn_ == "") {
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_sn_ger_srv_;
      audio_sn_ger_srv_ =
        node_ptr_->create_client<std_srvs::srv::Trigger>("get_dog_sn");
      if (!audio_sn_ger_srv_->wait_for_service(std::chrono::seconds(2))) {
        ERROR("call sn server not avalible");
      }
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future_result = audio_sn_ger_srv_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        sn_ = future_result.get()->message;
      } else {
        ERROR("call get sn service failed!");
        sn_ = "unkown";
      }
    }
    info += "\"sn\": ";
    info += "\"";
    info += sn_;
    info += "\"";
  }
  if (is_version) {
    rclcpp::Client<protocol::srv::OtaServerCmd>::SharedPtr ota_ver_get_srv_;
    ota_ver_get_srv_ =
      node_ptr_->create_client<protocol::srv::OtaServerCmd>("ota_versions");
    if (!ota_ver_get_srv_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call ota version not avalible");
    }
    std::chrono::seconds timeout(3);
    auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
    req->request.key = "ota_command_version_query";
    auto future_result = ota_ver_get_srv_->async_send_request(req);
    std::future_status status = future_result.wait_for(timeout);
    info += ";";
    if (status == std::future_status::ready) {
      std::string version = future_result.get()->response.type;
      info += "\"version\": ";
      info += version;
    } else {
      ERROR("call ota version failed!");
      info += "\"version\": \"unkown\"";
    }
  }
  info += "}";
  response->info = info;
}
