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
#include "protocol/msg/motion_id.hpp"
#include "cyberdog_manager/cyberdog_manager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "user_info_manager/UserAccountManager.hpp"
#include "low_power_consumption/pm_if.h"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

cyberdog::manager::CyberdogManager::CyberdogManager(const std::string & name)
: ManagerBase(name),
  name_(name), has_error_(false), uid_("")
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  query_node_srv_ptr_ = rclcpp::Node::make_shared(name_ + "_srv_query");
  query_node_ptr_ = std::make_unique<QueryInfo>(name_ + "_query");
  query_node_feedback_ptr_ = rclcpp::Node::make_shared(name_ + "_query_feedback");
  query_account_add_ptr_ = rclcpp::Node::make_shared(name_ + "_query_account_add");
  low_power_consumption_ptr_ = rclcpp::Node::make_shared(name_ + "_low_power_consumption");
  back_end_mqtt_ptr_ = rclcpp::Node::make_shared(name_ + "_back_end_mqtt");
  executor_.add_node(node_ptr_);
  executor_.add_node(query_node_srv_ptr_);
  executor_.add_node(query_node_feedback_ptr_);
  executor_.add_node(query_account_add_ptr_);
  executor_.add_node(low_power_consumption_ptr_);
  executor_.add_node(back_end_mqtt_ptr_);
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager/settings.json");
  Document json_document(kObjectType);
  auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
  if (result) {
    CyberdogJson::Get(json_document, "uid", uid_);
    rapidjson::Value dog_val;
    bool result = CyberdogJson::Get(json_document, "dog_info", dog_val);
    if (result) {
      if (dog_val.HasMember("nick_name")) {
        query_node_ptr_->GetNick() = dog_val["nick_name"].GetString();
      }
      if (dog_val.HasMember("enable")) {
        query_node_ptr_->GetSwitch() = dog_val["enable"].GetBool();
      }
    }
    query_node_ptr_->SetUid(uid_);
    INFO(
      "uid:%s, nick name:%s, switch:%s", uid_.c_str(),
      query_node_ptr_->GetNick().c_str(), query_node_ptr_->GetSwitch() == true ? "on" : "off");
  }
  uid_sub_ =
    query_node_feedback_ptr_->create_subscription<std_msgs::msg::String>(
    "uid_set", rclcpp::SystemDefaultsQoS(),
    std::bind(&CyberdogManager::UidCallback, this, std::placeholders::_1));
  dog_info_update_sub_ =
    query_node_feedback_ptr_->create_subscription<std_msgs::msg::Bool>(
    "dog_info_update", rclcpp::SystemDefaultsQoS(),
    std::bind(&CyberdogManager::DogInfoUpdate, this, std::placeholders::_1));
  device_info_get_srv_ =
    query_node_srv_ptr_->create_service<protocol::srv::DeviceInfo>(
    "query_divice_info",
    std::bind(
      &CyberdogManager::QueryDeviceInfo, this, std::placeholders::_1,
      std::placeholders::_2));
  uid_sn_srv_ =
    query_node_feedback_ptr_->create_service<protocol::srv::UidSn>(
    "uid_sn",
    std::bind(
      &CyberdogManager::UidSn, this, std::placeholders::_1,
      std::placeholders::_2));
  motion_status_sub_ =
    query_node_feedback_ptr_->create_subscription<protocol::msg::MotionStatus>(
    "motion_status", rclcpp::SystemDefaultsQoS(),
    std::bind(&CyberdogManager::MotionStatus, this, std::placeholders::_1));
  // manager_vec_.emplace_back("device");
  // manager_vec_.emplace_back("sensor");
  // manager_vec_.emplace_back("motion");
  // manager_vec_.emplace_back("perception");
  manager_vec_.emplace_back("audio");

  // black_box_ptr_ = std::make_shared<BlackBox>(node_ptr_);
  heart_beats_ptr_ = std::make_unique<cyberdog::machine::HeartBeats>(500, 5);

  lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();

  account_add_srv_ =
    query_account_add_ptr_->create_service<protocol::srv::AccountAdd>(
    "account_add",
    std::bind(
      &CyberdogManager::QueryAccountAdd, this, std::placeholders::_1,
      std::placeholders::_2));

  account_search_srv_ =
    query_account_add_ptr_->create_service<protocol::srv::AccountSearch>(
    "account_search",
    std::bind(
      &CyberdogManager::QueryAccountSearch, this, std::placeholders::_1,
      std::placeholders::_2));

  account_delete_srv_ =
    query_account_add_ptr_->create_service<protocol::srv::AccountDelete>(
    "account_delete",
    std::bind(
      &CyberdogManager::QueryAccountDelete, this, std::placeholders::_1,
      std::placeholders::_2));

  low_power_consumption_srv_ =
    low_power_consumption_ptr_->create_service<std_srvs::srv::SetBool>(
    "low_power_consumption",
    std::bind(
      &CyberdogManager::EnterLowPower, this, std::placeholders::_1,
      std::placeholders::_2));

  back_to_end_pub_ = back_end_mqtt_ptr_->create_publisher<std_msgs::msg::String>(
    "cyberdog/base_info/submit",
    rclcpp::SystemDefaultsQoS()
  );

  std::thread back_to_end_thread = std::thread(
    [this]() {
      while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30000));
        INFO("[start:back to end upload device info-------------------]");
        bool is_sn = false;
        bool is_version = true;
        bool is_uid = false;
        bool is_nick_name = true;
        bool is_volume = true;
        bool is_mic_state = true;
        bool is_voice_control = true;
        bool is_wifi = true;
        bool is_bat_info = true;
        bool is_motor_temper = true;
        bool is_audio_state = true;
        bool is_device_model = true;
        bool is_stand_up = true;
        std::chrono::seconds timeout(10);
        auto req = std::make_shared<protocol::srv::DeviceInfo::Request>();
        req->enables.resize(20);
        req->enables[0] = is_sn;
        req->enables[1] = is_version;
        req->enables[2] = is_uid;
        req->enables[3] = is_nick_name;
        req->enables[4] = is_volume;
        req->enables[5] = is_mic_state;
        req->enables[6] = is_voice_control;
        req->enables[7] = is_wifi;
        req->enables[8] = is_bat_info;
        req->enables[9] = is_motor_temper;
        req->enables[10] = is_audio_state;
        req->enables[11] = is_device_model;
        req->enables[12] = is_stand_up;
        std_msgs::msg::String msg;
        msg.data = query_node_ptr_->QueryDeviceInfo(req->enables);
        back_to_end_pub_->publish(msg);
        INFO("[stop:back to end upload device info-------------------]");
      }
    });
  back_to_end_thread.detach();
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
    connect_status_sub_ = node_ptr_->create_subscription<protocol::msg::ConnectorStatus>(
      "connector_state", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogManager::ConnectStatus, this, std::placeholders::_1));
    bms_status_sub_ = node_ptr_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&CyberdogManager::BmsStatus, this, std::placeholders::_1));
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
  Document json_info(kObjectType);
  std::string info;
  if (request->enables.size() < 1) {
    CyberdogJson::Add(json_info, "error", "parmameter count error");
    if (!CyberdogJson::Document2String(json_info, info)) {
      ERROR("error while encoding to json");
      info = "{\"error\": \"parmameter count error!\"}";
    }
    response->info = info;
    return;
  }
  response->info = query_node_ptr_->QueryDeviceInfo(request->enables);
}

void cyberdog::manager::CyberdogManager::UidCallback(const std_msgs::msg::String::SharedPtr msg)
{
  uid_ = msg->data;
  query_node_ptr_->SetUid(uid_);
  INFO("user id:%s", uid_.c_str());
  std::thread t([this]() {
      auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
      auto path = local_share_dir + std::string("/toml_config/manager");
      auto json_file = path + "/settings.json";
      Document json_document(kObjectType);
      CyberdogJson::ReadJsonFromFile(json_file, json_document);
      CyberdogJson::Add(json_document, "uid", uid_);
      CyberdogJson::WriteJsonToFile(json_file, json_document);
    });
  t.detach();
}

void cyberdog::manager::CyberdogManager::DogInfoUpdate(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/settings.json");
    Document json_document(kObjectType);
    auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
    if (result) {
      rapidjson::Value dog_val;
      bool result = CyberdogJson::Get(json_document, "dog_info", dog_val);
      if (result) {
        if (dog_val.HasMember("nick_name")) {
          query_node_ptr_->GetNick() = dog_val["nick_name"].GetString();
        }
        if (dog_val.HasMember("enable")) {
          query_node_ptr_->GetSwitch() = dog_val["enable"].GetBool();
        }
      }
      INFO(
        "update nick name:%s, switch %s", query_node_ptr_->GetNick().c_str(),
        (query_node_ptr_->GetSwitch() == true ? "on" : "off"));
    }
  }
}

void cyberdog::manager::CyberdogManager::ConnectStatus(
  const protocol::msg::ConnectorStatus::SharedPtr msg)
{
  if (msg->is_connected) {
    query_node_ptr_->GetWifiPtr()->ssid = msg->ssid;
    query_node_ptr_->GetWifiPtr()->ip = msg->robot_ip;
    query_node_ptr_->GetWifiPtr()->mac = "***********";
    query_node_ptr_->GetWifiPtr()->strength = msg->strength;
  } else {
    query_node_ptr_->GetWifiPtr()->ssid = "****";
    query_node_ptr_->GetWifiPtr()->ip = "****";
    query_node_ptr_->GetWifiPtr()->mac = "****";
    query_node_ptr_->GetWifiPtr()->strength = 0;
  }
}

void cyberdog::manager::CyberdogManager::BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
{
  query_node_ptr_->GetBms() = *msg;
}

void cyberdog::manager::CyberdogManager::MotionStatus(
  const protocol::msg::MotionStatus::SharedPtr msg)
{
  if (msg->motion_id == protocol::msg::MotionID::RECOVERYSTAND) {
    query_node_ptr_->GetStand() = true;
  } else {
    query_node_ptr_->GetStand() = false;
  }
}

void cyberdog::manager::CyberdogManager::EnterLowPower(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  static int r_count = 0;
  INFO("[%d]EnterLowPower %s:start", (r_count + 1), (request->data ? "true" : "false"));
  PM_DEV pd = PM_CAM_ALL;
  unsigned int err;
  int code = -1;
  if (request->data) {
    code = lpc_ptr_->LpcRelease(pd, &err);
    ++r_count;
  } else {
    code = lpc_ptr_->LpcRequest(pd, &err);
    ++r_count;
  }
  response->success = (code == 0 ? true : false);
  INFO("[%d]EnterLowPower %s:stop", (r_count + 1), (request->data ? "true" : "false"));
}

void cyberdog::manager::CyberdogManager::UidSn(
  const protocol::srv::UidSn::Request::SharedPtr request,
  protocol::srv::UidSn::Response::SharedPtr response)
{
  (void) request;
  response->sn = query_node_ptr_->GetSn();
  response->uid = uid_;
}

void cyberdog::manager::CyberdogManager::QueryAccountAdd(
  const protocol::srv::AccountAdd::Request::SharedPtr request,
  protocol::srv::AccountAdd::Response::SharedPtr response)
{
  std::string name = request->member;
  INFO("add_account_name is: %s", name.c_str());
  cyberdog::common::CyberdogAccountManager obj;
  if (obj.AddMember(name)) {
    response->status = true;
    INFO("add_account_success");
  } else {
    response->status = false;
    INFO("add_account_fail");
  }
}

void cyberdog::manager::CyberdogManager::QueryAccountSearch(
  const protocol::srv::AccountSearch::Request::SharedPtr request,
  protocol::srv::AccountSearch::Response::SharedPtr response)
{
  std::string account_name = request->member;
  INFO("search user_name: %s", account_name.c_str());
  Document json_info(rapidjson::kObjectType);
  rapidjson::Value arr(rapidjson::kArrayType);
  std::string data;
  cyberdog::common::CyberdogAccountManager obj;
  if (request->member == "") {
    INFO("search_all_user");
    std::vector<cyberdog::common::CyberdogAccountManager::UserInformation> member_data;
    if (obj.SearAllUser(member_data)) {
      for (unsigned int i = 0; i < member_data.size(); ++i) {
        rapidjson::Value js_obj(rapidjson::kObjectType);
        rapidjson::Value value(member_data[i].username.c_str(), json_info.GetAllocator());
        js_obj.AddMember("name", value, json_info.GetAllocator());
        js_obj.AddMember("face_state", member_data[i].faceStatus, json_info.GetAllocator());
        js_obj.AddMember("voice_state", member_data[i].voiceStatus, json_info.GetAllocator());
        arr.PushBack(js_obj, json_info.GetAllocator());
      }
      json_info.AddMember("accounts", arr, json_info.GetAllocator());
      response->status = true;
    } else {
      INFO("Search ALL Account faile!");
      response->status = false;
      response->data = "{\"error\":\"search failed!\"}";
    }
  } else {
    INFO("search_one_user");
    int result[2];
    rapidjson::Value js_obj(rapidjson::kObjectType);
    obj.SearchUser(account_name, result);
    int face_state = result[0];
    int voice_state = result[1];
    INFO(
      "account name:%s \n voice_state: %d \n face_stare: %d",
      account_name.c_str(), result[0], result[1]);
    rapidjson::Value value(account_name.c_str(), json_info.GetAllocator());
    js_obj.AddMember("name", value, json_info.GetAllocator());
    js_obj.AddMember("face_state", face_state, json_info.GetAllocator());
    js_obj.AddMember("voice_state", voice_state, json_info.GetAllocator());
    arr.PushBack(js_obj, json_info.GetAllocator());
    json_info.AddMember("accounts", arr, json_info.GetAllocator());
    response->status = true;
  }

  if (!CyberdogJson::Document2String(json_info, data)) {
    ERROR("error while encoding to json");
    data = "{\"error\": \"unkown encoding json error!\"}";
  }
  response->data = data;
  INFO("Search result is :%s", data.c_str());
}

void cyberdog::manager::CyberdogManager::QueryAccountDelete(
  const protocol::srv::AccountDelete::Request::SharedPtr request,
  protocol::srv::AccountDelete::Response::SharedPtr response)
{
  INFO("enter delete account callback");
  // voice delete
  rclcpp::Client<protocol::srv::AudioVoiceprintEntry>::SharedPtr voice_delete_client_;
  voice_delete_client_ =
    node_ptr_->create_client<protocol::srv::AudioVoiceprintEntry>("audio_voiceprint_entry");
  if (!voice_delete_client_->wait_for_service(std::chrono::seconds(2))) {
    ERROR("call voice service server not avalible");
    return;
  }
  auto request_voice = std::make_shared<protocol::srv::AudioVoiceprintEntry::Request>();
  request_voice->command = 4;
  request_voice->username = request->member;
  std::chrono::seconds timeout(3);
  auto future_result_voice = voice_delete_client_->async_send_request(request_voice);
  std::future_status status_face = future_result_voice.wait_for(timeout);
  if (status_face == std::future_status::ready) {
    INFO(
      "success to call vocie delete response services.");
  } else {
    INFO(
      "Failed to call vocie delete response services.");
    return;
  }

  // face delete
  rclcpp::Client<protocol::srv::FaceEntry>::SharedPtr face_delete_client_;
  face_delete_client_ =
    node_ptr_->create_client<protocol::srv::FaceEntry>("/cyberdog_face_entry_srv");

  if (!face_delete_client_->wait_for_service(std::chrono::seconds(2))) {
    ERROR("call face service server not avalible");
    return;
  }
  auto request_face = std::make_shared<protocol::srv::FaceEntry::Request>();
  request_face->command = 4;
  request_face->username = request->member;

  auto future_result_face = face_delete_client_->async_send_request(request_face);
  std::future_status status_voice = future_result_voice.wait_for(timeout);
  if (status_voice == std::future_status::ready) {
    INFO(
      "success to call face delete response services.");
  } else {
    INFO(
      "Failed to call face delete response services.");
    return;
  }

  // delete account
  std::string name = request->member;
  INFO("delete_account_name is: %s", name.c_str());
  if (future_result_voice.get()->success) {
    if (future_result_face.get()->result == 0) {
      cyberdog::common::CyberdogAccountManager obj;
      if (obj.DeleteUserInformation(name)) {
        INFO("delete_account_success");
        response->status = true;
      } else {
        INFO("delete_account_fail");
        response->status = false;
      }
    } else {
      INFO("delete face faile");
      response->status = false;
    }
  } else {
    INFO("delete voice faile");
    response->status = false;
  }
}
