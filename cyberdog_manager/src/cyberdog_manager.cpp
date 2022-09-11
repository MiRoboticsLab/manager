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

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

const std::string float_to_string(float & val)
{
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(2);
  ss << val;
  return ss.str();
}

cyberdog::manager::CyberdogManager::CyberdogManager(const std::string & name)
: ManagerBase(name),
  name_(name), has_error_(false), sn_(""), uid_(""),
  name_switch_(false), default_name_("铁蛋"), nick_name_("铁蛋")
{
  node_ptr_ = rclcpp::Node::make_shared(name_);
  query_node_ptr_ = rclcpp::Node::make_shared(name_ + "_query");
  query_node_feedback_ptr_ = rclcpp::Node::make_shared(name_ + "_query_feedback");
  executor_.add_node(node_ptr_);
  executor_.add_node(query_node_ptr_);
  executor_.add_node(query_node_feedback_ptr_);
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
        nick_name_ = dog_val["nick_name"].GetString();
      }
      if (dog_val.HasMember("enable")) {
        name_switch_ = dog_val["enable"].GetBool();
      }
    }
    INFO(
      "uid:%s, nick name:%s, switch:%s", uid_.c_str(),
      nick_name_.c_str(), name_switch_ == true ? "on" : "off");
  }
  wifi_info_ptr_ = std::make_unique<cyberdog::manager::WifiInfo>();
  uid_sub_ =
    query_node_feedback_ptr_->create_subscription<std_msgs::msg::String>(
    "uid_set", rclcpp::SystemDefaultsQoS(),
    std::bind(&CyberdogManager::UidCallback, this, std::placeholders::_1));
  dog_info_update_sub_ =
    query_node_feedback_ptr_->create_subscription<std_msgs::msg::Bool>(
    "dog_info_update", rclcpp::SystemDefaultsQoS(),
    std::bind(&CyberdogManager::DogInfoUpdate, this, std::placeholders::_1));
  device_info_get_srv_ =
    query_node_ptr_->create_service<protocol::srv::DeviceInfo>(
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
  audio_volume_get_client_ =
    query_node_feedback_ptr_->create_client<protocol::srv::AudioVolumeGet>("audio_volume_get");
  audio_execute_client_ =
    query_node_feedback_ptr_->create_client<protocol::srv::AudioExecute>("get_audio_state");
  audio_action_get_client_ =
    query_node_feedback_ptr_->create_client<std_srvs::srv::Trigger>("audio_action_get");
  motor_temper_client_ =
    query_node_feedback_ptr_->create_client<protocol::srv::MotorTemp>("motor_temp");
  audio_active_state_client_ =
    query_node_feedback_ptr_->create_client<std_srvs::srv::Trigger>("audio_active_state");
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
  bool is_sn = false;
  bool is_version = false;
  bool is_uid = false;
  bool is_nick_name = false;
  bool is_volume = false;
  bool is_mic_state = false;
  bool is_voice_control = false;
  bool is_wifi = false;
  bool is_bat_info = false;
  bool is_motor_temper = false;
  bool is_audio_state = false;
  bool is_device_model = false;
  bool is_stand_up = false;
  if (request->enables.size() > 0) {
    is_sn = request->enables[0];
  }
  if (request->enables.size() > 1) {
    is_version = request->enables[1];
  }
  if (request->enables.size() > 2) {
    is_uid = request->enables[2];
  }
  if (request->enables.size() > 3) {
    is_nick_name = request->enables[3];
  }
  if (request->enables.size() > 4) {
    is_volume = request->enables[4];
  }
  if (request->enables.size() > 5) {
    is_mic_state = request->enables[5];
  }
  if (request->enables.size() > 6) {
    is_voice_control = request->enables[6];
  }
  if (request->enables.size() > 7) {
    is_wifi = request->enables[7];
  }
  if (request->enables.size() > 8) {
    is_bat_info = request->enables[8];
  }
  if (request->enables.size() > 9) {
    is_motor_temper = request->enables[9];
  }
  if (request->enables.size() > 10) {
    is_audio_state = request->enables[10];
  }
  if (request->enables.size() > 11) {
    is_device_model = request->enables[11];
  }
  if (request->enables.size() > 12) {
    is_stand_up = request->enables[12];
  }
  if (is_sn) {
    if (sn_ == "") {
      rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_sn_ger_srv_;
      audio_sn_ger_srv_ =
        node_ptr_->create_client<std_srvs::srv::Trigger>("get_dog_sn");
      if (!audio_sn_ger_srv_->wait_for_service(std::chrono::seconds(2))) {
        ERROR("call sn server not avalible");
        sn_ = "unaviable";
      } else {
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
    }
    CyberdogJson::Add(json_info, "sn", sn_);
  }
  if (is_version) {
    rclcpp::Client<protocol::srv::OtaServerCmd>::SharedPtr ota_ver_get_srv_;
    ota_ver_get_srv_ =
      node_ptr_->create_client<protocol::srv::OtaServerCmd>("ota_versions");
    if (!ota_ver_get_srv_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call ota version not avalible");
      CyberdogJson::Add(json_info, "version", "unaviable");
    } else {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<protocol::srv::OtaServerCmd::Request>();
      req->request.key = "ota_command_version_query";
      auto future_result = ota_ver_get_srv_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        std::string version = future_result.get()->response.value;
        Document version_doc(kObjectType);
        if (!CyberdogJson::String2Document(version, version_doc)) {
          ERROR("error while encoding version info to json");
          CyberdogJson::Add(version_doc, "version", "exception");
        } else {
          CyberdogJson::Add(json_info, "version", version_doc);
        }
      } else {
        ERROR("call ota version failed!");
        CyberdogJson::Add(json_info, "version", "unkown");
      }
    }
  }
  if (is_uid) {
    CyberdogJson::Add(json_info, "uid", uid_);
  }
  if (is_nick_name) {
    rapidjson::Value name_val(rapidjson::kObjectType);
    Document::AllocatorType & allocator = json_info.GetAllocator();
    rapidjson::Value rval;
    if (name_switch_) {
      name_val.AddMember("current_name", rval.SetString(nick_name_.c_str(), allocator), allocator);
    } else {
      name_val.AddMember(
        "current_name", rval.SetString(default_name_.c_str(), allocator),
        allocator);
    }
    name_val.AddMember("default_name", rval.SetString(default_name_.c_str(), allocator), allocator);
    CyberdogJson::Add(json_info, "nick_name", name_val);
  }
  if (is_volume) {
    if (!audio_volume_get_client_->wait_for_service(std::chrono::seconds(2))) {
      INFO(
        "call VolumeGet server not avalible");
      CyberdogJson::Add(json_info, "volume", -255);
    } else {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<protocol::srv::AudioVolumeGet::Request>();
      auto future_result = audio_volume_get_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO(
          "success to call volumeget services.");
        CyberdogJson::Add(json_info, "volume", future_result.get()->volume);
      } else {
        INFO(
          "Failed to call volumeget services.");
        CyberdogJson::Add(json_info, "volume", -1);
      }
    }
  }
  if (is_mic_state) {
    if (!audio_execute_client_->wait_for_service(std::chrono::seconds(2))) {
      INFO(
        "call mic state server not avalible");
      CyberdogJson::Add(json_info, "mic_state", false);
    } else {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<protocol::srv::AudioExecute::Request>();
      req->client = name_;
      auto future_result = audio_execute_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO(
          "success to call audio execute services.");
        CyberdogJson::Add(json_info, "mic_state", future_result.get()->result);
      } else {
        INFO(
          "Failed to call audio execute services.");
        CyberdogJson::Add(json_info, "mic_state", false);
      }
    }
  }
  if (is_voice_control) {
    if (!audio_action_get_client_->wait_for_service(std::chrono::seconds(2))) {
      INFO(
        "call voice control server not avalible");
      CyberdogJson::Add(json_info, "voice_control", false);
    } else {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future_result = audio_action_get_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO(
          "success to call voice control services.");
        CyberdogJson::Add(json_info, "voice_control", future_result.get()->success);
      } else {
        INFO(
          "Failed to call voice control services.");
        CyberdogJson::Add(json_info, "voice_control", false);
      }
    }
  }
  if (is_wifi) {
    rapidjson::Value wifi_val(rapidjson::kObjectType);
    Document::AllocatorType & allocator = json_info.GetAllocator();
    rapidjson::Value rval;
    wifi_val.AddMember("name", rval.SetString(wifi_info_ptr_->ssid.c_str(), allocator), allocator);
    wifi_val.AddMember("ip", rval.SetString(wifi_info_ptr_->ip.c_str(), allocator), allocator);
    wifi_val.AddMember("mac", rval.SetString(wifi_info_ptr_->mac.c_str(), allocator), allocator);
    wifi_val.AddMember("strength", wifi_info_ptr_->strength, allocator);
    CyberdogJson::Add(json_info, "wifi", wifi_val);
  }
  if (is_bat_info) {
    rapidjson::Value bat_val(rapidjson::kObjectType);
    Document::AllocatorType & allocator = json_info.GetAllocator();
    bat_val.AddMember("capacity", bms_status_.batt_volt, allocator);
    bat_val.AddMember("power", bms_status_.batt_soc, allocator);
    bat_val.AddMember("voltage", bms_status_.batt_volt, allocator);
    bat_val.AddMember("temperature", bms_status_.batt_temp, allocator);
    bat_val.AddMember(
      "is_charging", ((bms_status_.batt_st & 2) >> 1) == 1 ? true : false,
      allocator);
    bat_val.AddMember("discharge_time", 120, allocator);
    CyberdogJson::Add(json_info, "bat_info", bat_val);
  }
  if (is_motor_temper) {
    rapidjson::Value motor_temper_val(rapidjson::kObjectType);
    if (!motor_temper_client_->wait_for_service(std::chrono::seconds(2))) {
      INFO(
        "call mic motor temper server not avalible");
      CyberdogJson::Add(json_info, "motor_temper", "unavalible");
    } else {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<protocol::srv::MotorTemp::Request>();
      auto future_result = motor_temper_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO(
          "success to call motor temper services.");
        rapidjson::Value hip_temper_array(rapidjson::kArrayType);
        float & hip_left_front = future_result.get()->motor_temp[1];
        float & hip_right_front = future_result.get()->motor_temp[0];
        float & hip_left_back = future_result.get()->motor_temp[3];
        float & hip_right_back = future_result.get()->motor_temp[2];
        rapidjson::Value val;
        std::string ss = float_to_string(hip_left_front);
        val.SetString(ss.c_str(), ss.length());
        hip_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(hip_right_front);
        val.SetString(ss.c_str(), ss.length());
        hip_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(hip_left_back);
        val.SetString(ss.c_str(), ss.length());
        hip_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(hip_right_back);
        val.SetString(ss.c_str(), ss.length());
        hip_temper_array.PushBack(
          val, json_info.GetAllocator());
        motor_temper_val.AddMember("hip", hip_temper_array, json_info.GetAllocator());
        rapidjson::Value thigh_temper_array(rapidjson::kArrayType);
        float & thigh_left_front = future_result.get()->motor_temp[5];
        float & thigh_right_front = future_result.get()->motor_temp[4];
        float & thigh_left_back = future_result.get()->motor_temp[7];
        float & thigh_right_back = future_result.get()->motor_temp[6];
        ss = float_to_string(thigh_left_front);
        val.SetString(ss.c_str(), ss.length());
        thigh_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(thigh_right_front);
        val.SetString(ss.c_str(), ss.length());
        thigh_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(thigh_left_back);
        val.SetString(ss.c_str(), ss.length());
        thigh_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(thigh_right_back);
        val.SetString(ss.c_str(), ss.length());
        thigh_temper_array.PushBack(
          val, json_info.GetAllocator());
        motor_temper_val.AddMember("thigh", thigh_temper_array, json_info.GetAllocator());
        rapidjson::Value crus_temper_array(rapidjson::kArrayType);
        float & crus_left_front = future_result.get()->motor_temp[9];
        float & crus_right_front = future_result.get()->motor_temp[8];
        float & crus_left_back = future_result.get()->motor_temp[11];
        float & crus_right_back = future_result.get()->motor_temp[10];
        ss = float_to_string(crus_left_front);
        val.SetString(ss.c_str(), ss.length());
        crus_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(crus_right_front);
        val.SetString(ss.c_str(), ss.length());
        crus_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(crus_left_back);
        val.SetString(ss.c_str(), ss.length());
        crus_temper_array.PushBack(
          val, json_info.GetAllocator());
        ss = float_to_string(crus_right_back);
        val.SetString(ss.c_str(), ss.length());
        crus_temper_array.PushBack(
          val, json_info.GetAllocator());
        motor_temper_val.AddMember("crus", crus_temper_array, json_info.GetAllocator());
        CyberdogJson::Add(json_info, "motor_temper", motor_temper_val);
      } else {
        INFO(
          "Failed to call amotor temper services.");
        CyberdogJson::Add(json_info, "motor_temper", "unkown");
      }
    }
  }
  if (is_audio_state) {
    if (uid_ == "") {
      INFO("uid does not exist!");
      CyberdogJson::Add(json_info, "audio_state", false);
    } else {
      if (!audio_active_state_client_->wait_for_service(std::chrono::seconds(2))) {
        INFO(
          "call audio active state server not avalible");
        CyberdogJson::Add(json_info, "audio_state", false);
      } else {
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future_result = audio_active_state_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call audio active state services.");
          CyberdogJson::Add(json_info, "audio_state", future_result.get()->success);
        } else {
          INFO(
            "Failed to call audio active state services.");
          CyberdogJson::Add(json_info, "audio_state", false);
        }
      }
    }
  }
  if (is_device_model) {
    CyberdogJson::Add(json_info, "device_model", "MS2241CN");
  }
  if (is_stand_up) {
    CyberdogJson::Add(json_info, "stand", standed_);
  }
  if (!CyberdogJson::Document2String(json_info, info)) {
    ERROR("error while encoding to json");
    info = "{\"error\": \"unkown encoding json error!\"}";
  }
  response->info = info;
}

void cyberdog::manager::CyberdogManager::UidCallback(const std_msgs::msg::String::SharedPtr msg)
{
  uid_ = msg->data;
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
          nick_name_ = dog_val["nick_name"].GetString();
        }
        if (dog_val.HasMember("enable")) {
          name_switch_ = dog_val["enable"].GetBool();
        }
      }
      INFO("update nick name:%s", nick_name_.c_str());
    }
  }
}

void cyberdog::manager::CyberdogManager::ConnectStatus(
  const protocol::msg::ConnectorStatus::SharedPtr msg)
{
  if (msg->is_connected) {
    wifi_info_ptr_->ssid = msg->ssid;
    wifi_info_ptr_->ip = msg->robot_ip;
    wifi_info_ptr_->mac = "***********";
    wifi_info_ptr_->strength = msg->strength;
  } else {
    wifi_info_ptr_->ssid = "****";
    wifi_info_ptr_->ip = "****";
    wifi_info_ptr_->mac = "****";
    wifi_info_ptr_->strength = 0;
  }
}

void cyberdog::manager::CyberdogManager::BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
{
  bms_status_ = *msg;
}

void cyberdog::manager::CyberdogManager::MotionStatus(
  const protocol::msg::MotionStatus::SharedPtr msg)
{
  if (msg->motion_id == protocol::msg::MotionID::RECOVERYSTAND) {
    standed_ = true;
  } else {
    standed_ = false;
  }
}

void cyberdog::manager::CyberdogManager::UidSn(
  const protocol::srv::UidSn::Request::SharedPtr request,
  protocol::srv::UidSn::Response::SharedPtr response)
{
  (void) request;
  response->sn = sn_;
  response->uid = uid_;
}
