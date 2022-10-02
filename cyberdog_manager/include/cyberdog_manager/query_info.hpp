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
#ifndef CYBERDOG_MANAGER__QUERY_INFO_HPP_
#define CYBERDOG_MANAGER__QUERY_INFO_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/ota_server_cmd.hpp"
#include "protocol/srv/audio_volume_get.hpp"
#include "protocol/srv/audio_execute.hpp"
#include "protocol/srv/motor_temp.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{

struct WifiInfo
{
  std::string ssid;
  std::string ip;
  std::string mac;
  uint8_t strength;
};  // struct WifiInfo

class QueryInfo final
{
public:
  explicit QueryInfo(std::string node_name)
  {
    query_node_ptr_ = rclcpp::Node::make_shared(node_name + "_auxiliary");
    wifi_info_ptr_ = std::make_unique<cyberdog::manager::WifiInfo>();
    audio_sn_ger_srv_ =
      query_node_ptr_->create_client<std_srvs::srv::Trigger>("get_dog_sn");
    ota_ver_get_srv_ =
      query_node_ptr_->create_client<protocol::srv::OtaServerCmd>("ota_versions");
    audio_volume_get_client_ =
      query_node_ptr_->create_client<protocol::srv::AudioVolumeGet>("audio_volume_get");
    audio_execute_client_ =
      query_node_ptr_->create_client<protocol::srv::AudioExecute>("get_audio_state");
    audio_action_get_client_ =
      query_node_ptr_->create_client<std_srvs::srv::Trigger>("audio_action_get");
    motor_temper_client_ =
      query_node_ptr_->create_client<protocol::srv::MotorTemp>("motor_temp");
    audio_active_state_client_ =
      query_node_ptr_->create_client<std_srvs::srv::Trigger>("audio_active_state");
    std::thread(
      [this]() {
        rclcpp::spin(query_node_ptr_);
      }).detach();
  }

  ~QueryInfo()
  {
  }

  const std::string & GetSn()
  {
    return sn_;
  }

  void SetUid(const std::string & uid)
  {
    uid_ = uid;
  }

  void SetBms(protocol::msg::BmsStatus & bms_status)
  {
    bms_status_ = bms_status;
  }

  bool & GetSwitch()
  {
    return name_switch_;
  }

  std::string & GetNick()
  {
    return nick_name_;
  }

  protocol::msg::BmsStatus & GetBms()
  {
    return bms_status_;
  }

  bool & GetStand()
  {
    return standed_;
  }

  std::unique_ptr<cyberdog::manager::WifiInfo> & GetWifiPtr()
  {
    return wifi_info_ptr_;
  }

  const std::string QueryDeviceInfo(std::vector<bool> & enables)
  {
    Document json_info(kObjectType);
    std::string info;
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
    if (enables.size() > 0) {
      is_sn = enables[0];
    }
    if (enables.size() > 1) {
      is_version = enables[1];
    }
    if (enables.size() > 2) {
      is_uid = enables[2];
    }
    if (enables.size() > 3) {
      is_nick_name = enables[3];
    }
    if (enables.size() > 4) {
      is_volume = enables[4];
    }
    if (enables.size() > 5) {
      is_mic_state = enables[5];
    }
    if (enables.size() > 6) {
      is_voice_control = enables[6];
    }
    if (enables.size() > 7) {
      is_wifi = enables[7];
    }
    if (enables.size() > 8) {
      is_bat_info = enables[8];
    }
    if (enables.size() > 9) {
      is_motor_temper = enables[9];
    }
    if (enables.size() > 10) {
      is_audio_state = enables[10];
    }
    if (enables.size() > 11) {
      is_device_model = enables[11];
    }
    if (enables.size() > 12) {
      is_stand_up = enables[12];
    }
    if (is_sn) {
      if (sn_ == "") {
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
        name_val.AddMember(
          "current_name", rval.SetString(nick_name_.c_str(), allocator),
          allocator);
      } else {
        name_val.AddMember(
          "current_name", rval.SetString(default_name_.c_str(), allocator),
          allocator);
      }
      name_val.AddMember(
        "default_name", rval.SetString(default_name_.c_str(), allocator),
        allocator);
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
        req->client = node_name_;
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
      wifi_val.AddMember(
        "name", rval.SetString(wifi_info_ptr_->ssid.c_str(), allocator),
        allocator);
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
    return info;
  }

private:
  const std::string float_to_string(float & val)
  {
    std::stringstream ss;
    ss.setf(std::ios::fixed);
    ss.precision(2);
    ss << val;
    return ss.str();
  }

private:
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Node::SharedPtr query_node_ptr_ {nullptr};
  std::string node_name_;
  std::string sn_ {""};
  std::string uid_ {""};
  bool name_switch_ {false};
  std::string default_name_ {"铁蛋"};
  std::string nick_name_ {"铁蛋"};
  protocol::msg::BmsStatus bms_status_;
  bool standed_{false};
  std::unique_ptr<cyberdog::manager::WifiInfo> wifi_info_ptr_ {nullptr};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_sn_ger_srv_;
  rclcpp::Client<protocol::srv::OtaServerCmd>::SharedPtr ota_ver_get_srv_;
  rclcpp::Client<protocol::srv::AudioVolumeGet>::SharedPtr audio_volume_get_client_;
  rclcpp::Client<protocol::srv::AudioExecute>::SharedPtr audio_execute_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_action_get_client_;
  rclcpp::Client<protocol::srv::MotorTemp>::SharedPtr motor_temper_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_active_state_client_;
};

}  // namespace manager
}  // namespace cyberdog
#endif  // CYBERDOG_MANAGER__QUERY_INFO_HPP_
