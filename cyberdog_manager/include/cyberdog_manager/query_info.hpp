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
#ifndef CYBERDOG_MANAGER__QUERY_INFO_HPP_
#define CYBERDOG_MANAGER__QUERY_INFO_HPP_

#include <string>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "protocol/srv/audio_volume_get.hpp"
#include "protocol/srv/audio_execute.hpp"
#include "protocol/srv/motor_temp.hpp"
#include "protocol/srv/device_info.hpp"
#include "protocol/msg/motion_id.hpp"
#include "protocol/msg/connector_status.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/srv/uid_sn.hpp"
#include "protocol/srv/audio_nick_name.hpp"
#include "protocol/srv/trigger.hpp"
#include "protocol/action/over_the_air.hpp"

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
  explicit QueryInfo(rclcpp::Node::SharedPtr node_ptr)
  {
    query_node_ptr_ = node_ptr;
    qdev_callback_group_ =
      query_node_ptr_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    wifi_info_ptr_ = std::make_unique<cyberdog::manager::WifiInfo>();
    sn_notify_sub_ =
      query_node_ptr_->create_subscription<std_msgs::msg::String>(
      "dog_sn", rclcpp::SystemDefaultsQoS(),
      std::bind(&QueryInfo::DogsnCallback, this, std::placeholders::_1));
    audio_sn_ger_srv_ =
      query_node_ptr_->create_client<std_srvs::srv::Trigger>(
      "get_dog_sn",
      rmw_qos_profile_services_default, qdev_callback_group_);
    audio_volume_get_client_ =
      query_node_ptr_->create_client<protocol::srv::AudioVolumeGet>(
      "audio_volume_get",
      rmw_qos_profile_services_default, qdev_callback_group_);
    audio_execute_client_ =
      query_node_ptr_->create_client<protocol::srv::AudioExecute>(
      "get_audio_state",
      rmw_qos_profile_services_default, qdev_callback_group_);
    audio_action_get_client_ =
      query_node_ptr_->create_client<std_srvs::srv::Trigger>(
      "audio_action_get",
      rmw_qos_profile_services_default, qdev_callback_group_);
    motor_temper_client_ =
      query_node_ptr_->create_client<protocol::srv::MotorTemp>(
      "motor_temp",
      rmw_qos_profile_services_default, qdev_callback_group_);
    audio_active_state_client_ =
      query_node_ptr_->create_client<protocol::srv::Trigger>(
      "audio_active_state",
      rmw_qos_profile_services_default, qdev_callback_group_);
    low_power_switch_state_client_ =
      query_node_ptr_->create_client<std_srvs::srv::Trigger>(
      "low_power_switch_state",
      rmw_qos_profile_services_default, qdev_callback_group_);
    version_get_client_ = rclcpp_action::create_client<protocol::action::OverTheAir>(
      query_node_ptr_,
      "cyberdog_ota_action");
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

  bool & GetNonStand()
  {
    return non_stand_;
  }

  std::unique_ptr<cyberdog::manager::WifiInfo> & GetWifiPtr()
  {
    return wifi_info_ptr_;
  }

  void DogsnCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    sn_ = msg->data;
  }

  const std::string QueryDeviceInfo(std::vector<bool> & enables, std::string app_uid = "")
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
    bool is_lowpower_control = false;
    bool is_charging = false;
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
    if (enables.size() > 13) {
      is_lowpower_control = enables[13];
    }
    if (enables.size() > 14) {
      is_charging = enables[14];
    }
    if (is_sn) {
      if (sn_ == "") {
        if (!audio_sn_ger_srv_->wait_for_service(std::chrono::seconds(2))) {
          ERROR("call sn server wait for service not avalible[sn:unaviable]");
          // sn_ = "unaviable";
          sn_ = "";
        } else {
          std::chrono::seconds timeout(3);
          auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
          auto future_result = audio_sn_ger_srv_->async_send_request(req);
          std::future_status status = future_result.wait_for(timeout);
          if (status == std::future_status::ready) {
            sn_ = future_result.get()->message;
          } else {
            ERROR("call get sn service overtime failed![sn:unkown]");
            // sn_ = "unkown";
            sn_ = "";
          }
        }
      }
      CyberdogJson::Add(json_info, "sn", sn_);
    }
    if (is_version) {
      if (version_.empty()) {
        GetVersion();
      }
      Document version_doc(kObjectType);
      CyberdogJson::String2Document(version_, version_doc);
      CyberdogJson::Add(json_info, "version", version_doc);
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
      bat_val.AddMember("capacity", 4500, allocator);
      bat_val.AddMember("power", bms_status_.batt_soc, allocator);
      bat_val.AddMember("voltage", bms_status_.batt_volt, allocator);
      bat_val.AddMember("temperature", bms_status_.batt_temp, allocator);
      bat_val.AddMember(
        "is_charging", (bms_status_.power_wired_charging || bms_status_.power_wp_charging),
        allocator);
      bat_val.AddMember("charge_over_current", bms_status_.charge_over_current, allocator);
      bat_val.AddMember("discharge_over_current", bms_status_.discharge_over_current, allocator);
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
          // 髋部-hip;大腿-thigh;小腿-crus
          float & hip_left_front = future_result.get()->motor_temp[3];
          float & hip_right_front = future_result.get()->motor_temp[0];
          float & hip_left_back = future_result.get()->motor_temp[9];
          float & hip_right_back = future_result.get()->motor_temp[6];
          float & thigh_left_front = future_result.get()->motor_temp[4];
          float & thigh_right_front = future_result.get()->motor_temp[1];
          float & thigh_left_back = future_result.get()->motor_temp[10];
          float & thigh_right_back = future_result.get()->motor_temp[7];
          float & crus_left_front = future_result.get()->motor_temp[5];
          float & crus_right_front = future_result.get()->motor_temp[2];
          float & crus_left_back = future_result.get()->motor_temp[11];
          float & crus_right_back = future_result.get()->motor_temp[8];
          rapidjson::Value val(rapidjson::kStringType);
          std::string ss = float_to_string(hip_left_front);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          hip_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(hip_right_front);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          hip_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(hip_left_back);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          hip_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(hip_right_back);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          hip_temper_array.PushBack(
            val, json_info.GetAllocator());
          motor_temper_val.AddMember("hip", hip_temper_array, json_info.GetAllocator());
          rapidjson::Value thigh_temper_array(rapidjson::kArrayType);
          ss = float_to_string(thigh_left_front);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          thigh_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(thigh_right_front);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          thigh_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(thigh_left_back);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          thigh_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(thigh_right_back);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          thigh_temper_array.PushBack(
            val, json_info.GetAllocator());
          motor_temper_val.AddMember("thigh", thigh_temper_array, json_info.GetAllocator());
          rapidjson::Value crus_temper_array(rapidjson::kArrayType);
          ss = float_to_string(crus_left_front);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          crus_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(crus_right_front);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          crus_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(crus_left_back);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
          crus_temper_array.PushBack(
            val, json_info.GetAllocator());
          ss = float_to_string(crus_right_back);
          val.SetString(ss.c_str(), ss.length(), json_info.GetAllocator());
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
          auto req = std::make_shared<protocol::srv::Trigger::Request>();
          req->data = app_uid;
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
    if (is_lowpower_control) {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future_result = low_power_switch_state_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO(
          "success to call get low power switch state services.");
        CyberdogJson::Add(json_info, "lowpower_control", future_result.get()->success);
      } else {
        INFO(
          "Failed to call get low power switch state services.");
        CyberdogJson::Add(json_info, "lowpower_control", false);
      }
    }
    if (is_charging) {
      bool charging = (bms_status_.power_wired_charging | bms_status_.power_wp_charging);
      CyberdogJson::Add(json_info, "charging", charging);
    }
    CyberdogJson::Add(json_info, "non-standing", non_stand_);
    if (!CyberdogJson::Document2String(json_info, info)) {
      ERROR("error while encoding to json");
      info = "{\"error\": \"unkown encoding json error!\"}";
    }

    // INFO("info:%s", info.c_str());
    return info;
  }

  void GetVersion()
  {
    if (!version_get_client_->wait_for_action_server(std::chrono::seconds(5))) {
      ERROR("call ota version not avalible");
    } else {
      auto goal = protocol::action::OverTheAir::Goal();
      goal.id = "query_divice_info";
      goal.operate = "inquire";
      goal.user = "cyberdog_manager";
      auto send_goal_options =
        rclcpp_action::Client<protocol::action::OverTheAir>::SendGoalOptions();
      using GoalHandleOverTheAir = rclcpp_action::ClientGoalHandle<protocol::action::OverTheAir>;
      auto get_result = [&](const GoalHandleOverTheAir::WrappedResult & result) -> void {
          if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            ERROR("get version failed");
            return;
          }
          const std::string version = result.result->result_msg;
          rapidjson::Document tmp_doc;
          rapidjson::Value tem_val;
          std::string tmp_version;
          if (version.empty()) {
            INFO("query version is empty");
            return;
          }
          CyberdogJson::String2Document(version, tmp_doc);
          CyberdogJson::Get(tmp_doc, "version", tem_val);
          CyberdogJson::Value2String(tem_val, tmp_version);
          version_ = tmp_version;
        };
      send_goal_options.result_callback = get_result;
      auto goal_handle_futrue = version_get_client_->async_send_goal(goal, send_goal_options);
      std::future_status status = goal_handle_futrue.wait_for(std::chrono::seconds(3));
      if (status != std::future_status::ready) {
        WARN("query version is failed");
      }
    }
    sleep(5);
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
  rclcpp::Node::SharedPtr query_node_ptr_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr qdev_callback_group_;
  std::string node_name_;
  std::string sn_ {""};
  std::string uid_ {""};
  std::string version_ {""};
  bool name_switch_ {false};
  std::string default_name_ {"铁蛋"};
  std::string nick_name_ {"铁蛋"};
  protocol::msg::BmsStatus bms_status_;
  bool standed_{false};
  bool non_stand_{false};
  std::unique_ptr<cyberdog::manager::WifiInfo> wifi_info_ptr_ {nullptr};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sn_notify_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_sn_ger_srv_;
  rclcpp::Client<protocol::srv::AudioVolumeGet>::SharedPtr audio_volume_get_client_;
  rclcpp::Client<protocol::srv::AudioExecute>::SharedPtr audio_execute_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr audio_action_get_client_;
  rclcpp::Client<protocol::srv::MotorTemp>::SharedPtr motor_temper_client_;
  rclcpp::Client<protocol::srv::Trigger>::SharedPtr audio_active_state_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr low_power_switch_state_client_;
  rclcpp_action::Client<protocol::action::OverTheAir>::SharedPtr version_get_client_;
};

class QueryInfoNode final
{
public:
  explicit QueryInfoNode(rclcpp::Node::SharedPtr node_ptr)
  : uid_("")
  {
    query_info_node_ = node_ptr;
    query_srv_callback_group_ =
      query_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    device_info_get_srv_ =
      query_info_node_->create_service<protocol::srv::DeviceInfo>(
      "query_divice_info",
      std::bind(
        &QueryInfoNode::QueryDeviceInfo, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, query_srv_callback_group_);
    query_feedback_callback_group_ =
      query_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = query_feedback_callback_group_;
    uid_sub_ =
      query_info_node_->create_subscription<std_msgs::msg::String>(
      "uid_set", rclcpp::SystemDefaultsQoS(),
      std::bind(&QueryInfoNode::UidCallback, this, std::placeholders::_1),
      sub_options);
    dog_info_update_sub_ =
      query_info_node_->create_subscription<std_msgs::msg::Bool>(
      "dog_info_update", rclcpp::SystemDefaultsQoS(),
      std::bind(&QueryInfoNode::DogInfoUpdate, this, std::placeholders::_1),
      sub_options);
    uid_sn_srv_ =
      query_info_node_->create_service<protocol::srv::UidSn>(
      "uid_sn",
      std::bind(
        &QueryInfoNode::UidSn, this, std::placeholders::_1,
        std::placeholders::_2));
    motion_status_sub_ =
      query_info_node_->create_subscription<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&QueryInfoNode::MotionStatus, this, std::placeholders::_1),
      sub_options);
    connect_status_sub_ = query_info_node_->create_subscription<protocol::msg::ConnectorStatus>(
      "connector_state", rclcpp::SystemDefaultsQoS(),
      std::bind(&QueryInfoNode::ConnectStatus, this, std::placeholders::_1),
      sub_options);
    bms_status_sub_ = query_info_node_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&QueryInfoNode::BmsStatus, this, std::placeholders::_1),
      sub_options);
    query_node_ptr_ = std::make_unique<QueryInfo>(query_info_node_);
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
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = query_feedback_callback_group_;
    back_to_end_pub_ = query_info_node_->create_publisher<std_msgs::msg::String>(
      "cyberdog/base_info/submit",
      rclcpp::SystemDefaultsQoS(),
      pub_options
    );
    dev_name_set_client_ =
      query_info_node_->create_client<protocol::srv::AudioNickName>(
      "set_nick_name",
      rmw_qos_profile_services_default, query_feedback_callback_group_);
    aiconnect_state_sub_ =
      query_info_node_->create_subscription<std_msgs::msg::Bool>(
      "ai_connect_state", rclcpp::SystemDefaultsQoS(),
      std::bind(&QueryInfoNode::AiStateCallback, this, std::placeholders::_1));
  }

  void Init()
  {
    std::thread(
      [this]() {
        while (rclcpp::ok()) {
          if (!is_reporting_) {
            continue;
          }
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
          bool is_lowpower_control = true;
          bool is_charging = true;
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
          req->enables[13] = is_lowpower_control;
          req->enables[14] = is_charging;
          std_msgs::msg::String msg;
          msg.data = query_node_ptr_->QueryDeviceInfo(req->enables);
          INFO_ONCE("To back end server upload info:%s", msg.data.c_str());
          back_to_end_pub_->publish(msg);
          INFO("[stop:back to end upload device info-------------------]");
          std::this_thread::sleep_for(std::chrono::milliseconds(30000));
        }
      }).detach();
  }

  void Report(bool report)
  {
    is_reporting_ = report;
  }

private:
  void QueryDeviceInfo(
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
    response->info = query_node_ptr_->QueryDeviceInfo(request->enables, request->uid);
  }
  void UidCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    dog_info_notify_ = false;
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
    t.join();
  }
  void DogInfoUpdate(const std_msgs::msg::Bool::SharedPtr msg)
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
  void UidSn(
    const protocol::srv::UidSn::Request::SharedPtr request,
    protocol::srv::UidSn::Response::SharedPtr response)
  {
    (void) request;
    response->sn = query_node_ptr_->GetSn();
    response->uid = uid_;
  }

  void MotionStatus(const protocol::msg::MotionStatus::SharedPtr msg)
  {
    // get standing state
    int motion_state = msg->motion_id;
    if (motion_state == protocol::msg::MotionID::RECOVERYSTAND) {
      query_node_ptr_->GetStand() = true;
    } else {
      query_node_ptr_->GetStand() = false;
    }

    // get non-standing state
    if (motion_state == 0 || motion_state == 101 || motion_state == 102) {
      query_node_ptr_->GetNonStand() = true;
    } else {
      query_node_ptr_->GetNonStand() = false;
    }
  }

  void ConnectStatus(const protocol::msg::ConnectorStatus::SharedPtr msg)
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

  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    query_node_ptr_->GetBms() = *msg;
  }

  void AiStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    INFO("ai connect state:%s", msg->data ? "true" : "false");
    if (msg->data) {
      if (!dog_info_notify_) {
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/manager/settings.json");
        Document json_document(kObjectType);
        auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
        rapidjson::Value dog_val(rapidjson::kObjectType);
        if (!result) {
          dog_info_notify_ = true;
        } else {
          result = CyberdogJson::Get(json_document, "dog_info", dog_val);
          if (!result) {
            dog_info_notify_ = true;
          } else {
            if (!dog_val.HasMember("activate_date")) {
              dog_info_notify_ = true;
            }
          }
        }
      }
      if (dog_info_notify_) {
        if (!dev_name_set_client_->wait_for_service(std::chrono::seconds(3))) {
          INFO(
            "call setnickname server not avaiable"
          );
          return;
        }
        std::chrono::seconds timeout(3);
        auto req = std::make_shared<protocol::srv::AudioNickName::Request>();
        req->nick_name = "铁蛋";
        req->wake_name = "铁蛋铁蛋";
        auto future_result = dev_name_set_client_->async_send_request(req);
        std::future_status status = future_result.wait_for(timeout);
        if (status == std::future_status::ready) {
          INFO(
            "success to call setnickname request services.");
          dog_info_notify_ = false;
        } else {
          INFO(
            "Failed to call setnickname request  services.");
        }
      }
    }
  }

private:
  std::string uid_;
  rclcpp::Node::SharedPtr query_info_node_ {nullptr};
  std::unique_ptr<QueryInfo> query_node_ptr_ {nullptr};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr back_to_end_pub_;
  rclcpp::CallbackGroup::SharedPtr query_srv_callback_group_;
  rclcpp::Service<protocol::srv::DeviceInfo>::SharedPtr device_info_get_srv_;
  rclcpp::CallbackGroup::SharedPtr query_feedback_callback_group_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr uid_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dog_info_update_sub_;
  rclcpp::Service<protocol::srv::UidSn>::SharedPtr uid_sn_srv_;
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_;
  rclcpp::Subscription<protocol::msg::ConnectorStatus>::SharedPtr connect_status_sub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr aiconnect_state_sub_;
  rclcpp::Client<protocol::srv::AudioNickName>::SharedPtr dev_name_set_client_;
  bool is_reporting_ {false};
  bool dog_info_notify_ {false};
};

}  // namespace manager
}  // namespace cyberdog
#endif  // CYBERDOG_MANAGER__QUERY_INFO_HPP_
