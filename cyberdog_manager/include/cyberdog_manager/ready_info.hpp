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
#ifndef CYBERDOG_MANAGER__READY_INFO_HPP_
#define CYBERDOG_MANAGER__READY_INFO_HPP_

#include <string>
#include <map>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/msg/self_check_status.hpp"
#include "protocol/msg/state_switch_status.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/srv/audio_text_play.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/srv/bes_http_send_file.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_common/cyberdog_toml.hpp"


using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::Value;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{
enum SelfcheckState
{
  UNKNOW = -1,
  ALL_SUCCESS = 0,
  HARDWARE_CHECKING = 1,
  CRITICAL_HARDWARE_FAILED = 2,
  HARDWARE_SUCCESS = 3,
  UNCRITICAL_HARDWARE_FAILED = 4,
  SOFTWARE_SETUP_FAILED = 5,
  SOFTWARE_SETUP_SUCCESS = 6
};
enum ErrorCode
{
  kLedSelfcheckError = 1109,
  kBmsSelfcheckError = 1409,
  kTouchSelfcheckError = 1509,
  kUWBSelfcheckError = 1609,
  kLidarSelfcheckError = 2109,
  kTofSelfcheckError = 2209,
  // kUltrasnoicSelfcheckError = 2309,
  // kGpsSelfcheckError = 2409,

  kLedAndUWBSelfcheckError = 1109 + 1609,
  kTofAndLidarSelfcheckError = 2109 + 2209
};
class ReadyNotifyNode final
{
  using EXCEPTION_PLAYSOUND_CALLBACK = std::function<void (int32_t )>;

public:
  explicit ReadyNotifyNode(rclcpp::Node::SharedPtr node_ptr)
  {
    ready_notify_node_ = node_ptr;
    ready_notify_callback_group_ =
      ready_notify_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = ready_notify_callback_group_;
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = ready_notify_callback_group_;
    ready_notify_pub_ = ready_notify_node_->create_publisher<std_msgs::msg::Bool>(
      "ready_notify",
      rclcpp::SystemDefaultsQoS(),
      pub_options
    );
    self_check_status_pub_ = ready_notify_node_->create_publisher<protocol::msg::SelfCheckStatus>(
      "self_check_status",
      rclcpp::SystemDefaultsQoS(),
      pub_options
    );
    state_swith_status_pub_ =
      ready_notify_node_->create_publisher<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), pub_options);

    app_connect_state_sub_ = ready_notify_node_->create_subscription<std_msgs::msg::Bool>(
      "app_connection_state", rclcpp::SystemDefaultsQoS(),
      std::bind(&ReadyNotifyNode::AppConnectState, this, std::placeholders::_1),
      sub_options);

    motion_excute_client_ =
      ready_notify_node_->create_client<protocol::srv::MotionResultCmd>(
      "motion_result_cmd",
      rmw_qos_profile_services_default, ready_notify_callback_group_);

    audio_play_client_ =
      ready_notify_node_->create_client<protocol::srv::AudioTextPlay>(
      "speech_text_play",
      rmw_qos_profile_services_default, ready_notify_callback_group_);

    bms_status_sub_ =
      ready_notify_node_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&ReadyNotifyNode::BmsStatus, this, std::placeholders::_1), sub_options);

    upload_events_client_ = ready_notify_node_->create_client<protocol::srv::BesHttpSendFile>(
      "bes_http_send_file_srv", rmw_qos_profile_services_default);
    // std::thread(
    //   [this]() {
    //     executor_.add_node(train_plan_node_ptr_);
    //     executor_.spin();
    //      rclcpp::shutdown();
    //   }).detach();
    GetCriticalMoudle();
  }

  void Ready(bool ready)
  {
    count_ = 3000;
    ready_ = ready;
    if (!notify_message_thread.joinable()) {
      notify_message_thread = std::thread(
        [this]() {
          while (!exit_ && count_ > 0 && rclcpp::ok()) {
            std_msgs::msg::Bool msg;
            msg.data = ready_;
            ready_notify_pub_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            --count_;
          }
        });
    }
  }

  void SelfCheck(int32_t state, const std::map<std::string, int32_t> & stmap)
  {
    selfcheck_state_ = state;
    INFO("current state is %d", selfcheck_state_);
    if (!notify_selfcheck_thread.joinable()) {
      notify_selfcheck_thread = std::thread(
        [this, &stmap]() {
          static int32_t scs = -1;
          while (!exit_ && rclcpp::ok()) {
            protocol::msg::SelfCheckStatus msg;
            if (0 == selfcheck_state_ || 6 == selfcheck_state_) {
              static std::string describ = UpSelfCheckState(selfcheck_state_, stmap);
              msg.description = describ;
            } else {
              msg.description = UpSelfCheckState(selfcheck_state_, stmap);
            }
            msg.code = selfcheck_state_;
            self_check_status_pub_->publish(msg);
            INFO_EXPRESSION(
              (selfcheck_state_ != scs), "self check status:%s",
              msg.description.c_str());
            scs = selfcheck_state_;
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          }
        });
    }
  }

  bool UploadEvents()
  {
    if (!upload_events_client_->wait_for_service(std::chrono::seconds(3))) {
      WARN("call upload events service not avalible!");
      return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(2));
    int64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    std::string events = "{" + std::string("\"timestamp\": ") +
      std::to_string(now_ms) + ", \"info\": " + state_str_ + "}";
    INFO("upload events which selfcheck fialed is %s", events.c_str());
    auto req = std::make_shared<protocol::srv::BesHttpSendFile::Request>();
    req->method = protocol::srv::BesHttpSendFile::Request::HTTP_METHOD_POST;
    req->url = "device/system/log";
    req->info = events;
    req->milsecs = 60000;  // 60s
    auto future_result = upload_events_client_->async_send_request(req);
    std::future_status status = future_result.wait_for(std::chrono::seconds(60));
    if (status == std::future_status::ready) {
      auto response = future_result.get()->data;
      INFO("upload log respon: %s", response.c_str());
    } else {
      WARN("calling bes_http_send_file_srv service timeout!");
      return false;
    }
    return true;
  }

  void SetExceptionPlaySoundCallback(EXCEPTION_PLAYSOUND_CALLBACK callback)
  {
    play_sound = callback;
  }

  void GetCriticalMoudle()
  {
    std::vector<std::string> sensor_vec_;
    std::vector<std::string> device_vec_;
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/selfcheck_config.toml");
    toml::value config;
    if (common::CyberdogToml::ParseFile(path, config)) {
      INFO("Parse Selfcheck config file started, toml file is valid");
      toml::value sensors_sec;
      if (common::CyberdogToml::Get(config, "sensors", sensors_sec)) {
        INFO("Selfheck init started, parse sensors config succeed");
        if (common::CyberdogToml::Get(sensors_sec, "critical", critical_sensor_)) {
          INFO("Self Check init started, parse sensors jump array succeed");
        }
      }
      toml::value device_sec;
      if (common::CyberdogToml::Get(config, "devices", device_sec)) {
        INFO("Selfcheck init started, parse devices config succeed");
        if (common::CyberdogToml::Get(device_sec, "critical", critical_device_)) {
          INFO("Selfcheck init started, parse device jump array succeed");
        }
      }
    }
  }

  bool IsSelfcheckError(const std::map<std::string, int32_t> & stmap)
  {
    for (const auto & [moudle, code] : stmap) {
      (void)moudle;
      if (code != 0) {
        return true;
      }
    }

    return false;
  }

  bool IsCriticalError(const std::map<std::string, int32_t> & stmap)
  {
    for (const auto & [module, code] : stmap) {
      INFO("[moudle,code]:[%s, %d]", module.c_str(), code);
      if (0 == code) {
        continue;
      }

      if ("motion_manager" == module || "audio_manager" == module) {
        play_sound(code);
        return true;
      }

      if ("device_manager" == module) {
        std::vector<int> error_code;
        for (auto n : critical_device_) {
          auto itr = module_map_.find(n);
          if (itr != module_map_.end()) {
            error_code.emplace_back(static_cast<int>(itr->second));
          }
          INFO("critical devices array is %s", n.c_str());
        }

        if (std::find(error_code.begin(), error_code.end(), code) != error_code.end()) {
          play_sound(code);
          return true;
        }
      }

      if ("sensor_manager" == module) {
        std::vector<int> error_code;
        for (auto n : critical_sensor_) {
          auto itr = module_map_.find(n);
          if (itr != module_map_.end()) {
            error_code.emplace_back(static_cast<int>(itr->second));
          }
          INFO("critical sensors array is %s", n.c_str());
        }

        if (std::find(error_code.begin(), error_code.end(), code) == error_code.end()) {
          play_sound(code);
          return true;
        }
      }
    }
    return false;
  }


  void StoreSelfcheckResults()
  {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    auto path = SELFCHECK_DIR + "/" + FILE_NAME;
    if (access(SELFCHECK_DIR.c_str(), F_OK) != 0) {
      std::string cmd = "mkdir -p " + SELFCHECK_DIR;
      std::system(cmd.c_str());
      cmd = "chmod 777 " + SELFCHECK_DIR;
      std::system(cmd.c_str());
    }
    CyberdogJson::WriteJsonToFile(path, selfcheck_info_);
  }

  std::string UpSelfCheckState(const int32_t state, const std::map<std::string, int32_t> & stmap)
  {
    int32_t value {0};
    Document self_check_state(kObjectType);
    Document device(kObjectType);
    Document sensor(kObjectType);
    Document::AllocatorType & allocator = self_check_state.GetAllocator();
    if (1 == state) {
      value = -1;
    }
    CyberdogJson::Add(device, "bms", value);
    CyberdogJson::Add(device, "led", value);
    CyberdogJson::Add(device, "touch", value);
    CyberdogJson::Add(device, "uwb", value);
    CyberdogJson::Add(device, "bluetooth", value);
    CyberdogJson::Add(device, "others", value);
    CyberdogJson::Add(sensor, "lidar", value);
    // CyberdogJson::Add(sensor, "gps", value);
    // CyberdogJson::Add(sensor, "ultrasonic", value);
    CyberdogJson::Add(sensor, "tof", value);
    CyberdogJson::Add(sensor, "others", value);
    CyberdogJson::Add(self_check_state, "audio", value);
    CyberdogJson::Add(self_check_state, "motion_manager", value);
    if (3 == state) {
      value = -1;
    }
    CyberdogJson::Add(self_check_state, "algorithm_manager", value);
    CyberdogJson::Add(self_check_state, "vp_engine", value);
    CyberdogJson::Add(self_check_state, "RealSenseActuator", value);
    if (2 == state || 4 == state || 6 == state) {
      for (const auto & [moudle, code] : stmap) {
        if (moudle == "algorithm_manager") {
          Value & tmp = self_check_state["algorithm_manager"];
          tmp.SetInt(code);
          continue;
        }
        if (moudle == "device_manager") {
          switch (static_cast<ErrorCode>(code)) {
            case kLedSelfcheckError: {
                Value & tmp = device["led"];
                tmp.SetInt(static_cast<int>(kLedSelfcheckError));
                break;
              }
            case kBmsSelfcheckError: {
                Value & tmp = device["bms"];
                tmp.SetInt(static_cast<int>(kBmsSelfcheckError));
                break;
              }
            case kTouchSelfcheckError: {
                Value & tmp = device["touch"];
                tmp.SetInt(static_cast<int>(kTouchSelfcheckError));
                break;
              }
            case kUWBSelfcheckError: {
                Value & tmp = device["uwb"];
                tmp.SetInt(static_cast<int>(kUWBSelfcheckError));
                break;
              }
            case kLedAndUWBSelfcheckError: {
                Value & tmp = device["led"];
                tmp.SetInt(static_cast<int>(kLedSelfcheckError));
                Value & tmp2 = device["uwb"];
                tmp2.SetInt(static_cast<int>(kUWBSelfcheckError));
                break;
              }
            case kTofAndLidarSelfcheckError:
            case kTofSelfcheckError:
            case kLidarSelfcheckError:
            default: {
                Value & tmp = device["others"];
                tmp.SetInt(static_cast<int>(code));
                break;
              }
          }
          continue;
        }
        if (moudle == "motion_manager") {
          Value & tmp = self_check_state["motion_manager"];
          tmp.SetInt(code);
          continue;
        }
        if (moudle == "sensor_manager") {
          switch (static_cast<ErrorCode>(code)) {
            case kLidarSelfcheckError: {
                Value & tmp = sensor["lidar"];
                tmp.SetInt(static_cast<int>(kLidarSelfcheckError));
                break;
              }
            case kTofSelfcheckError: {
                Value & tmp = sensor["tof"];
                tmp.SetInt(static_cast<int>(kTofSelfcheckError));
                break;
              }
            case kTofAndLidarSelfcheckError: {
                Value & tmp = sensor["lidar"];
                tmp.SetInt(static_cast<int>(kLidarSelfcheckError));
                Value & tmp2 = sensor["tof"];
                tmp2.SetInt(static_cast<int>(kTofSelfcheckError));
                break;
              }
            case kLedSelfcheckError:
            case kBmsSelfcheckError:
            case kTouchSelfcheckError:
            case kUWBSelfcheckError:
            case kLedAndUWBSelfcheckError:
            default: {
                Value & tmp = sensor["others"];
                tmp.SetInt(static_cast<int>(code));
                break;
              }
          }
          continue;
        }
        if (moudle == "vp_engine") {
          Value & tmp = self_check_state["vp_engine"];
          tmp.SetInt(code);
          continue;
        }
        if (moudle == "RealSenseActuator") {
          Value & tmp = self_check_state["RealSenseActuator"];
          tmp.SetInt(code);
          continue;
        }
      }
    }
    self_check_state.AddMember("device", device, allocator);
    self_check_state.AddMember("sensor", sensor, allocator);
    CyberdogJson::Document2String(self_check_state, state_str_);
    selfcheck_info_.CopyFrom(self_check_state, allocator);
    return state_str_;
  }

  void MachineState(int32_t state)
  {
    protocol::msg::StateSwitchStatus sss;
    sss.code = 0;
    sss.state = state;
    state_swith_status_pub_->publish(sss);
  }

  void AppConnectState(const std_msgs::msg::Bool msg)
  {
    static bool pre_connect_state{false};
    if (selfcheck_state_ == -1 || selfcheck_state_ == 1 || selfcheck_state_ == 2) {
      return;
    }
    // app第一次连接后站立
    if (pre_connect_state == false && msg.data == true) {
      pre_connect_state = true;
      if (is_charging) {
        return;
      }
      cyberdog_standing_ = 1;
      INFO("app connected, dog standup");
      PlayAudioService(2002);
      cyberdog_standing_ = 2;
      if (!motion_excute_client_->wait_for_service(std::chrono::seconds(5))) {
        ERROR("call motion server not avalible");
      }
      auto request_motion = std::make_shared<protocol::srv::MotionResultCmd::Request>();
      request_motion->motion_id = 111;
      request_motion->cmd_source = 0;
      auto future_result_motion = motion_excute_client_->async_send_request(request_motion);
      std::future_status status_motion = future_result_motion.wait_for(std::chrono::seconds(5));
      if (status_motion != std::future_status::ready) {
        ERROR("call motion service failed");
      }
      if (future_result_motion.get()->result != 0) {
        ERROR("control motion fialed, error code is:%d", future_result_motion.get()->code);
      }
      pre_connect_state = true;
    }
    // app重连后站立
    // if (pre_connect_state == true && msg.data == false) {
    //   pre_connect_state = false;
    // }
  }

  void PlayAudioService(const uint16_t play_id)
  {
    auto request_audio = std::make_shared<protocol::srv::AudioTextPlay::Request>();
    request_audio->module_name = ready_notify_node_->get_name();
    request_audio->is_online = false;
    request_audio->speech.play_id = play_id;
    auto callback = [](rclcpp::Client<protocol::srv::AudioTextPlay>::SharedFuture future) {
        INFO("Audio play result: %s", future.get()->status == 0 ? "success" : "failed");
      };
    auto future_result_audio = audio_play_client_->async_send_request(request_audio, callback);
    if (future_result_audio.wait_for(std::chrono::milliseconds(3000)) ==
      std::future_status::timeout)
    {
      ERROR("Cannot get response from AudioPlay");
    }
  }

  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    is_charging = msg->power_wired_charging;
  }

  int GetAudioOccupancyState()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    return cyberdog_standing_;
  }

  ~ReadyNotifyNode()
  {
    exit_ = true;
    if (notify_message_thread.joinable()) {
      notify_message_thread.join();
    }
  }

private:
  bool exit_ {false};
  bool ready_ {false};
  bool is_charging {false};
  int32_t selfcheck_state_ {-1};
  std::string name_;
  rclcpp::Node::SharedPtr ready_notify_node_ {nullptr};
  // rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::CallbackGroup::SharedPtr ready_notify_callback_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_notify_pub_;
  rclcpp::Publisher<protocol::msg::SelfCheckStatus>::SharedPtr self_check_status_pub_;
  rclcpp::Publisher<protocol::msg::StateSwitchStatus>::SharedPtr state_swith_status_pub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr app_connect_state_sub_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_excute_client_;
  rclcpp::Client<protocol::srv::AudioTextPlay>::SharedPtr audio_play_client_ {nullptr};
  rclcpp::Client<protocol::srv::BesHttpSendFile>::SharedPtr upload_events_client_ {nullptr};
  std::thread notify_message_thread;
  std::thread notify_selfcheck_thread;
  int count_;
  int cyberdog_standing_ {0};
  std::string state_str_ {};
  EXCEPTION_PLAYSOUND_CALLBACK play_sound {[](int32_t) {}};
  rapidjson::Document selfcheck_info_;
  const std::string SELFCHECK_DIR {"/home/mi/.ros/log"};
  const std::string FILE_NAME {"last_selfcheck_info.json"};
  std::vector<std::string> critical_sensor_;
  std::vector<std::string> critical_device_;
  const std::map<std::string, ErrorCode> module_map_ {
    {"led", kLedSelfcheckError},
    {"bms", kBmsSelfcheckError},
    {"touch", kTouchSelfcheckError},
    {"lidar", kLidarSelfcheckError},
    {"tof", kBmsSelfcheckError},
    // {"Ultrasnoic", kUltrasnoicSelfcheckError},
    // {"Gps", kGpsSelfcheckError},
  };
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__READY_INFO_HPP_
