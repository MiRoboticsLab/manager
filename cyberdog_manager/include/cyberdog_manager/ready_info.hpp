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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/msg/self_check_status.hpp"
#include "protocol/msg/state_switch_status.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::Value;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{
class ReadyNotifyNode final
{
public:
  explicit ReadyNotifyNode(const std::string & node_name)
  : name_(node_name)
  {
    ready_notify_node_ = rclcpp::Node::make_shared(name_);
    ready_notify_callback_group_ =
      ready_notify_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions options;
    options.callback_group = ready_notify_callback_group_;
    ready_notify_pub_ = ready_notify_node_->create_publisher<std_msgs::msg::Bool>(
      "ready_notify",
      rclcpp::SystemDefaultsQoS(),
      options
    );
    self_check_status_pub_ = ready_notify_node_->create_publisher<protocol::msg::SelfCheckStatus>(
      "self_check_status",
      rclcpp::SystemDefaultsQoS(),
      options
    );
    state_swith_status_pub_ =
      ready_notify_node_->create_publisher<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), options);
    std::thread(
      [this]() {
        rclcpp::spin(ready_notify_node_);
      }).detach();
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
    if (!notify_selfcheck_thread.joinable()) {
      notify_selfcheck_thread = std::thread(
        [this, &stmap]() {
          static int32_t scs = -1;
          while (!exit_ && rclcpp::ok()) {
            protocol::msg::SelfCheckStatus msg;
            if (0 == selfcheck_state_) {
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

  std::string UpSelfCheckState(const int32_t state, const std::map<std::string, int32_t> & stmap)
  {
    int32_t value {0};
    std::string state_str {};
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
    CyberdogJson::Add(sensor, "lidar", value);
    CyberdogJson::Add(sensor, "gps", value);
    CyberdogJson::Add(sensor, "ultrasonic", value);
    CyberdogJson::Add(sensor, "tof", value);
    CyberdogJson::Add(self_check_state, "audio", value);
    CyberdogJson::Add(self_check_state, "motion_manager", value);
    CyberdogJson::Add(self_check_state, "algorithm_manager", value);
    CyberdogJson::Add(self_check_state, "vp_engine", value);
    CyberdogJson::Add(self_check_state, "RealSenseActuator", value);

    if (2 == state) {
      for (const auto & n : stmap) {
        if (n.first == "algorithm_manager") {
          Value & tmp = self_check_state["algorithm_manager"];
          tmp.SetInt(n.second);
          continue;
        }
        if (n.first == "device_manager") {
          Value & tmp = device["bms"];
          tmp.SetInt(n.second);
          Value & tmp2 = device["led"];
          tmp2.SetInt(n.second);
          Value & tmp3 = device["touch"];
          tmp3.SetInt(n.second);
          Value & tmp4 = device["uwb"];
          tmp4.SetInt(n.second);
          continue;
        }
        if (n.first == "motion_manager") {
          Value & tmp = self_check_state["motion_manager"];
          tmp.SetInt(n.second);
          continue;
        }
        if (n.first == "sensor_manager") {
          Value & tmp = sensor["lidar"];
          tmp.SetInt(n.second);
          Value & tmp2 = sensor["gps"];
          tmp2.SetInt(n.second);
          Value & tmp3 = sensor["ultrasonic"];
          tmp3.SetInt(n.second);
          Value & tmp4 = sensor["tof"];
          tmp4.SetInt(n.second);
          continue;
        }
        if (n.first == "vp_engine") {
          Value & tmp = self_check_state["vp_engine"];
          tmp.SetInt(n.second);
          continue;
        }
        if (n.first == "RealSenseActuator") {
          Value & tmp = self_check_state["RealSenseActuator"];
          tmp.SetInt(n.second);
          continue;
        }
      }
    }
    self_check_state.AddMember("device", device, allocator);
    self_check_state.AddMember("sensor", sensor, allocator);
    CyberdogJson::Document2String(self_check_state, state_str);
    return state_str;
  }

  void MachineState(int32_t state)
  {
    protocol::msg::StateSwitchStatus sss;
    sss.code = 0;
    sss.state = state;
    state_swith_status_pub_->publish(sss);
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
  int32_t selfcheck_state_ {-1};
  std::string name_;
  rclcpp::Node::SharedPtr ready_notify_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr ready_notify_callback_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_notify_pub_;
  rclcpp::Publisher<protocol::msg::SelfCheckStatus>::SharedPtr self_check_status_pub_;
  rclcpp::Publisher<protocol::msg::StateSwitchStatus>::SharedPtr state_swith_status_pub_;
  std::thread notify_message_thread;
  std::thread notify_selfcheck_thread;
  int count_;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__READY_INFO_HPP_
