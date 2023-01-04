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
#ifndef CYBERDOG_MANAGER__READY_INFO_HPP_
#define CYBERDOG_MANAGER__READY_INFO_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/msg/self_check_status.hpp"

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

  void SelfCheck(bool is_ok)
  {
    is_ok_ = is_ok;
    if (!notify_selfcheck_thread.joinable()) {
      notify_selfcheck_thread = std::thread(
        [this]() {
          while (!exit_ && rclcpp::ok()) {
            protocol::msg::SelfCheckStatus msg;
            if (is_ok_) {
              msg.code = 0;
              msg.description = "ok";
            } else {
              msg.code = -1;
              msg.description = "failed";
            }
            self_check_status_pub_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            --count_;
          }
        });
    }
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
  bool is_ok_ {false};
  std::string name_;
  rclcpp::Node::SharedPtr ready_notify_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr ready_notify_callback_group_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_notify_pub_;
  rclcpp::Publisher<protocol::msg::SelfCheckStatus>::SharedPtr self_check_status_pub_;
  std::thread notify_message_thread;
  std::thread notify_selfcheck_thread;
  int count_;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__READY_INFO_HPP_
