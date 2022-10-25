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
#ifndef CYBERDOG_MANAGER__ERROR_CONTEXT_HPP_
#define CYBERDOG_MANAGER__ERROR_CONTEXT_HPP_

#include <string>
#include <queue>
#include <condition_variable>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/msg/node_error.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{
struct ErrorRecorder
{
  std::string name;
  int32_t code;
  int32_t counter;
  explicit ErrorRecorder(std::string nm = "cyberdog_manager")
  : name(nm),
    code(0), counter(0)
  {
  }
};    // struct ErrorRecorder
class ErrorContext final
{
private:
  /* data */

public:
  explicit ErrorContext(std::string node_name)
  : name_(node_name), exit_(false)
  {
    error_report_node_ = rclcpp::Node::make_shared(name_);
    error_report_callback_group_ =
      error_report_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = error_report_callback_group_;
    node_error_sub_ =
      error_report_node_->create_subscription<protocol::msg::NodeError>(
      "node_errors", rclcpp::SystemDefaultsQoS(),
      std::bind(
        &ErrorContext::ErrorsCallback, this,
        std::placeholders::_1),
      sub_options);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = error_report_callback_group_;
    error_pub_ = error_report_node_->create_publisher<std_msgs::msg::String>(
      "manager_errors",
      rclcpp::SystemDefaultsQoS(),
      pub_options
    );
    std::thread(
      [this]() {
        rclcpp::spin(error_report_node_);
      }).detach();
  }
  ~ErrorContext()
  {
    exit_ = true;
    {
      std::lock_guard<std::mutex> lk(er_mut_);
      er_cond_.notify_one();
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  void Init()
  {
    INFO("arror context thread started.");
    thread_ = std::thread(
      [this]() {
        while (rclcpp::ok() && !exit_) {
          std::unique_lock<std::mutex> lck(er_mut_);
          er_cond_.wait(lck, [&] {return !er_vec_.empty() || exit_;});
          if (exit_) {
            break;
          }
          ErrorRecorder er = er_vec_.front();
          er_vec_.pop();
          --er.counter;
          if (er.counter < 0) {
            er.counter = 0;
            er_vec_.push(er);
          } else if (er.counter > 0) {
            er_vec_.push(er);
          }
          lck.unlock();
          Document json_info(kObjectType);
          CyberdogJson::Add(json_info, "name", er.name);
          CyberdogJson::Add(json_info, "code", er.code);
          std::string info;
          if (!CyberdogJson::Document2String(json_info, info)) {
            ERROR("arror context error while encoding to json");
            continue;
          }
          std_msgs::msg::String msg;
          msg.data = info;
          error_pub_->publish(msg);
        }
      });
  }
  void NotifyError(ErrorRecorder er)
  {
    std::lock_guard<std::mutex> lk(er_mut_);
    er_vec_.push(er);
    er_cond_.notify_one();
  }
  void ClearError()
  {
    std::lock_guard<std::mutex> lk(er_mut_);
    while (!er_vec_.empty()) {
      er_vec_.pop();
    }
  }

private:
  void ErrorsCallback(const protocol::msg::NodeError::SharedPtr msg)
  {
    ErrorRecorder er;
    er.name = msg->name;
    er.code = msg->code;
    er.counter = 1;
    NotifyError(er);
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr error_report_node_;
  rclcpp::CallbackGroup::SharedPtr error_report_callback_group_;
  bool exit_;
  std::thread thread_;
  rclcpp::Subscription<protocol::msg::NodeError>::SharedPtr node_error_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_pub_;
  std::queue<ErrorRecorder> er_vec_;
  std::mutex er_mut_;
  std::condition_variable er_cond_;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__ERROR_CONTEXT_HPP_
