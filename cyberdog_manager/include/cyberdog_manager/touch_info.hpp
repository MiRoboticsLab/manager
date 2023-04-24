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
#ifndef CYBERDOG_MANAGER__TOUCH_INFO_HPP_
#define CYBERDOG_MANAGER__TOUCH_INFO_HPP_

#include <chrono>
#include <cmath>
#include <mutex>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "std_msgs/msg/string.hpp"
#include "protocol/msg/touch_status.hpp"
#include "protocol/msg/bms_status.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/msg/state_switch_status.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace cyberdog
{
namespace manager
{
class TouchInfoNode final
{
public:
  explicit TouchInfoNode(rclcpp::Node::SharedPtr node_ptr)
  {
    touch_info_node_ = node_ptr;
    touch_callback_group_ =
      touch_info_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;

    sub_options.callback_group = touch_callback_group_;
    touch_status_sub_ = touch_info_node_->create_subscription<protocol::msg::TouchStatus>(
      "touch_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&TouchInfoNode::ReportPower, this, std::placeholders::_1), sub_options);

    bms_status_sub_ = touch_info_node_->create_subscription<protocol::msg::BmsStatus>(
      "bms_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&TouchInfoNode::BmsStatus, this, std::placeholders::_1), sub_options);

    StateSwitch_sub = touch_info_node_->create_subscription<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(),
      std::bind(&TouchInfoNode::SelfCheckCallback, this, std::placeholders::_1), sub_options);

    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = touch_callback_group_;
    audio_play_extend_pub =
      touch_info_node_->create_publisher<protocol::msg::AudioPlayExtend>(
      "speech_play_extend", rclcpp::SystemDefaultsQoS(), pub_options);
    low_power_exit_client_ = touch_info_node_->create_client<std_srvs::srv::Trigger>(
      "low_power_exit");
    std::thread ExitLowPowerThread = std::thread(&TouchInfoNode::ExitLowPower, this);
    ExitLowPowerThread.detach();
  }

private:
  void ReportPower(const protocol::msg::TouchStatus::SharedPtr msg)
  {
    INFO_MILLSECONDS(30000, "[cyberdog_manager:] touch_info-->> Enter TouchStatus callback!!");
    std::chrono::milliseconds report_previous_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
    INFO("report_previous_time is %d", report_previous_time.count());
    if (msg->touch_state == 3 && report_flag &&
      abs(reporting_time.count() - report_previous_time.count()) >= 2000)
    {
      reporting_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
      protocol::msg::AudioPlayExtend msg;
      msg.is_online = false;
      msg.module_name = touch_info_node_->get_name();
      msg.speech.play_id = 1000+battery_percent;
      // msg.text = "剩余电量为百分之" + std::to_string(battery_percent);
      // INFO("语言播报的电量为：%s", msg.text.c_str());
      audio_play_extend_pub->publish(msg);
      if (is_low_power) {  // 通知消费者
        lowPower_cv_.notify_all();
      }
    }
  }

  void BmsStatus(const protocol::msg::BmsStatus::SharedPtr msg)
  {
    battery_percent = msg->batt_soc;
    report_flag = true;
    INFO_MILLSECONDS(
      30000,
      "[cyberdog_manager:] touch_info-->> Enter BMS callback.the batt_soc is %d.",
      battery_percent);
  }
  void SelfCheckCallback(const protocol::msg::StateSwitchStatus::SharedPtr msg)
  {
    if (msg->state == 2) {
      is_low_power = true;
      INFO("state_switch_status state is %d", msg->state);
    } else {
      is_low_power = false;
    }
  }
  void ExitLowPower()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lck(exitLowPower_mtx_);
      while (!is_low_power) {  // 消费者得到通知，跳出循环执行退出低功耗指令
        lowPower_cv_.wait(lck);
      }
      // 退出低功耗请求
      if (!low_power_exit_client_->wait_for_service(std::chrono::seconds(3))) {
        INFO("low_power_exit service is not avaiable");
      }
      std::chrono::seconds timeout(13);  // wait for completely exiting
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future_result = low_power_exit_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO("call low_power_exit success.");
        is_low_power = false;
      } else {
        ERROR("call low_power_exit timeout.");
      }
    }
  }

private:
  rclcpp::Node::SharedPtr touch_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr touch_callback_group_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr low_power_exit_client_;
  rclcpp::Subscription<protocol::msg::TouchStatus>::SharedPtr touch_status_sub_;
  rclcpp::Subscription<protocol::msg::BmsStatus>::SharedPtr bms_status_sub_;
  rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_extend_pub;
  int battery_percent;
  std::chrono::milliseconds reporting_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch());
  bool report_flag = false;

  rclcpp::Subscription<protocol::msg::StateSwitchStatus>::SharedPtr StateSwitch_sub;
  // rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  bool low_power_status = false;
  std::mutex exitLowPower_mtx_;
  bool is_low_power{false};
  std::condition_variable lowPower_cv_;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__TOUCH_INFO_HPP_
