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
#ifndef CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
#define CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_

#include <string>
#include <memory>
#include <chrono>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/led_execute.hpp"
#include "protocol/msg/motion_status.hpp"
#include "low_power_consumption/low_power_consumption.hpp"
#include "protocol/msg/state_switch_status.hpp"
#include "protocol/msg/audio_play_extend.hpp"
#include "protocol/srv/motion_result_cmd.hpp"

namespace cyberdog
{
namespace manager
{
enum class PowerMachineState : uint8_t
{
  PMS_NORMAL    = 0,    // 正常
  PMS_PROTECT   = 1,    // 保护
  PMS_LOWPOWER  = 2,    // 低功耗
  PMS_UNKOWN    = 255,  // 未知
};

class PowerConsumptionInfoNode final
{
  using PCIN_CALLBACK = std::function<void ()>;

public:
  explicit PowerConsumptionInfoNode(rclcpp::Node::SharedPtr node_ptr, PCIN_CALLBACK callback)
  : enter_lowpower_handler(callback)
  {
    power_consumption_info_node_ = node_ptr;
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
    power_consumption_callback_group_ =
      power_consumption_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = power_consumption_callback_group_;
    led_excute_client_ =
      power_consumption_info_node_->create_client<protocol::srv::LedExecute>(
      "led_execute",
      rmw_qos_profile_services_default, power_consumption_callback_group_);
    motion_excute_client_ =
      power_consumption_info_node_->create_client<protocol::srv::MotionResultCmd>(
      "motion_result_cmd",
      rmw_qos_profile_services_default, power_consumption_callback_group_);
    dog_leg_calibration_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::SetBool>(
      "dog_leg_calibration",
      std::bind(
        &PowerConsumptionInfoNode::MotorControl, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);
    audio_play_extend_pub_ =
      power_consumption_info_node_->create_publisher<protocol::msg::AudioPlayExtend>(
      "speech_play_extend",
      rclcpp::SystemDefaultsQoS(), pub_options);

    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    state_swith_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_state_switch_status_callback,
        this, std::placeholders::_1), options);
  }

  void Init()
  {
    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    motion_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::MotionStatus>(
      "motion_status", rclcpp::SystemDefaultsQoS(), std::bind(
        &PowerConsumptionInfoNode::sub_mostion_status_callback,
        this, std::placeholders::_1), options);
    start = std::chrono::steady_clock::now();
  }

  bool EnterLowPower(bool is_enter)
  {
    static int r_count = 0;
    PM_DEV pd = PM_ALL_NO_TOF;
    PM_DEV turn_on_led = PM_TOF;
    unsigned int err;
    int code = -1;
    bool result = false;
    if (is_enter) {
      INFO("[LowPower]: [%d]LowPower enter inner-call:start", (r_count + 1));
      code = lpc_ptr_->LpcRelease(pd, &err);
      if (code == 0) {
        is_lowpower_ = true;
        INFO("[LowPower]: [%d]LowPower enter inner-call:success", (r_count + 1));
        result = true;
      } else {
        INFO(
          "[LowPower]: [%d]LowPower enter inner-call:failed! error code is %d",
          (r_count + 1), code);
      }
      ++r_count;
    } else {
      INFO("[LowPower]: [%d]LowPower exit inner-call:start", (r_count + 1));
      code = lpc_ptr_->LpcRequest(turn_on_led, &err);
      TailLedControl(true, true);
      code = lpc_ptr_->LpcRequest(pd, &err);
      if (code == 0) {
        start = std::chrono::steady_clock::now();
        is_lowpower_ = false;
        INFO("[LowPower]: [%d]LowPower exit inner-call:success", (r_count + 1));
        result = true;
      } else {
        INFO(
          "[LowPower]: [%d]LowPower exit inner-call:failed! error code is %d",
          (r_count + 1), code);
      }
      ++r_count;
    }
    return result;
  }

  void MotorControl(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (!motor_mutex_.try_lock()) {
      WARN(
        "The motor is powering %s and rejecting duplicate requests",
        (request->data ? "up" : "down"));
      return;
    }
    PM_DEV pd = PM_MOTOR;
    int code = -1;
    unsigned int err;
    if (!request->data) {
      // 控制铁蛋高阻尼趴下,电机下电
      PlayAudio("铁蛋即将趴下后进行断电，请注意安全");
      sleep(4);
      MotionContrl(101);
      code = lpc_ptr_->LpcRelease(pd, &err);
      response->success = (code ? false : true);
      INFO("motor shutdown is %s, code is %d", (code ? "failed" : "successed"), code);
      if (code != 0) {
        return;
      }
      PlayAudio("断电成功，请帮我把腿摆放正常后，点击上电");
    } else {
      // 电机上电
      code = lpc_ptr_->LpcRequest(pd, &err);
      response->success = (code ? false : true);
      if (code != 0) {
        return;
      }
      PlayAudio("铁蛋上电成功后将站立，请注意安全");
      sleep(3);
      MotionContrl(111);
      INFO("motor start is %s, code is %d", (code ? "failed" : "successed"), code);
    }
    motor_mutex_.unlock();
  }

  void PlayAudio(const std::string & text)
  {
    protocol::msg::AudioPlayExtend msg;
    msg.is_online = true;
    msg.module_name = power_consumption_info_node_->get_name();
    msg.text = text;
    audio_play_extend_pub_->publish(msg);
  }

  bool MotionContrl(const int motion_id)
  {
    if (!motion_excute_client_->wait_for_service(std::chrono::seconds(5))) {
      ERROR("call led_excute server not avalible");
      return false;
    }
    auto request_motion = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    request_motion->motion_id = motion_id;
    request_motion->cmd_source = 0;
    auto future_result_motion = motion_excute_client_->async_send_request(request_motion);
    std::future_status status_motion = future_result_motion.wait_for(std::chrono::seconds(5));
    if (status_motion == std::future_status::timeout) {
      ERROR("call motion service failed");
    }
    if (future_result_motion.get()->code != 0) {
      ERROR("control motion fialed, error code is:%d", future_result_motion.get()->code);
    }
    return true;
  }

  bool QueryLowPower()
  {
    PM_DEV pd = PM_CAM_REALSNS;
    int code = lpc_ptr_->LpcQuery(pd);
    return (code == 0) ? true : false;
  }

  int ShutdownOrReboot(bool trigger)
  {
    // trigger == true --> reboot, trigger == false --> shutdown
    PM_SYS pd = (trigger ? PM_SYS_REBOOT : PM_SYS_SHUTDOWN);
    INFO("[PowerConsumption]: %s", (trigger ? "reboot..." : "poweroff..."));
    int code = -1;
    code = lpc_ptr_->LpcSysRequest(pd);
    if (code != 0) {
      INFO(
        "[PowerConsumption]: %s failed, function LpcRequest error code is %d",
        (trigger ? "reboot..." : "poweroff..."), code);
    } else {
      INFO("[PowerConsumption]: %s successfully", (trigger ? "reboot..." : "poweroff..."));
    }
    return code;
  }

  void TailLedControl(bool is_light_off, bool is_exiting = false)
  {
    if (!led_excute_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call led_excute server not avalible");
      return;
    }
    auto request_led = std::make_shared<protocol::srv::LedExecute::Request>();
    request_led->occupation = is_light_off;
    request_led->client = "lowpower";
    request_led->target = 2;
    request_led->mode = 0x01;
    request_led->effect = 0xA0;
    if (is_exiting) {
      request_led->mode = 0x02;
      request_led->effect = 0x04;
      request_led->r_value = 0x06;
      request_led->g_value = 0x21;
      request_led->b_value = 0xE2;
    }
    auto future_result_tail = led_excute_client_->async_send_request(request_led);
    std::future_status status_tail = future_result_tail.wait_for(std::chrono::seconds(2));
    if (status_tail != std::future_status::ready) {
      ERROR("call led_execute service failed");
      return;
    }
    if (future_result_tail.get()->code == 0) {
      INFO("call led service successed");
    } else {
      ERROR(
        "control tail led fialed, error code is:%d", future_result_tail.get()->code);
    }
  }

private:
  void sub_mostion_status_callback(const protocol::msg::MotionStatus::SharedPtr msg)
  {
    // 状态机切换也有判断
    if (is_ota_) {
      INFO_MILLSECONDS(10000, "[LowPower]: in ota state return.");
      return;
    }
    // motion_id: 趴下(101)、站立(111)
    int motion_id = msg->motion_id;
    if (is_lowpower_) {
      return;
    }

    if (motion_id == 0 || motion_id == 101) {
      auto end = std::chrono::steady_clock::now();
      auto diff = std::chrono::duration_cast<std::chrono::seconds>(end - start);
      if (diff.count() >= enter_lowpower_time_) {
        // INFO("[LowPower]: enter lowpower, get down for more than 30s");
        start = std::chrono::steady_clock::now();
        enter_lowpower_handler();
        start = std::chrono::steady_clock::now();
      }
    } else {
      start = std::chrono::steady_clock::now();
    }
  }

  void sub_state_switch_status_callback(const protocol::msg::StateSwitchStatus::SharedPtr msg)
  {
    if (msg->state == 3) {
      is_ota_ = true;
    } else {
      is_ota_ = false;
    }
  }

private:
  rclcpp::Node::SharedPtr power_consumption_info_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr power_consumption_callback_group_;
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_excute_client_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_excute_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr dog_leg_calibration_srv_;
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_ {nullptr};
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  rclcpp::Publisher<protocol::msg::AudioPlayExtend>::SharedPtr audio_play_extend_pub_ {nullptr};
  rclcpp::Subscription<protocol::msg::StateSwitchStatus>::SharedPtr \
    state_swith_status_sub_ {nullptr};
  PCIN_CALLBACK request_handler;
  PCIN_CALLBACK release_handler;
  PowerMachineState pms {PowerMachineState::PMS_UNKOWN};
  PCIN_CALLBACK enter_lowpower_handler;
  bool is_lowpower_ {false};
  bool is_ota_ {false};
  int enter_lowpower_time_ {30};
  std::chrono::steady_clock::time_point start;
  std::mutex motor_mutex_;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
