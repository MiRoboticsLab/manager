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
#ifndef CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
#define CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "protocol/msg/motion_status.hpp"
#include "low_power_consumption/low_power_consumption.hpp"

namespace cyberdog
{
namespace manager
{
class PowerConsumptionInfoNode final
{
  using PCIN_CALLBACK = std::function<void ()>;

public:
  explicit PowerConsumptionInfoNode(rclcpp::Node::SharedPtr node_ptr)
  : request_handler([](void) {}), release_handler([](void) {})
  {
    power_consumption_info_node_ = node_ptr;
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
    power_consumption_callback_group_ =
      power_consumption_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    low_power_consumption_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::SetBool>(
      "low_power_consumption",
      std::bind(
        &PowerConsumptionInfoNode::EnterLowPower, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);
    // sub motion init
    rclcpp::SubscriptionOptions options;
    options.callback_group = power_consumption_callback_group_;
    motion_status_sub_ = power_consumption_info_node_->
      create_subscription<protocol::msg::MotionStatus>(
      "motion_status", 10, std::bind(
        &PowerConsumptionInfoNode::sub_mostion_status_callback,
        this, std::placeholders::_1),
      options);
  }

public:
  void SetActive(PCIN_CALLBACK callback)
  {
    request_handler = callback;
  }
  void SetDeactive(PCIN_CALLBACK callback)
  {
    release_handler = callback;
  }

private:
  void EnterLowPower(
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
  void sub_mostion_status_callback(const protocol::msg::MotionStatus msg)
  {
    // INFO("sub motion_id messages*********");
    // motion_id: 趴下(101)、站立(111)
    static bool convert_motion_flage = true;
    static int lay_count = 0;
    static bool times_flag = true;
    static int count = 0;
    PM_DEV pd = PM_ALL;
    unsigned int err;
    int code = -1;

    // 启动后不进行任何操作，60s后进入低功耗
    if (times_flag == true && msg.motion_id == 0) {
      ++count;
      if (count == 100) {
        count = 0;
        convert_motion_flage = false;
        times_flag = false;
        INFO("call low power consumption");
        // code = lpc_ptr_->LpcRelease(pd, &err);
        // if(code == 0)
        release_handler();
      }
    } else {
      count = 0;
    }
    // 运动状态转换至趴下，30s后启动低功耗
    if (convert_motion_flage == true && msg.motion_id == 101) {
      // motion_status的发布频率为10Hz，延时30s，lay_count == 300
      ++lay_count;
      if (lay_count == 300) {
        INFO("call low power consumption");
        INFO("call low power");
        // code = lpc_ptr_->LpcRelease(pd, &err);
        // if(code == 0)
        release_handler();
        convert_motion_flage = false;
        lay_count = 0;
      }
    } else {
      lay_count = 0;
    }

    // 状态切换到站立，启动正常功耗
    if (convert_motion_flage == false && msg.motion_id == 111) {
      INFO("call nomal power consumption");
      // code = lpc_ptr_->LpcRelease(pd, &err);
      // if(code == 0)
      request_handler();
      convert_motion_flage = true;
      times_flag = true;
    }
  }

private:
  rclcpp::Node::SharedPtr power_consumption_info_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr power_consumption_callback_group_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_power_consumption_srv_;
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_ {nullptr};
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  PCIN_CALLBACK request_handler;
  PCIN_CALLBACK release_handler;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
