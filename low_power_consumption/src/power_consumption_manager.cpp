// Copyright (c) 2021 Xiaomi Corporation
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

#include <low_power_consumption/power_consumption_manager.hpp>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{
PowerConsumptionManager::PowerConsumptionManager(const std::string & name)
{
  this->node_sub_motion_ptr_ = rclcpp::Node::make_shared(name);
  this->node_power_consump_ptr_ = rclcpp::Node::make_shared("power_consumption");
  INFO("Creating sub_node object(node)");
  INFO("Creating power_consumption object(node)");
}

PowerConsumptionManager::~PowerConsumptionManager()
{
  INFO("Destroy power_consumption object(node)");
}

bool PowerConsumptionManager::Init()
{
  INFO("PowerConsumptionManager node init");
  // sub motion init
  this->motion_status_sub_ = this->node_sub_motion_ptr_->
    create_subscription<protocol::msg::MotionStatus>(
    "motion_status", 10, std::bind(
      &PowerConsumptionManager::sub_mostion_status_callback,
      this, std::placeholders::_1));

  // power manager srv init
  this->power_consumption_manager_srv_ = this->node_power_consump_ptr_->
    create_service<std_srvs::srv::SetBool>(
    "power_consumption_manager", std::bind(
      &PowerConsumptionManager::EnterLowPower,
      this, std::placeholders::_1, std::placeholders::_2));

  this->power_comsumption_client_ = this->node_power_consump_ptr_->
    create_client<std_srvs::srv::SetBool>("power_consumption_manager");

  // create subcribe
  this->lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
  return true;
}

bool PowerConsumptionManager::send_power_manager_request(const bool msg)
{
  if (!power_comsumption_client_->wait_for_service()) {
    INFO(
      "call power consumption manager server not avalible");
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = msg;
  auto result_future = power_comsumption_client_->async_send_request(req);
  std::chrono::seconds timeout(3);
  std::future_status status = result_future.wait_for(timeout);

  if (status == std::future_status::ready) {
    INFO("success to call the low_power_comsumption server.");
  } else {
    INFO("Failed to call the low_power_comsumption server.");
    return false;
  }
  // INFO("result is: %ld", result_future.get()->success);
  return true;
}

void PowerConsumptionManager::sub_mostion_status_callback(
  const protocol::msg::MotionStatus msg)
{
  // INFO("sub motion_id messages*********");
  // motion_id: 趴下(101)、站立(111)
  static bool convert_motion_flage = true;
  static int lay_count = 0;

  static bool times_flag = true;
  static int count = 0;

  // 启动(或急停)后不进行任何操作，60s后进入低功耗
  if (times_flag == true && msg.motion_id == 0) {
    ++count;
    INFO("lay_count %d", count);
    if (count == 600) {
      count = 0;
      convert_motion_flage = false;
      times_flag = false;
      INFO("call low power consumption");
      send_power_manager_request(true);
    }
  } else {
    count = 0;
  }

  // 运动状态转换至趴下，30s后启动低功耗
  if (convert_motion_flage == true && msg.motion_id == 101) {
    // motion_status的发布频率为10Hz，延时30s，lay_count == 300
    ++lay_count;
    INFO("lay_count = %d", lay_count);
    if (lay_count == 300) {
      INFO("call low power consumption");
      INFO("call low power");
      send_power_manager_request(true);
      convert_motion_flage = false;
      lay_count = 0;
    }
  } else {
    lay_count = 0;
  }

  // 状态切换到站立，启动正常功耗
  if (convert_motion_flage == false && msg.motion_id == 111) {
    INFO("call nomal power consumption");
    send_power_manager_request(false);
    convert_motion_flage = true;
    times_flag = true;
  }
}

void PowerConsumptionManager::EnterLowPower(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  static int r_count = 0;
  INFO("[%d]EnterLowPower %s:start", (r_count + 1), (request->data ? "true" : "false"));
  PM_DEV pd = PM_ALL;
  unsigned int err;
  int code = -1;
  if (request->data) {
    INFO("starting low power consumption");
    code = lpc_ptr_->LpcRelease(pd, &err);
    ++r_count;
    INFO("LpcRelease() return is %d", code);
  } else {
    INFO("starting nomal power consumption");
    code = lpc_ptr_->LpcRequest(pd, &err);
    ++r_count;
    INFO("LpcRequest() return is %d", code);
  }
  response->success = (code == 0 ? true : false);
  if (code != 0) {
    response->message = std::to_string(err);
    INFO("err is %d", err);
  }
  INFO("[%d]EnterLowPower %s:stop", (r_count + 1), (request->data ? "true" : "false"));
}

void PowerConsumptionManager::Run()
{
  INFO("cyberdog power manager node spin");
  this->executor_.add_node(node_sub_motion_ptr_);
  this->executor_.add_node(node_power_consump_ptr_);
  this->executor_.spin();
  rclcpp::shutdown();
}
}  // namespace manager
}  // namespace cyberdog
