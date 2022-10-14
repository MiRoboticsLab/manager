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
#include "low_power_consumption/low_power_consumption.hpp"

namespace cyberdog
{
namespace manager
{
class PowerConsumptionInfoNode final
{
public:
  explicit PowerConsumptionInfoNode(rclcpp::Node::SharedPtr node_ptr)
  {
    power_consumption_info_node_ = node_ptr;
    power_consumption_callback_group_ =
      power_consumption_info_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    low_power_consumption_srv_ =
      power_consumption_info_node_->create_service<std_srvs::srv::SetBool>(
      "low_power_consumption",
      std::bind(
        &PowerConsumptionInfoNode::EnterLowPower, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, power_consumption_callback_group_);
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
  }
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

private:
  rclcpp::Node::SharedPtr power_consumption_info_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr power_consumption_callback_group_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_power_consumption_srv_;
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_ {nullptr};
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__POWER_CONSUMPTION_INFO_HPP_
