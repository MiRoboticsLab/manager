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

#ifndef LOW_POWER_CONSUMPTION__POWER_CONSUMPTION_MANAGER_HPP_
#define LOW_POWER_CONSUMPTION__POWER_CONSUMPTION_MANAGER_HPP_

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "protocol/msg/motion_status.hpp"
#include "low_power_consumption/low_power_consumption.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{
class PowerConsumptionManager
{
public:
  explicit PowerConsumptionManager(const std::string & name);
  ~PowerConsumptionManager();

  bool low_power_consumption_set();
  bool nomal_power_consumption_set();

  bool Init();
  void Run();

private:
  void sub_mostion_status_callback(const protocol::msg::MotionStatus msg);
  bool send_power_manager_request(const bool msg);
  void EnterLowPower(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);
  rclcpp::Node::SharedPtr node_sub_motion_ptr_ {nullptr};
  rclcpp::Node::SharedPtr node_power_consump_ptr_ {nullptr};
  rclcpp::Subscription<protocol::msg::MotionStatus>::SharedPtr motion_status_sub_ {nullptr};
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr power_comsumption_client_ {nullptr};
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_consumption_manager_srv_ {nullptr};
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_ {nullptr};
  rclcpp::executors::MultiThreadedExecutor executor_;
};  // class PowerConsumptionManager

}   // namespace manager
}   // namespace cyberdog

#endif  // LOW_POWER_CONSUMPTION__POWER_CONSUMPTION_MANAGER_HPP_
