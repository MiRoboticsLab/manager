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
#ifndef CYBERDOG_PERMISSION__CYBERDOG_PERMISSION_HPP_
#define CYBERDOG_PERMISSION__CYBERDOG_PERMISSION_HPP_
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_permission/board_info.hpp"
#include "std_srvs/srv/trigger.hpp"

#define NODE_NAME "cyberdog_permission"

namespace cyberdog
{
namespace manager
{
class CyberdogPermission final : public rclcpp::Node
{
public:
  CyberdogPermission();

private:
  void SnCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

private:
  std::string cyberdog_sn;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sn_srv_;
};  // class CyberdogPermission
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_PERMISSION__CYBERDOG_PERMISSION_HPP_
