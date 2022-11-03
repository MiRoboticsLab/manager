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
#ifndef CYBERDOG_MANAGER__TOUCH_INFO_HPP_
#define CYBERDOG_MANAGER__TOUCH_INFO_HPP_

#include "rclcpp/rclcpp.hpp"

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
  }

private:
  rclcpp::Node::SharedPtr touch_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr touch_callback_group_;
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__TOUCH_INFO_HPP_
