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
#ifndef BLACK_BOX__BLACK_BOX_HPP_
#define BLACK_BOX__BLACK_BOX_HPP_
#include <string>
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"
#include "protocol/msg/touch_status.hpp"

namespace cyberdog
{
namespace manager
{
class BlackBox final
{
  using TouchStatusMsg = protocol::msg::TouchStatus;

public:
  explicit BlackBox(rclcpp::Node::SharedPtr node_ptr)
  {
    if (node_ptr != nullptr) {
      node_ptr_ = node_ptr;
    }
  }
  ~BlackBox() {}

  void Config()
  {
    // 读取toml配置， 选择订阅哪些消息数据
  }

  bool Init()
  {
    // 初始化数据库及ros消息回调，允许失败时返回false
    if (node_ptr_ == nullptr) {
      // error msg
      return false;
    }
    Config();
    touch_status_sub_ = node_ptr_->create_subscription<TouchStatusMsg>(
      "touch_status",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&BlackBox::TouchStatusCallback, this, std::placeholders::_1));
    return true;
  }

private:
  /* ros implementation, node and subscribers */
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Subscription<TouchStatusMsg>::SharedPtr touch_status_sub_ {nullptr};

private:
  /* topic callback */
  void TouchStatusCallback(const TouchStatusMsg::SharedPtr msg)
  {
    // save msg data
    (void)msg;
  }
};  // class BlackBox
}  // namespace manager
}  // namespace cyberdog

#endif  // BLACK_BOX__BLACK_BOX_HPP_
