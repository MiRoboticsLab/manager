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
#ifndef MANAGER_BASE__MANAGER_BASE_HPP_
#define MANAGER_BASE__MANAGER_BASE_HPP_
#include <string>
#include <map>
#include <functional>
#include <memory>
#include "rclcpp/node.hpp"
#include "cyberdog_system/robot_code.hpp"
#include "cyberdog_system/robot_state.hpp"
#include "protocol/msg/manager_state.hpp"
#include "protocol/srv/manager_init.hpp"


namespace cyberdog
{
namespace manager
{
/**
 * @brief Base class for all cyberdog managers.
 *
 */
class ManagerBase
{
  using ManagerStateMsg = protocol::msg::ManagerState;
  using ManagerInitSrv = protocol::srv::ManagerInit;
  using StateFunction = std::function<void ()>;

public:
  explicit ManagerBase(const std::string & name)
  : name_(name)
  {
    state_ = system::ManagerState::kNotReady;
    BuildStateMap();
  }
  ~ManagerBase()
  {
    if (!state_map_.empty()) {
      state_map_.clear();
    }
  }

  virtual void Config() = 0;
  virtual bool Init() = 0;
  virtual void Run() = 0;
  virtual bool SelfCheck() = 0;

// state machine functions

public:
  virtual void OnError() = 0;
  virtual void OnLowPower() = 0;
  virtual void OnSuspend() = 0;
  virtual void OnProtected() = 0;
  virtual void OnActive() = 0;

  bool RegisterRosHandler(
    rclcpp::Node::SharedPtr node_ptr,
    bool need_self_check_server = true)
  {
    if (node_ptr == nullptr) {
      return false;
    }
    state_sub_ = node_ptr->create_subscription<ManagerStateMsg>(
      "manager_set_state",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ManagerBase::StateCheckout, this, std::placeholders::_1));

    state_pub_ = node_ptr->create_publisher<ManagerStateMsg>(
      "manager_set_state",
      rclcpp::SystemDefaultsQoS());

    std::string server_name = std::string("manager_init_") + GetName();
    if (need_self_check_server) {
      init_server_ = node_ptr->create_service<ManagerInitSrv>(
        server_name,
        std::bind(
          &ManagerBase::ManagerServiceCall, this, std::placeholders::_1,
          std::placeholders::_2));
    }

    is_registered = true;
    return true;
  }

  bool SetState(int8_t state)
  {
    if (!IsRegistered() ) {
      return false;
    }
    auto msg = ManagerStateMsg();
    msg.state = state;
    this->state_pub_->publish(msg);
    return true;
  }

  system::ManagerState GetState()
  {
    return state_;
  }

  bool IsRegistered()
  {
    return is_registered;
  }

  std::string GetName()
  {
    return name_;
  }

private:
  /**
   * @brief State machine checkout callback.
   *
   * @param msg ros msg for trans state to checkout.
   */
  void StateCheckout(const ManagerStateMsg::SharedPtr msg)
  {
    auto function_ptr = state_map_.find((system::ManagerState)msg->state);
    if (function_ptr != state_map_.end()) {
      state_ = (system::ManagerState)msg->state;
      function_ptr->second;
    }
  }

  /**
   * @brief Callback function for cyberdog_manager call all managers' selfcheck.
   *
   * @param request Unused.
   * @param response res_code: -1 / 0, OK / Failed.
   */
  void ManagerServiceCall(
    const std::shared_ptr<ManagerInitSrv::Request> request,
    std::shared_ptr<ManagerInitSrv::Response> response)
  {
    (void)request;
    response->res_code = (int32_t)system::KeyCode::kOK;
    if (!SelfCheck() ) {
      response->res_code = (int32_t)system::KeyCode::kFailed;
    }
  }

  void BuildStateMap()
  {
    state_map_.insert(
      std::make_pair(
        system::ManagerState::kLowPower,
        std::bind(&ManagerBase::OnLowPower, this)));
    state_map_.insert(
      std::make_pair(
        system::ManagerState::kError,
        std::bind(&ManagerBase::OnError, this)));
    state_map_.insert(
      std::make_pair(
        system::ManagerState::kSuspend,
        std::bind(&ManagerBase::OnSuspend, this)));
    state_map_.insert(
      std::make_pair(
        system::ManagerState::kProtected,
        std::bind(&ManagerBase::OnProtected, this)));
    state_map_.insert(
      std::make_pair(
        system::ManagerState::kActive,
        std::bind(&ManagerBase::OnActive, this)));
  }

private:
  std::string name_;
  std::atomic_bool is_registered {false};
  system::ManagerState state_;
  std::map<system::ManagerState, StateFunction> state_map_;
  // rclcpp::Node::WeakPtr node_ {nullptr};
  rclcpp::Subscription<ManagerStateMsg>::SharedPtr state_sub_;
  rclcpp::Publisher<ManagerStateMsg>::SharedPtr state_pub_;
  rclcpp::Service<ManagerInitSrv>::SharedPtr init_server_;
};  // class ManagerBase
}  // namespace manager
}  // namespace cyberdog

#endif  // MANAGER_BASE__MANAGER_BASE_HPP_
