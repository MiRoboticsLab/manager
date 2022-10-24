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
#ifndef CYBERDOG_MANAGER__STATE_CONTEXT_HPP_
#define CYBERDOG_MANAGER__STATE_CONTEXT_HPP_

#include <string>
#include <memory>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"

namespace cyberdog
{
namespace manager
{
class StateContext final
{
public:
  explicit StateContext(std::string node_name)
  : name_(node_name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name_);
    machine_context_ptr_ = std::make_unique<cyberdog::machine::MachineContext>();
    current_state_ = machine_context_ptr_->Context(
      cyberdog::machine::MachineState::MS_Uninitialized);
    machine_controller_ptr_ = std::make_shared<cyberdog::machine::MachineController>();
    std::thread(
      [this]() {
        rclcpp::spin(node_ptr_);
    }).detach();
  }

  ~StateContext() {}

  bool Init()
  {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/state_machine_config.toml");
    if (!machine_controller_ptr_->MachineControllerInit(
        path, node_ptr_))
    {
      ERROR("Machine State Init failed!");
      return false;
    } else if (!machine_controller_ptr_->WaitActuatorsSetUp()) {
      ERROR("Machine State Init failed, actuators setup failed!");
      return false;
    } else {
      INFO("Machine State Init OK.");
      return true;
    }
  }

  bool SetState(cyberdog::machine::MachineState ms)
  {
    current_state_ = machine_context_ptr_->Context(ms);
    if (!machine_controller_ptr_->SetState(current_state_)) {
      ERROR("set state:%s failed, cannot running!", current_state_.c_str());
      return false;
    }
    return true;
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_{nullptr};
  std::shared_ptr<cyberdog::machine::MachineController> machine_controller_ptr_ {nullptr};
  std::string current_state_;
  std::unique_ptr<cyberdog::machine::MachineContext> machine_context_ptr_ {nullptr};
};
}  // namespace manager
}  // namespace cyberdog
#endif  // CYBERDOG_MANAGER__STATE_CONTEXT_HPP_
