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
#ifndef CYBERDOG_MANAGER__ENV_CONTEX_HPP_
#define CYBERDOG_MANAGER__ENV_CONTEX_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/trigger.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{

class EnvContex final
{
public:
  explicit EnvContex(rclcpp::Node::SharedPtr node_ptr)
  : env_contex_node_(node_ptr), environment_contex_("pro")
  {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/env.json");
    Document json_document(kObjectType);
    auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
    if (result) {
      std::string env_str;
      CyberdogJson::Get(json_document, "environment", env_str);
      if (!env_str.empty()) {
        INFO("environment:%s.", env_str.c_str());
        environment_contex_ = env_str;
      }
    }
    env_callback_group_ =
      env_contex_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    switch_environment_srv_ =
      env_contex_node_->create_service<protocol::srv::Trigger>(
      "set_nx_environment",
      std::bind(
        &EnvContex::SwitchEnvironmentService, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, env_callback_group_);
    get_environment_srv_ =
      env_contex_node_->create_service<std_srvs::srv::Trigger>(
      "get_nx_environment",
      std::bind(
        &EnvContex::GetEnvironmentService, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, env_callback_group_);
  }

private:
  void SwitchEnvironmentService(
    const protocol::srv::Trigger::Request::SharedPtr request,
    protocol::srv::Trigger::Response::SharedPtr response)
  {
    environment_contex_ = request->data;
    INFO("switch nx environment:%s", environment_contex_.c_str());
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager");
    auto json_file = path + "/env.json";
    Document json_document(kObjectType);
    CyberdogJson::ReadJsonFromFile(json_file, json_document);
    CyberdogJson::Add(json_document, "environment", environment_contex_);
    CyberdogJson::WriteJsonToFile(json_file, json_document);
    response->success = true;
  }

  void GetEnvironmentService(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    response->message = environment_contex_;
    response->success = true;
  }

private:
  rclcpp::Node::SharedPtr env_contex_node_{nullptr};
  std::string environment_contex_;
  rclcpp::CallbackGroup::SharedPtr env_callback_group_;
  rclcpp::Service<protocol::srv::Trigger>::SharedPtr switch_environment_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_environment_srv_;
};

}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__ENV_CONTEX_HPP_
