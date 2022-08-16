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
#include "cyberdog_permission/cyberdog_permission.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

cyberdog::manager::CyberdogPermission::CyberdogPermission()
: Node(NODE_NAME)
{
  cyberdog_sn = BoardInfo::Get_Sn();
  INFO("sn:%s", cyberdog_sn.c_str());
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto path = local_share_dir + std::string("/toml_config/manager");
  if (access(path.c_str(), F_OK) != 0) {
    std::string cmd = "mkdir -p " + path;
    std::system(cmd.c_str());
    cmd = "chmod 777 " + path;
    std::system(cmd.c_str());    
  }  
  sn_srv_ =
    this->create_service<std_srvs::srv::Trigger>(
    "get_dog_sn",
    std::bind(&CyberdogPermission::SnCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void cyberdog::manager::CyberdogPermission::SnCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = true;
  response->message = cyberdog_sn;
}