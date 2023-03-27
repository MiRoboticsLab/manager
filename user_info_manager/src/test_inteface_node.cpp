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

#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "user_info_manager/UserAccountManager.hpp"
#include "vector"
class Test_node : public rclcpp::Node
{
public:
  explicit Test_node(std::string name)
  : Node(name)
  {
    RCLCPP_INFO(this->get_logger(), "node name is %s.", name.c_str());
    INFO("enter test!!!!");
    // cyberdog::common::CyberdogAccountManager account_manager;
    // account_manager.AddMember("ding");
    // account_manager.ModifyUserInformation("fff",100,0);
    // int result[2];
    // account_manager.SearchUser("ding",result);
    // std::vector<cyberdog::common::CyberdogAccountManager::UserInformation> vectorUser;
    // account_manager.SearAllUser(vectorUser);
    // account_manager.DeleteVoice("sss");
    // account_manager.DeleteFace("ding");
    // account_manager.DeleteUserInformation("eee");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Test_node>("test_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
