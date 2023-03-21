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

#include <black_box/black_box.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>
#include <memory>
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("Global_Name");
  auto node = rclcpp::Node::make_shared("test");
  cyberdog::manager::BlackBox test_data;
  test_data.AddUser("xiaomi");
  test_data.AddUser("xiaoxiao");
  test_data.ModifyUserName("xiaomi", "mimi");
  auto black_box_ptr = std::make_shared<cyberdog::manager::BlackBox>(node);
  auto result = black_box_ptr->Init();
  INFO("init: %d", result);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
