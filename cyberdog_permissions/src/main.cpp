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
#include <memory>
#include "cyberdog_permission/cyberdog_permission.hpp"
#include "cyberdog_common/cyberdog_log.hpp"


int main(int argc, char ** argv)
{
  LOGGER_MAIN_INSTANCE(NODE_NAME);
  rclcpp::init(argc, argv);
  auto permission_node =
    std::make_shared<cyberdog::manager::CyberdogPermission>();
  try {
    rclcpp::spin(permission_node);
  } catch (rclcpp::exceptions::RCLError & e) {
    ERROR(
      "node spin rcl error exception:(line:%d,file:%d,messgae:%s[%s])",
      e.line, e.file, e.message, e.formatted_message);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  } catch (...) {
    ERROR("node spin unkown exception");
  }
  rclcpp::shutdown();
  return 0;
}
