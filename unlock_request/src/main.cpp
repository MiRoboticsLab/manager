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

#include <memory>
#include "unlock_request/unlock_request.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("unlock_request");
  auto service_node = std::make_shared<cyberdog::manager::UnlockRequestNode>();
  rclcpp::executors::MultiThreadedExecutor executor_;
  executor_.add_node(service_node);
  executor_.spin();
  rclcpp::shutdown();
  return 0;
}
