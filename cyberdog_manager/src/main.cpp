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
#include "cyberdog_manager/cyberdog_manager.hpp"
#include "manager_base/manager_base.hpp"
#include "cyberdog_common/cyberdog_log.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  LOGGER_MAIN_INSTANCE("cyberdog_manager");

  std::shared_ptr<cyberdog::manager::CyberdogManager> manager =
    std::make_shared<cyberdog::manager::CyberdogManager>("cyberdog_manager");
  std::thread t([&]() {
      if (!manager->Init()) {
        ERROR("init error");
      }
    });
  t.detach();
  manager->Run();
  return 0;
}
