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
#include <string>
#include "cyberdog_permission/cyberdog_permission.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#define MAX_SN_SIZE  50
#define LINE_MAX_SIZE 1024

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

cyberdog::manager::CyberdogPermission::CyberdogPermission()
: Node(NODE_NAME)
{
  INFO("start get sn");
  cyberdog_sn = BoardInfo::Get_Sn();
  INFO("sn:%s", cyberdog_sn.c_str());
  if (cyberdog_sn.length() > MAX_SN_SIZE) {
    cyberdog_sn.resize(MAX_SN_SIZE);
    cyberdog_sn[MAX_SN_SIZE - 1] = '\0';
  }
  sn_pub_ =
    this->create_publisher<std_msgs::msg::String>(
    "dog_sn",
    rclcpp::SystemDefaultsQoS());
  std::thread t([this]() {
      try {
        auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
        auto path = local_share_dir + std::string("/toml_config/manager");
        if (access(path.c_str(), F_OK) != 0) {
          std::string cmd = "mkdir -p " + path;
          std::system(cmd.c_str());
          cmd = "chmod 777 " + path;
          std::system(cmd.c_str());
        }
        uint16_t pub_cnt = 0;
        while (rclcpp::ok() && (++pub_cnt < 3600)) {
          std_msgs::msg::String msg;
          msg.data = cyberdog_sn;
          sn_pub_->publish(msg);
          INFO_MILLSECONDS(3000, "publish sn:%s", cyberdog_sn.c_str());
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      } catch (...) {
        ERROR("mkdir manager directory error.");
      }
    });
  t.detach();
  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  sn_srv_ =
    this->create_service<std_srvs::srv::Trigger>(
    "get_dog_sn",
    std::bind(&CyberdogPermission::SnCallback, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default,
    callback_group_);
  realsense_recovery_srv_ =
    this->create_service<std_srvs::srv::Trigger>(
    "realsense_recovery",
    std::bind(
      &CyberdogPermission::RealsenseRecovery,
      this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default);
}

void cyberdog::manager::CyberdogPermission::SnCallback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  response->success = true;
  response->message = cyberdog_sn;
  INFO("reponse:get dog sn:--%s", response->message.c_str());
}

void cyberdog::manager::CyberdogPermission::RealsenseRecovery(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::string cmd = "realsense_recovery.sh";
  int32_t code;
  std::string message;
  bool ok = Shell(cmd, code, message);
  INFO("shell message:%s", message.c_str());
  if (code == 0) {
    response->success = true;
    response->message = "ok";
  } else {
    response->success = false;
    response->message = "fail";
  }
}

bool cyberdog::manager::CyberdogPermission::Shell(
  const std::string & _command, int & _code, std::string & _message)
{
  try {
    INFO("Shell: %s", _command.c_str());
    _message = "";
    if (!_command.empty()) {
      std::string _code_cmd = _command + "; echo $?";  //  | xargs echo
      std::string _code_str = "";
      FILE * fstream_ptr = nullptr;
      fstream_ptr = popen(_code_cmd.c_str(), "r");
      if (fstream_ptr != nullptr) {
        char buffer[LINE_MAX_SIZE];
        while (fgets(buffer, LINE_MAX_SIZE, fstream_ptr) != nullptr) {
          _code_str = buffer;
          memset(buffer, '\0', sizeof(buffer));
          _message += _code_str;
        }
        pclose(fstream_ptr);
        fstream_ptr = nullptr;
        _code = std::atoi(_code_str.c_str());
        if (_code == static_cast<int>(0)) {
          return true;
        } else {
          return false;
        }
      } else {
        _code = -1;
        _message = "Canot shell command popen.\n - command : " + _command +
          "\n - error : " + strerror(errno);
      }
    } else {
      _code = -2;
      _message = "Shell command is empty.\n - command : " + _command;
    }
  } catch (const std::exception & e) {
    _code = -3;
    _message = "Shell command is error.\n - command : " + _command +
      "\n - error : " + e.what();
  }
  ERROR("Shell: %s, %s", _command.c_str(), _message.c_str());
  return false;
}
