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

#ifndef UNLOCK_REQUEST__UNLOCK_REQUEST_HPP_
#define UNLOCK_REQUEST__UNLOCK_REQUEST_HPP_
#define CPPHTTPLIB_OPENSSL_SUPPORT
#include <stdio.h>
#include <cpp_httplib/httplib.h>
#include <string>
#include <cstdlib>
#include <fstream>
#include <cstring>
#include <chrono>
#include <thread>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_utils/motion_utils.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/srv/unlock.hpp"
#include "protocol/srv/unlock_status.hpp"
#include "low_power_consumption/low_power_consumption.hpp"
#include "protocol/srv/bes_http.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "protocol/srv/reboot_machine.hpp"
#include "black_box/black_box.hpp"
// using namespace std::chrono_literals;
namespace cyberdog
{
namespace manager
{
class UnlockRequestNode : public rclcpp::Node
{
public:
  UnlockRequestNode()
  : Node("unlock_request")
  {
    INFO("unlock request start");
    unlock_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    unlock_requset_srv_ =
      this->create_service<protocol::srv::Unlock>(
      "unlock_develop_access",
      std::bind(
        &UnlockRequestNode::UnlockMachineRequest,
        this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, unlock_callback_group_);
    reboot_machine_srv_ =
      this->create_service<protocol::srv::RebootMachine>(
      "reboot_machine",
      std::bind(
        &UnlockRequestNode::RstartMachine,
        this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, unlock_callback_group_);
    motion_result_client_ =
      this->create_client<protocol::srv::MotionResultCmd>("motion_result_cmd");
    bes_http_client_ =
      this->create_client<protocol::srv::BesHttp>("bes_http_srv");
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
    std::thread UpSendThread = std::thread(&UnlockRequestNode::SendUnlockStatus_, this);
    UpSendThread.detach();
    INFO("unlock request node init success");
  }

private:
  std::string GetKeyFile()
  {
    std::string ShellCommand_ = "cd /SDCARD && tar -xf pub_key.tar";
    FILE * ShellResult = popen(ShellCommand_.c_str(), "r");
    if (!ShellResult) {
      INFO("popen unlock get key file error,errno = %d\n", errno);
    }
    return "/SDCARD";
  }
  void SendUnlockStatus_()
  {
    INFO("enter SendUnlockStatus_");
    bool modify_flag = true;
    while (flag) {
      std::string filename_ = "/opt/ros2/cyberdog/share/params/toml_config/manager";
      std::string filename = filename_ + "/UnlockStatus.db";
      int result[2];
      if (!black_box.readUnlockStatus(result)) {
        flag = false;
        modify_flag = false;
        break;
      } else {
        if (result[0] == 0 && result[1] == 1) {
          INFO("[SendUnlockStatus_]>>UpSend has successed and not reboot");
          flag = false;
          modify_flag = false;
          break;
          // return;
        }
      }
      int unlockStatus_ = GetUnlockStatus();
      INFO("[SendUnlockStatus_]>>the unlock status is %d", unlockStatus_);
      if (SendUnlockStatusClient(unlockStatus_)) {              // 上报接收成功
        INFO("[SendUnlockStatus_]>>send success");
        black_box.ModifyUnlockStatus("UPSENDSTATUS", 1);
        flag = false;
        break;
      } else {
        INFO("[SendUnlockStatus_]>>send failed");
        black_box.ModifyUnlockStatus("UPSENDSTATUS", 0);
        flag = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    if (modify_flag) {
      black_box.ModifyUnlockStatus("REBOOTSTATUS", 0);
    }
  }
  int GetUnlockStatus()
  {
    std::string parament = " unlock-state";
    std::string ShellCommand = "/usr/bin/cyberdog_get_info " + parament;
    INFO("command is : %s", ShellCommand.c_str());
    FILE * ShellResult = popen(ShellCommand.c_str(), "r");
    char ShellResultBuffer_[256];
    fread(ShellResultBuffer_, 1, sizeof(ShellResultBuffer_), ShellResult);
    std::string result_str_ = ShellResultBuffer_;
    pclose(ShellResult);
    INFO("get unlock status return int is :%s", result_str_.c_str());
    return GetUnlockCommand();
  }
  int UnlockAccess(const std::string KeyPath)
  {
    INFO("enter UnlockAccess function");
    std::string parammentKey = KeyPath + "/cyberdog-key";
    std::string parammentKey_ = KeyPath + "/cyber-pub.pem";
    std::string ShellCommand = "/usr/bin/cyberdog_get_info " + parammentKey + " " + parammentKey_;
    INFO("%s", ShellCommand.c_str());
    FILE * ShellResult = popen(ShellCommand.c_str(), "r");
    char ShellResultBuffer_[256];
    fread(ShellResultBuffer_, 1, sizeof(ShellResultBuffer_), ShellResult);
    std::string result_str_ = ShellResultBuffer_;
    pclose(ShellResult);
    INFO("unlock status return int is :%s", result_str_.c_str());
    return GetUnlockCommand();
  }
  int GetUnlockCommand()
  {
    int result = 10000;
    std::string ShellCommand = "echo $?";
    INFO("GetUnlockCommand() :%s", ShellCommand.c_str());
    FILE * ShellResult_ = popen(ShellCommand.c_str(), "r");
    char ShellResultBuffer[256];
    fread(ShellResultBuffer, 1, sizeof(ShellResultBuffer), ShellResult_);
    std::string result_str = ShellResultBuffer;
    pclose(ShellResult_);
    INFO("unlock status return int is :%s", result_str.c_str());
    result = std::stoi(result_str);
    return result;
  }
  bool SendUnlockStatusClient(const int Unlock_status)
  {
    INFO("enter SendUnlockStatusClient function");
    std::chrono::seconds timeout(5);
    auto req = std::make_shared<protocol::srv::BesHttp::Request>();
    INFO("BesHttp_srv init success");
    req->method = 1;
    req->url = "device/developerInfo/result";
    rapidjson::Document json_response(rapidjson::kObjectType);
    std::string rsp_string;
    cyberdog::common::CyberdogJson::Add(json_response, "result", Unlock_status);
    if (!cyberdog::common::CyberdogJson::Document2String(json_response, rsp_string)) {
      ERROR("error while set mic state response encoding to json");
    }
    INFO("response result of send unlock status is %s", rsp_string.c_str());
    req->params = rsp_string;
    req->milsecs = 5000;
    auto future_result = bes_http_client_->async_send_request(req);
    std::future_status status = future_result.wait_for(timeout);
    if (status == std::future_status::ready) {
      INFO("success to call bes_http_transmit services.");
    } else {
      INFO("Failed to call bes_http_transmit services.");
      return false;
    }
    INFO("bes http  result : %s", future_result.get()->data.c_str());
    std::string return_true = "true";
    std::string return_false = "false";
    std::string::size_type idx;
    idx = future_result.get()->data.find(return_true);
    if (idx == std::string::npos) {
      idx = future_result.get()->data.find(return_false);
      if (idx == std::string::npos) {
        return false;
      } else {
        return false;                  // 后台返回字符串中包含false;
      }
    } else {
      return true;                // 后台返回字符串中包含true;
    }
  }
  void RstartMachine(
    const protocol::srv::RebootMachine::Request::SharedPtr request,
    protocol::srv::RebootMachine::Response::SharedPtr response)
  {
    INFO("enter Restart Machine");
    if (request->rebootmachine == 1997) {
      INFO("start reboot");
      black_box.ModifyUnlockStatus("REBOOTSTATUS", 1);
      black_box.ModifyUnlockStatus("UPSENDSTATUS", 0);
      PM_SYS pd = PM_SYS_REBOOT;
      int reboot_result = lpc_ptr_->LpcSysRequest(pd);
      INFO("reboot result is %d", reboot_result);
      response->rebootresult = reboot_result;
    }
  }
  void UnlockMachineRequest(
    const protocol::srv::Unlock::Request::SharedPtr request,
    protocol::srv::Unlock::Response::SharedPtr response)
  {
    INFO("enter unlock_request service");
    std::chrono::seconds timeout(5);
    auto req = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    INFO("requestMotion init success");
    req->motion_id = 101;
    auto future_result = motion_result_client_->async_send_request(req);
    std::future_status status = future_result.wait_for(timeout);
    if (status == std::future_status::ready) {
      INFO("success to call callMotionServoCmd services.");
    } else {
      INFO("Failed to call callMotionServoCmd services.");
      response->unlock_result = 111;
      return;
    }
    black_box.CreateUnlockStatusDB();              // 创建或复原上报状态记录文件
    INFO("%s", request->httplink.c_str());
    std::string httpCom = "https://cnbj2m.fds.api.xiaomi.com";
    std::string httpFilePath = request->httplink.substr(httpCom.length());
    INFO("request httplink is %s", request->httplink.c_str());
    INFO("http link path is %s", httpFilePath.c_str());
    httplib::Client cli(httpCom);
    cli.set_read_timeout(10, 0);
    auto res = cli.Get(httpFilePath);
    if (res->status != 200) {
      INFO("Get key file error");
      return;
    }
    if (res->body.length() == 0) {
      INFO("the key file is empty");
      return;
    }
    std::fstream f;
    f.open("/SDCARD/pub_key.tar", std::ios::out);
    f << res->body << std::endl;
    f.close();
    std::string keyPath = GetKeyFile();
    INFO("key path is %s", keyPath.c_str());
    switch (UnlockAccess(keyPath)) {
      case 0:
        {
          INFO("unlock success!!!");
          response->unlock_result = 0;
        }
        break;
      case 1:
        {
          INFO("unlocked failed,parameter verification failes");
          response->unlock_result = 1;
        }
        break;
      case 2:
        {
          INFO("unlocked failed,the key information does not match the device information");
          response->unlock_result = 2;
        }
        break;
      case 3:
        {
          INFO("unlocked failed,operation control (MR813) failed to unlock Settings");
          response->unlock_result = 3;
        }
        break;
      case 4:
        {
          INFO("unlocked failed,voice control (R239) unlock Settings failed;");
          response->unlock_result = 4;
        }
        break;
      case 5:
        {
          INFO("unlocked failed,the device has not been unlocked;");
          response->unlock_result = 5;
        }
        break;
      case 100:
        {
          INFO("unlocked failed,unknow error!!");
          response->unlock_result = 100;
        }
        break;
      default:
        {
          INFO("shell return error: default!!");
          response->unlock_result = 100;
        }
        break;
    }
  }

private:
  rclcpp::Service<protocol::srv::Unlock>::SharedPtr unlock_requset_srv_;
  rclcpp::Service<protocol::srv::RebootMachine>::SharedPtr reboot_machine_srv_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_;
  rclcpp::Client<protocol::srv::BesHttp>::SharedPtr bes_http_client_;
  rclcpp::CallbackGroup::SharedPtr unlock_callback_group_;
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_{nullptr};
  cyberdog::manager::BlackBox black_box;
  bool flag = true;
};
}  // namespace manager
}  // namespace cyberdog
#endif  // UNLOCK_REQUEST__UNLOCK_REQUEST_HPP_
