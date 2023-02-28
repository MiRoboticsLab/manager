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
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "motion_utils/motion_utils.hpp"
#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/msg/motion_status.hpp"
#include "protocol/srv/unlock.hpp"
#include "low_power_consumption/low_power_consumption.hpp"
#include "protocol/srv/bes_http.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "protocol/srv/reboot_machine.hpp"
#include "black_box/black_box.hpp"
#include "protocol/msg/self_check_status.hpp"
#include "protocol/srv/machine_state.hpp"
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
    client_cb_group_ =
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
        &UnlockRequestNode::RestartMachine,
        this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, unlock_callback_group_);
    motion_result_client_ =
      this->create_client<protocol::srv::MotionResultCmd>("motion_result_cmd");
    machine_state_switch_keep_client_ =
      this->create_client<protocol::srv::MachineState>(
      "machine_state_switch_keep",
      rmw_qos_profile_services_default, client_cb_group_);
    bes_http_client_ =
      this->create_client<protocol::srv::BesHttp>(
      "bes_http_srv",
      rmw_qos_profile_services_default, client_cb_group_);
    lpc_ptr_ = std::make_unique<cyberdog::manager::LowPowerConsumption>();
    callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;
    self_check_sub =
      this->create_subscription<protocol::msg::SelfCheckStatus>(
      "self_check_status", rclcpp::QoS(10),
      std::bind(&UnlockRequestNode::SelfCheckTopicCallback, this, std::placeholders::_1),
      sub1_opt);
    INFO("unlock request node init success");
  }

private:
  void SelfCheckTopicCallback(const protocol::msg::SelfCheckStatus::SharedPtr msg)
  {
    if (msg->code == 0 || msg->code == 2) {
      if (upsend_counter < 3) {
        INFO_MILLSECONDS(30000, "enter SelfCheckTopicCallback");
        SendUnlockStatus();
        upsend_counter++;
      }
    }
  }
  std::string GetKeyFile()
  {
    std::string ShellCommand_ = "cd /SDCARD && tar -xf pub_key.tar";
    FILE * ShellResult = popen(ShellCommand_.c_str(), "r");
    if (!ShellResult) {
      INFO("popen unlock get key file error,errno = %d\n", errno);
    }
    return "/SDCARD";
  }
  void SendUnlockStatus()
  {
    INFO("enter SendUnlockStatus()");
    bool modify_flag = true;
    std::string filename_ = "/opt/ros2/cyberdog/share/params/toml_config/manager";
    std::string filename = filename_ + "/UnlockStatus.db";
    int result[2];
    if (flag) {
      if (!black_box.readUnlockStatus(result)) {
        modify_flag = false;
        return;
      } else {
        if (result[0] == 0 && result[1] == 1) {
          INFO("[SendUnlockStatus]>>UpSend has successed and not reboot");
          modify_flag = false;
          return;
        }
      }
      int unlockStatus_ = GetUnlockStatus();
      INFO("[SendUnlockStatus]>>the unlock status is %d", unlockStatus_);
      if (SendUnlockStatusClient(unlockStatus_)) {              // 上报接收成功
        INFO("[SendUnlockStatus_]>>send success");
        black_box.ModifyUnlockStatus("UPSENDSTATUS", 1);
        flag = false;
        return;
      } else {
        INFO("[SendUnlockStatus_]>>send failed");
        black_box.ModifyUnlockStatus("UPSENDSTATUS", 0);
        flag = true;
      }
      std::this_thread::sleep_for(std::chrono::seconds(10));
      if (modify_flag) {
        black_box.ModifyUnlockStatus("REBOOTSTATUS", 0);
      }
    }
  }
  std::vector<std::string> GetVector(
    const std::string & _message, char _delim,
    const std::string & _head)
  {
    std::vector<std::string> _vector;
    std::stringstream message_str;
    message_str.str(_message);
    std::string elems;
    while (std::getline(message_str, elems, _delim)) {
      _vector.push_back(std::string(_head + elems));
    }
    return _vector;
  }
  std::string RunShellCommand(const char * cmd, bool & success)
  {
    std::array<char, 128> buffer;
    std::string result;
    int exec_result = -1;
    auto pipe = popen(cmd, "r");
    INFO("RunCommand is [ %s ]", cmd);
    if (!pipe) {
      throw std::runtime_error("popen() failed!");
      success = false;
    }
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
      result += buffer.data();
      INFO("%s", buffer.data());
    }
    exec_result = pclose(pipe);
    INFO("exec_result = %d", exec_result);
    success = exec_result == 0 ? true : false;
    return result;
  }
  int GetUnlockStatus()
  {
    std::string parament = " unlock-state; echo $?";
    std::string ShellCommand = "/usr/bin/cyberdog_get_info" + parament;
    int result = GetShellCommandResult(ShellCommand);
    INFO("GetUnlockStatus() will return : %d", result);
    return result;
  }

  int UnlockAccess()
  {
    std::string parammentKey = "/SDCARD/cyberdog-key; echo $?";
    std::string ShellCommand = "/usr/bin/cyberdog_get_info unlock-request " + parammentKey;
    int result = GetShellCommandResult(ShellCommand);
    INFO("UnlockAccess() will return : %d", result);
    return result;
  }
  int GetShellCommandResult(const std::string ShellCommand)
  {
    bool success;
    int result = 100;
    std::string result_str_ = RunShellCommand(ShellCommand.c_str(), success);
    INFO("RunShellCommand() return :%s", result_str_.c_str());
    if (success) {
      std::vector<std::string> result_vector = GetVector(result_str_, '\n', "");
      std::string unlock_result = result_vector.back();
      if (!unlock_result.empty()) {
        INFO("result_vector.end(): %s", unlock_result.c_str());
        result = std::stoi(unlock_result);
      }
    }
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
  void RestartMachine(
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
  void RequestMotion()
  {
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
      return;
    }
  }
  bool RequestStateKeep()
  {
    std::chrono::seconds timeout(5);
    auto req = std::make_shared<protocol::srv::MachineState::Request>();
    INFO("machine_state_switch_keep init success");
    req->state = 0;
    req->ticks = 30;
    auto future_result = machine_state_switch_keep_client_->async_send_request(req);
    std::future_status status = future_result.wait_for(timeout);
    if (status == std::future_status::ready) {
      state_keep_response = future_result.get()->success;
      state_keep_code = future_result.get()->code;
      INFO("success to call state_switch_keep services.Code:%d", state_keep_code);
      return state_keep_response;
    } else {
      INFO("Failed to call machine_state_switch_keep services.");
      return false;
    }
  }
  void UnlockMachineRequest(
    const protocol::srv::Unlock::Request::SharedPtr request,
    protocol::srv::Unlock::Response::SharedPtr response)
  {
    INFO("enter unlock_request service");
    RequestMotion();
    if (RequestStateKeep()) {
      INFO("machine_state_switch_keep success");
    } else {
      INFO("machine_state_switch_keep failed");
      response->unlock_result = 100;
      response->code = 9100;
      return;
    }
    black_box.CreateUnlockStatusDB();              // 创建或复原上报状态记录文件
    INFO("request httplink is %s", request->httplink.c_str());
    std::string httpCom = "https://cnbj2m.fds.api.xiaomi.com";
    std::string httpFilePath = request->httplink.substr(httpCom.length());
    INFO("http link path is %s", httpFilePath.c_str());
    httplib::Client cli(httpCom);
    cli.set_read_timeout(5, 0);
    auto res = cli.Get(httpFilePath);
    if (res->status != 200) {
      INFO("Get key file error");
      response->unlock_result = 100;
      response->code = 9100;
      return;
    }
    if (res->body.length() == 0) {
      INFO("the key file is empty");
      response->unlock_result = 100;
      response->code = 9100;
      return;
    }
    std::fstream f;
    f.open("/SDCARD/pub_key.tar", std::ios::out);
    f << res->body << std::endl;
    f.close();
    GetKeyFile();
    // INFO("key path is %s", keyPath.c_str());
    int unlock_access_result = UnlockAccess();
    int unlock_status_result = GetUnlockStatus();
    INFO("UnlockAccess() = %d", unlock_access_result);
    INFO("GetUnlockStatus() = %d", unlock_access_result);
    if ( (unlock_access_result == 0) && (unlock_status_result == 0) ) {
      INFO("unlock success!!!");
      response->unlock_result = 0;
      response->code = 9000;
    } else {
      INFO("unlock faild!!!");
      response->unlock_result = 100;
      response->code = 9100;
    }
  }

private:
  rclcpp::Subscription<protocol::msg::SelfCheckStatus>::SharedPtr self_check_sub;
  rclcpp::Service<protocol::srv::Unlock>::SharedPtr unlock_requset_srv_;
  rclcpp::Service<protocol::srv::RebootMachine>::SharedPtr reboot_machine_srv_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_;
  rclcpp::Client<protocol::srv::BesHttp>::SharedPtr bes_http_client_;
  rclcpp::Client<protocol::srv::MachineState>::SharedPtr machine_state_switch_keep_client_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::CallbackGroup::SharedPtr unlock_callback_group_;
  std::unique_ptr<cyberdog::manager::LowPowerConsumption> lpc_ptr_{nullptr};
  cyberdog::manager::BlackBox black_box;
  bool state_keep_response = false;
  bool flag = true;
  int upsend_counter = 0;
  int state_keep_code;
};
}  // namespace manager
}  // namespace cyberdog
#endif  // UNLOCK_REQUEST__UNLOCK_REQUEST_HPP_
