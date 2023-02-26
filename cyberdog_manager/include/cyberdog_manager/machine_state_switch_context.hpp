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
#ifndef CYBERDOG_MANAGER__MACHINE_STATE_SWITCH_CONTEXT_HPP_
#define CYBERDOG_MANAGER__MACHINE_STATE_SWITCH_CONTEXT_HPP_

#include <string>
#include <memory>
#include <map>
#include <mutex>
#include <vector>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cyberdog_machine/cyberdog_fs_machine.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/msg/state_switch_status.hpp"
#include "protocol/srv/machine_state.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{
enum class MsscMachineState : uint8_t
{
  MSSC_ACTIVE    = 0,    // 正常
  MSSC_PROTECT   = 1,    // 保护
  MSSC_LOWPOWER  = 2,    // 低功耗
  MSSC_OTA       = 3,    // OTA
  MSSC_SHUTDOWN  = 4,    // 关机
  MSSC_SELFCHECK = 5,    // 自检
  MSSC_DEACTIVE  = 6,    // 待机
  MSSC_ERROR     = 7,    // 错误
  MSSC_UNKOWN    = 255,  // 未知
};
enum class MachineStateChild : uint8_t
{
  MSC_ACTIVE      = 0,
  MSC_PROTECTED   = 1,
  MSC_LOWPOWER    = 2,
  MSC_OTA         = 3,
  MSC_TEARDOWN    = 4,
  MSC_SELFCHECK   = 5,
  MSC_DEACTIVE    = 6,
  MSC_ERROR       = 7,
};

class StateContext final
{
public:
  explicit StateContext(const std::string & node_name)
  : name_(node_name)
  {
    node_ptr_ = rclcpp::Node::make_shared(name_);
    machine_context_ptr_ = std::make_unique<cyberdog::machine::MachineContext>();
    current_state_ = machine_context_ptr_->Context(
      cyberdog::machine::MachineState::MS_Uninitialized);
    machine_controller_ptr_ = std::make_shared<cyberdog::machine::MachineController>();
    std::thread(
      [this]() {
        rclcpp::spin(node_ptr_);
      }).detach();
  }

  ~StateContext() {}

  bool Init()
  {
    auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
    auto path = local_share_dir + std::string("/toml_config/manager/state_machine_config.toml");
    if (!machine_controller_ptr_->MachineControllerInit(
        path, node_ptr_))
    {
      ERROR("[MachineState-Context]: Machine State Init failed!");
      return false;
    } else if (!machine_controller_ptr_->WaitActuatorsSetUp()) {
      ERROR("[MachineState-Context]: Machine State Init failed, actuators setup failed!");
      return false;
    } else {
      INFO("[MachineState-Context]: Machine State Init OK.");
      return true;
    }
  }

  int32_t SetState(cyberdog::machine::MachineState ms)
  {
    int32_t result = 0;
    current_state_ = machine_context_ptr_->Context(ms);
    result = machine_controller_ptr_->SetState(current_state_);
    if (result != 0) {
      ERROR("[MachineState-Context]: set state:%s failed, cannot running!", current_state_.c_str());
    }
    return result;
  }

  const std::vector<std::string> & GetAchieveStates()
  {
    return machine_controller_ptr_->GetNeedAchieveStates();
  }

private:
  std::string name_;
  rclcpp::Node::SharedPtr node_ptr_{nullptr};
  std::shared_ptr<cyberdog::machine::MachineController> machine_controller_ptr_ {nullptr};
  std::string current_state_;
  std::unique_ptr<cyberdog::machine::MachineContext> machine_context_ptr_ {nullptr};
};

class MachineStateSwitchContext final
{
  using BSSC_CALLBACK = std::function<void ()>;
  using MACHINE_STATE_CALLBACK = std::function<void ()>;
  using LOWPOWER_ENTERANDEXIT_CALLBACK = std::function<bool (bool )>;
  using EXCEPTION_PLAYSOUND_CALLBACK = std::function<void (int32_t )>;

public:
  explicit MachineStateSwitchContext(rclcpp::Node::SharedPtr node_ptr)
  : mssc_node_(node_ptr)
  {
    Config();
    machine_state_ptr_ = std::make_unique<cyberdog::manager::StateContext>("contex_machine");
    mssc_callback_group_ =
      mssc_node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = mssc_callback_group_;
    state_set_sub_ = mssc_node_->create_subscription<std_msgs::msg::UInt8>(
      "machine_state_switch", rclcpp::SystemDefaultsQoS(),
      std::bind(&MachineStateSwitchContext::MachineStateSwitch, this, std::placeholders::_1),
      sub_options);
    machine_state_switch_keep_srv_ =
      mssc_node_->create_service<protocol::srv::MachineState>(
      "machine_state_switch_keep",
      std::bind(
        &MachineStateSwitchContext::MachineStateSwitchKeep, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    state_valget_srv_ =
      mssc_node_->create_service<std_srvs::srv::Trigger>(
      "machine_state_valget",
      std::bind(
        &MachineStateSwitchContext::MachineStateGet, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    switch_ota_state_srv_ =
      mssc_node_->create_service<std_srvs::srv::SetBool>(
      "ota_state_switch",
      std::bind(
        &MachineStateSwitchContext::OtaMachineState, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    low_power_enable_srv_ =
      mssc_node_->create_service<std_srvs::srv::SetBool>(
      "low_power_enable",
      std::bind(
        &MachineStateSwitchContext::LowPowerEnable, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    low_power_exit_srv_ =
      mssc_node_->create_service<std_srvs::srv::Trigger>(
      "low_power_exit",
      std::bind(
        &MachineStateSwitchContext::LowPowerExit, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    low_power_onoff_srv_ =
      mssc_node_->create_service<std_srvs::srv::SetBool>(
      "low_power_onoff",
      std::bind(
        &MachineStateSwitchContext::LowPowerOnoff, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    low_power_switch_state_srv_ =
      mssc_node_->create_service<std_srvs::srv::Trigger>(
      "low_power_switch_state",
      std::bind(
        &MachineStateSwitchContext::LowPowerSwitchState, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, mssc_callback_group_);
    power_off_client_ =
      mssc_node_->create_client<std_srvs::srv::Trigger>(
      "poweroff",
      rmw_qos_profile_services_default, mssc_callback_group_);
    low_power_client_ =
      mssc_node_->create_client<std_srvs::srv::SetBool>(
      "low_power_consumption",
      rmw_qos_profile_services_default, mssc_callback_group_);
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = mssc_callback_group_;
    state_swith_status_pub_ =
      mssc_node_->create_publisher<protocol::msg::StateSwitchStatus>(
      "state_switch_status", rclcpp::SystemDefaultsQoS(), pub_options);
    timer_callback_group_ =
      mssc_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  }
  void ExecuteSetUp()
  {
    if (!machine_state_ptr_->Init()) {
      ERROR("[MachineState-Switch]: >>>XXXXX---machine state init error!");
    }
    SetStateHandler(machine_state_ptr_->GetAchieveStates());
  }
  bool ExecuteSelfCheck()
  {
    return SetState(cyberdog::machine::MachineState::MS_SelfCheck);
  }
  bool ExecuteActive()
  {
    return SetState(cyberdog::machine::MachineState::MS_Active);
  }
  void Init()
  {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = mssc_callback_group_;
    wake_up_sub_ =
      mssc_node_->create_subscription<std_msgs::msg::Bool>(
      "dog_wakeup", rclcpp::SystemDefaultsQoS(),
      std::bind(&MachineStateSwitchContext::DogWakeup, this, std::placeholders::_1),
      sub_options);
    mssc_machine_state = MsscMachineState::MSSC_ACTIVE;
  }
  void SetStateHandler(const std::vector<std::string> & state_vec)
  {
    for (auto & elem : state_vec) {
      if (elem == "SelfCheck") {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_SELFCHECK] =
          std::bind(&MachineStateSwitchContext::OnSelfCheck, this);
      } else if (elem == "TearDown") {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_TEARDOWN] =
          std::bind(&MachineStateSwitchContext::OnTearDown, this);
      } else if (elem == "Active") {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_ACTIVE] =
          std::bind(&MachineStateSwitchContext::OnActive, this);
      } else if (elem == "DeActive") {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_DEACTIVE] =
          std::bind(&MachineStateSwitchContext::OnDeactive, this);
      } else if (elem == "Protected") {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_PROTECTED] =
          std::bind(&MachineStateSwitchContext::OnProtected, this);
      } else if (elem == "LowPower" && !disable_lowpower_) {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_LOWPOWER] =
          std::bind(&MachineStateSwitchContext::OnLowPower, this);
      } else if (elem == "OTA") {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_OTA] =
          std::bind(&MachineStateSwitchContext::OnOta, this);
      } else if (elem == "Error") {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_ERROR] =
          std::bind(&MachineStateSwitchContext::OnError, this);
      }
    }
  }
  void SetLowpowerEnterAndExitCallback(LOWPOWER_ENTERANDEXIT_CALLBACK callback)
  {
    lowpower = callback;
  }
  void SetExceptionPlaySoundCallback(EXCEPTION_PLAYSOUND_CALLBACK callback)
  {
    play_sound = callback;
  }
  void DogWakeup(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (machine_state_keep) {
      return;
    }
    if (msg->data && mssc_machine_state != MsscMachineState::MSSC_LOWPOWER) {
      return;
    }
    if (is_switching_ms_) {
      INFO("[MachineState-Switch]: wakeup, now in switching machine state");
      return;
    }
    INFO("[MachineState-Switch]: dog wakeup...");
    if (battery_charge_val < 5 && !disable_lowpower_) {
      INFO("[MachineState-Switch]: rejected switch, battery soc less than 5 and keep low-power");
    } else if (battery_charge_val < 20) {
      std::lock_guard<std::mutex> lck(state_mtx_);
      machine_state_handler_map[MachineStateChild::MSC_PROTECTED]();
    } else {
      std::lock_guard<std::mutex> lck(state_mtx_);
      machine_state_handler_map[MachineStateChild::MSC_ACTIVE]();
    }
  }
  void BatteryChargeUpdate(uint8_t bc, bool is_charging)
  {
    if (machine_state_keep) {
      return;
    }
    battery_charge_val = bc;
    is_charging_ = is_charging;
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return;
    } else if (battery_charge_val <= 0 && (!is_charging)) {
      // 关机
      std::lock_guard<std::mutex> lck(state_mtx_);
      machine_state_handler_map[MachineStateChild::MSC_TEARDOWN]();
    } else if (battery_charge_val < 5 && !disable_lowpower_) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER || is_charging) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_PROTECT) {
        // 切换到低功耗模式
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_LOWPOWER]();
      } else if (mssc_machine_state == MsscMachineState::MSSC_ACTIVE) {
        // 切换到低功耗模式
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_LOWPOWER]();
      }
    } else if (battery_charge_val < 20) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_PROTECT) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_ACTIVE) {
        // 切换到保护模式
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_PROTECTED]();
      }
    } else {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        return;
      } else if (mssc_machine_state == MsscMachineState::MSSC_PROTECT) {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_ACTIVE]();
      } else if (mssc_machine_state == MsscMachineState::MSSC_ACTIVE) {
        return;
      }
    }
  }
  void KeepDownOverTime()
  {
    if (machine_state_keep) {
      return;
    }
    std::lock_guard<std::mutex> lck(state_mtx_);
    machine_state_handler_map[MachineStateChild::MSC_LOWPOWER]();
  }

private:
  void Config()
  {
    auto path = CONFIG_DIR + "/" + CONFIG_FILE;
    Document json_document(kObjectType);
    auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
    if (result) {
      rapidjson::Value lowpower_val;
      bool result = CyberdogJson::Get(json_document, "lowpower_info", lowpower_val);
      if (result) {
        if (lowpower_val.HasMember("switch")) {
          disable_lowpower_ = !lowpower_val["switch"].GetBool();
        } else {
          disable_lowpower_ = true;
        }
      } else {
        disable_lowpower_ = true;
      }
    } else {
      disable_lowpower_ = true;
    }
  }
  void MachineStateSwitch(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    SwitchState(static_cast<MsscMachineState>(msg->data));
  }
  void MachineStateSwitchKeep(
    const protocol::srv::MachineState::Request::SharedPtr request,
    protocol::srv::MachineState::Response::SharedPtr response)
  {
    if (!machine_state_keep) {
      if (request->ticks > 0) {
        {
          std::lock_guard<std::mutex> lck(state_mtx_);
          machine_state_handler_map[static_cast<MachineStateChild>(request->state)]();
        }
        machine_state_keep = true;
        INFO("[MachineState-Switch]: keep machine state to start");
        keep_timer_ = mssc_node_->create_wall_timer(
          std::chrono::seconds(request->ticks),
          std::bind(&MachineStateSwitchContext::KeepTimerCallback, this), timer_callback_group_);
      } else {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[static_cast<MachineStateChild>(request->state)]();
      }
      response->success = true;
      response->code = 0;
    } else {
      response->success = true;
      response->code = -1;
    }
  }
  void KeepTimerCallback()
  {
    keep_timer_->cancel();
    machine_state_keep = false;
    INFO("[MachineState-Switch]: keep machine state to stop");
  }
  void MachineStateGet(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    // SwitchState(static_cast<MsscMachineState>(msg->data));
    auto iter = mssc_machine_state_code_map.find(mssc_machine_state);
    if (iter == mssc_machine_state_code_map.end()) {
      response->message = "error-state";
    } else {
      response->message = mssc_machine_state_code_map.at(mssc_machine_state);
    }
    response->success = true;
  }
  void OtaMachineState(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (request->data) {
      if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
        lowpower(false);
        INFO("[MachineState-LowPower]: exit lowpower to ota");
      } else if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
        response->success = true;
        return;
      }
      SwitchState(MsscMachineState::MSSC_OTA);
    } else {
      SwitchState(MsscMachineState::MSSC_ACTIVE);
    }
    response->success = true;
  }
  void LowPowerEnable(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lck(state_mtx_);
    if (request->data) {
      machine_state_handler_map[MachineStateChild::MSC_LOWPOWER] =
        std::bind(&MachineStateSwitchContext::OnLowPower, this);
    } else {
      machine_state_handler_map[MachineStateChild::MSC_LOWPOWER] =
        []() {};
      machine_state_handler_map[MachineStateChild::MSC_ACTIVE]();
    }
    response->success = true;
  }
  void LowPowerExit(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lck(state_mtx_);
    machine_state_handler_map[MachineStateChild::MSC_ACTIVE]();
    response->success = true;
  }
  void LowPowerOnoff(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (disable_lowpower_ == request->data) {
      disable_lowpower_ = (!request->data);
      if (disable_lowpower_) {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_LOWPOWER] =
          []() {};
        machine_state_handler_map[MachineStateChild::MSC_ACTIVE]();
      } else {
        std::lock_guard<std::mutex> lck(state_mtx_);
        machine_state_handler_map[MachineStateChild::MSC_LOWPOWER] =
          std::bind(&MachineStateSwitchContext::OnLowPower, this);
      }
      auto path = CONFIG_DIR + "/" + CONFIG_FILE;
      if (access(CONFIG_DIR.c_str(), F_OK) != 0) {
        std::string cmd = "mkdir -p " + CONFIG_DIR;
        std::system(cmd.c_str());
        cmd = "chmod 777 " + CONFIG_DIR;
        std::system(cmd.c_str());
      }
      Document json_document(kObjectType);
      Document::AllocatorType & allocator = json_document.GetAllocator();
      auto result = CyberdogJson::ReadJsonFromFile(path, json_document);
      rapidjson::Value lowpower_val(rapidjson::kObjectType);
      if (result) {
        result = CyberdogJson::Get(json_document, "lowpower_info", lowpower_val);
        if (result) {
          if (lowpower_val.HasMember("switch")) {
            lowpower_val["switch"].SetBool(!disable_lowpower_);
          } else {
            lowpower_val.AddMember("switch", !disable_lowpower_, allocator);
          }
        }
      } else {
        INFO("[MachineState-LowPower]: no file:%s, create it!", path.c_str());
        lowpower_val.AddMember("switch", request->data, allocator);
      }
      CyberdogJson::Add(json_document, "lowpower_info", lowpower_val);
      CyberdogJson::WriteJsonToFile(path, json_document);
    }
    response->success = true;
  }
  void LowPowerSwitchState(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    response->success = disable_lowpower_ ? false : true;
    response->message = disable_lowpower_ ? "off" : "on";
  }
  bool SwitchState(MsscMachineState mm)
  {
    std::lock_guard<std::mutex> lck(switch_mtx);
    is_switching_ms_ = true;
    bool result = false;
    switch (mm) {
      case MsscMachineState::MSSC_ACTIVE:
        {
          INFO("[MachineState-Switch]: ^^^ switch state:active ^^^");
          result = SetState(cyberdog::machine::MachineState::MS_Active);
          // active_handler();
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 0;
          state_swith_status_pub_->publish(sss);
        }
        break;
      case MsscMachineState::MSSC_PROTECT:
        {
          INFO("[MachineState-Switch]: ^^^ switch state:protected ^^^");
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 1;
          state_swith_status_pub_->publish(sss);
          // protect_handler();
          result = SetState(cyberdog::machine::MachineState::MS_Protected);
        }
        break;
      case MsscMachineState::MSSC_LOWPOWER:
        {
          INFO("[MachineState-Switch]: ^^^ switch state:low-power ^^^");
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 2;
          state_swith_status_pub_->publish(sss);
          // lowpower_handler();
          result = SetState(cyberdog::machine::MachineState::MS_LowPower);
          if (result) {
            result = lowpower(true);
          }
        }
        break;
      case MsscMachineState::MSSC_OTA:
        {
          INFO("[MachineState-Switch]: ^^^ switch state:ota ^^^");
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 3;
          state_swith_status_pub_->publish(sss);
          // ota_handler();
          result = SetState(cyberdog::machine::MachineState::MS_OTA);
        }
        break;
      case MsscMachineState::MSSC_SHUTDOWN:
        {
          INFO("[MachineState-Switch]: ^^^ switch state:shutdown ^^^");
          // need modify
          protocol::msg::StateSwitchStatus sss;
          sss.code = 0;
          sss.state = 4;
          state_swith_status_pub_->publish(sss);
          result = SetState(cyberdog::machine::MachineState::MS_TearDown);
          if (result) {
            poweroff();
          }
          // shutdown_handler();
        }
        break;
      default:
        {
          INFO("[MachineState-Switch]: ^^^ switch state:unkown ^^^");
        }
        break;
    }
    if (result) {
      mssc_machine_state = mm;
    }
    is_switching_ms_ = false;
    return result;
  }

  void poweroff()
  {
    INFO("cyberdog start poweroff when the battery soc is 0.");
    if (!power_off_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call poweroff service not avalible");
    } else {
      std::chrono::seconds timeout(3);
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future_result = power_off_client_->async_send_request(req);
      std::future_status status = future_result.wait_for(timeout);
      if (status == std::future_status::ready) {
        INFO("call poweroff service success.");
      } else {
        ERROR("call poweroff service failed!");
      }
    }
  }

  bool OnSelfCheck()
  {
    bool result = true;
    if (is_switching_ms_) {
      INFO("[MachineState-SelfCheck]: rejected, now in switching machine state");
      return false;
    }
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return false;
    } else if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
      result = lowpower(false);
    }
    return result ? SwitchState(MsscMachineState::MSSC_SELFCHECK) : result;
  }

  bool OnLowPower()
  {
    if (is_switching_ms_) {
      INFO("[MachineState-LowPower]: rejected, now in switching machine state");
      return false;
    }
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return false;
    } else if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
      return true;
    }
    return SwitchState(MsscMachineState::MSSC_LOWPOWER);
  }

  bool OnProtected()
  {
    bool result = true;
    if (is_switching_ms_) {
      INFO("[MachineState-Protected]: rejected, now in switching machine state");
      return false;
    }
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return false;
    } else if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
      result = lowpower(false);
    }
    return result ? SwitchState(MsscMachineState::MSSC_PROTECT) : result;
  }

  bool OnActive()
  {
    bool result = true;
    if (is_switching_ms_) {
      INFO("[MachineState-Active]: rejected, now in switching machine state");
      return false;
    }
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return false;
    } else if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
      result = lowpower(false);
    }
    return result ? SwitchState(MsscMachineState::MSSC_ACTIVE) : result;
  }

  bool OnDeactive()
  {
    bool result = true;
    if (is_switching_ms_) {
      INFO("[MachineState-Active]: rejected, now in switching machine state");
      return false;
    }
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return false;
    } else if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
      result = lowpower(false);
    }
    return result ? SwitchState(MsscMachineState::MSSC_DEACTIVE) : result;
  }

  bool OnTearDown()
  {
    bool result = true;
    if (is_switching_ms_) {
      INFO("[MachineState-ShutDown]: rejected, now in switching machine state");
      return false;
    }
    if (mssc_machine_state == MsscMachineState::MSSC_OTA) {
      return false;
    } else if (battery_charge_val <= 0 && (!is_charging_)) {
      // 关机
      result = SwitchState(MsscMachineState::MSSC_SHUTDOWN);
    }
    result = SwitchState(MsscMachineState::MSSC_SHUTDOWN);
    return result;
  }

  bool OnOta()
  {
    bool result = true;
    if (is_switching_ms_) {
      INFO("[MachineState-Ota]: rejected, now switching machine state");
      return false;
    }
    if (mssc_machine_state == MsscMachineState::MSSC_LOWPOWER) {
      result = lowpower(false);
    }
    return result ? SwitchState(MsscMachineState::MSSC_OTA) : result;
  }

  bool OnError()
  {
    return SwitchState(MsscMachineState::MSSC_ERROR);
  }

  bool SetState(cyberdog::machine::MachineState ms)
  {
    int32_t code = machine_state_ptr_->SetState(ms);
    if (code != 0) {
      play_sound(code);
    }
    return code == 0;
  }

private:
  rclcpp::Node::SharedPtr mssc_node_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr mssc_callback_group_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_set_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_valget_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_ota_state_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_power_enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr low_power_exit_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr low_power_onoff_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr low_power_switch_state_srv_;
  rclcpp::Service<protocol::srv::MachineState>::SharedPtr machine_state_switch_keep_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_off_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr low_power_client_;
  rclcpp::Publisher<protocol::msg::StateSwitchStatus>::SharedPtr state_swith_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wake_up_sub_ {nullptr};
  MsscMachineState mssc_machine_state {MsscMachineState::MSSC_UNKOWN};
  std::mutex switch_mtx;
  uint8_t battery_charge_val {100};
  bool is_charging_ {false};
  const std::map<MsscMachineState, std::string> mssc_machine_state_code_map = {
    {MsscMachineState::MSSC_ACTIVE, "active"},
    {MsscMachineState::MSSC_PROTECT, "protected"},
    {MsscMachineState::MSSC_LOWPOWER, "lowpower"},
    {MsscMachineState::MSSC_OTA, "ota"},
    {MsscMachineState::MSSC_SHUTDOWN, "shutdown"},
    {MsscMachineState::MSSC_SELFCHECK, "selfcheck"},
    {MsscMachineState::MSSC_UNKOWN, "unkown"}
  };
  bool is_switching_ms_ {false};
  std::mutex state_mtx_;
  std::map<MachineStateChild, MACHINE_STATE_CALLBACK> machine_state_handler_map = {
    {MachineStateChild::MSC_SELFCHECK, []() {}},
    {MachineStateChild::MSC_ACTIVE, []() {}},
    {MachineStateChild::MSC_DEACTIVE, []() {}},
    {MachineStateChild::MSC_PROTECTED, []() {}},
    {MachineStateChild::MSC_LOWPOWER, []() {}},
    {MachineStateChild::MSC_OTA, []() {}},
    {MachineStateChild::MSC_TEARDOWN, []() {}},
    {MachineStateChild::MSC_ERROR, []() {}},
  };
  bool disable_lowpower_ {false};
  LOWPOWER_ENTERANDEXIT_CALLBACK lowpower {[](bool) {return true;}};
  EXCEPTION_PLAYSOUND_CALLBACK play_sound {[](int32_t) {}};
  std::unique_ptr<cyberdog::manager::StateContext> machine_state_ptr_ {nullptr};
  bool machine_state_keep {false};
  rclcpp::TimerBase::SharedPtr keep_timer_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  const std::string CONFIG_DIR = "/home/mi/.cyberdog/manager";
  const std::string CONFIG_FILE = "config.json";
};
}  // namespace manager
}  // namespace cyberdog

#endif  // CYBERDOG_MANAGER__MACHINE_STATE_SWITCH_CONTEXT_HPP_
