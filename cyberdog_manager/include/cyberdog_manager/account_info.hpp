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
#ifndef CYBERDOG_MANAGER__ACCOUNT_INFO_HPP_
#define CYBERDOG_MANAGER__ACCOUNT_INFO_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_json.hpp"
#include "protocol/srv/account_add.hpp"
#include "protocol/srv/account_search.hpp"
#include "protocol/srv/account_delete.hpp"
#include "protocol/srv/account_change.hpp"
#include "protocol/srv/all_user_search.hpp"
#include "protocol/srv/audio_voiceprint_entry.hpp"
#include "protocol/srv/face_entry.hpp"
#include "user_info_manager/UserAccountManager.hpp"

using cyberdog::common::CyberdogJson;
using rapidjson::Document;
using rapidjson::kObjectType;

namespace cyberdog
{
namespace manager
{

enum class AccountErrorCode : int32_t
{
  kAccountSearchError = 21,
  kAccountDatabaseError = 22,
  kAccounVoiceprintError = 23,
  kAccountFaceError = 24,
};

class AccountInfoNode final
{
public:
  explicit AccountInfoNode(rclcpp::Node::SharedPtr node_ptr)
  {
    code_ptr_ = std::make_shared<cyberdog::system::CyberdogCode<AccountErrorCode>>(
      cyberdog::system::ModuleCode::kRobot);
    account_info_node_ = node_ptr;
    account_callback_group_ =
      account_info_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    account_add_srv_ =
      account_info_node_->create_service<protocol::srv::AccountAdd>(
      "account_add",
      std::bind(
        &AccountInfoNode::QueryAccountAdd, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, account_callback_group_);
    account_search_srv_ =
      account_info_node_->create_service<protocol::srv::AccountSearch>(
      "account_search",
      std::bind(
        &AccountInfoNode::QueryAccountSearch, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, account_callback_group_);
    account_delete_srv_ =
      account_info_node_->create_service<protocol::srv::AccountDelete>(
      "account_delete",
      std::bind(
        &AccountInfoNode::QueryAccountDelete, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, account_callback_group_);
    account_search_all_ =
      account_info_node_->create_service<protocol::srv::AllUserSearch>(
      "all_user_search",
      std::bind(
        &AccountInfoNode::AllUserSearch, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, account_callback_group_);
    account_change_srv_ =
      account_info_node_->create_service<protocol::srv::AccountChange>(
      "account_change",
      std::bind(
        &AccountInfoNode::ChangeUserName, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, account_callback_group_);
  }

private:
  void AllUserSearch(
    const protocol::srv::AllUserSearch::Request::SharedPtr request,
    protocol::srv::AllUserSearch::Response::SharedPtr response)
  {
    INFO("enter search all user service  callback");
    cyberdog::common::CyberdogAccountManager obj;
    std::vector<cyberdog::manager::MemberInformaion> member_data;
    int result[2];
    if (request->command == "") {
      INFO("search all user");
      if (obj.SearAllUser(member_data)) {
        response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
      } else {
        response->code = code_ptr_->GetCode(AccountErrorCode::kAccountSearchError);
        return;
      }
      int len = member_data.size();
      response->result.resize(len);
      for (int i = 0; i < len; i++) {
        response->result[i].id = member_data[i].id;
        response->result[i].username = member_data[i].name;
        response->result[i].voicestatus = member_data[i].voiceStatus;
        response->result[i].facestatus = member_data[i].faceStatus;
      }
      INFO("response->result length is %d", response->result.size());
    } else {
      INFO("search one user");
      std::string username = request->command;
      response->result.resize(1);
      if (obj.SearchUser(username, result)) {
        // INFO(
        //   "search all service (serach one user : name:%s, %d, %d)",
        //   username.c_str(), result[0], result[1]);
        response->result[0].username = username;
        response->result[0].voicestatus = result[0];
        response->result[0].facestatus = result[1];
        response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
      } else {
        response->code = code_ptr_->GetCode(AccountErrorCode::kAccountSearchError);
        return;
      }
    }
  }

  void QueryAccountAdd(
    const protocol::srv::AccountAdd::Request::SharedPtr request,
    protocol::srv::AccountAdd::Response::SharedPtr response)
  {
    std::string name = request->member;
    INFO("add_account_name is: %s", name.c_str());
    cyberdog::common::CyberdogAccountManager obj;
    if (obj.AddMember(name)) {
      response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
      INFO("add_account_success");
    } else {
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccountDatabaseError);
      INFO("add_account_fail");
    }
  }

  void QueryAccountSearch(
    const protocol::srv::AccountSearch::Request::SharedPtr request,
    protocol::srv::AccountSearch::Response::SharedPtr response)
  {
    std::string account_name = request->member;
    INFO("search user_name: %s", account_name.c_str());
    Document json_info(rapidjson::kObjectType);
    rapidjson::Value arr(rapidjson::kArrayType);
    std::string data;
    cyberdog::common::CyberdogAccountManager obj;
    if (account_name.empty()) {
      INFO("search_all_user");
      std::vector<cyberdog::manager::MemberInformaion> member_data;
      if (obj.SearAllUser(member_data)) {
        for (unsigned int i = 0; i < member_data.size(); ++i) {
          rapidjson::Value js_obj(rapidjson::kObjectType);
          rapidjson::Value value(member_data[i].name.c_str(), json_info.GetAllocator());
          js_obj.AddMember("id", member_data[i].id, json_info.GetAllocator());
          js_obj.AddMember("name", value, json_info.GetAllocator());
          js_obj.AddMember("face_state", member_data[i].faceStatus, json_info.GetAllocator());
          js_obj.AddMember("voice_state", member_data[i].voiceStatus, json_info.GetAllocator());
          arr.PushBack(js_obj, json_info.GetAllocator());
        }
        json_info.AddMember("accounts", arr, json_info.GetAllocator());
        response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
      } else {
        INFO("Search ALL Account faile!");
        response->code = code_ptr_->GetCode(AccountErrorCode::kAccountSearchError);
        response->data = "{\"code\":121}";
      }
    } else {
      INFO("search_one_user");
      int result[2]{-1, -1};
      rapidjson::Value js_obj(rapidjson::kObjectType);
      if (!obj.SearchUser(account_name, result)) {
        INFO("search %s faild, SearchUser() return 0", account_name.c_str());
        response->code = code_ptr_->GetCode(AccountErrorCode::kAccountSearchError);
        response->data = "{\"code\":121}";
        return;
      }
      int voice_state = result[0];
      int face_state = result[1];
      INFO(
        "account name:%s \n voice_state: %d \n face_stare: %d",
        account_name.c_str(), result[0], result[1]);
      json_info.AddMember("code", 0, json_info.GetAllocator());
      rapidjson::Value value(account_name.c_str(), json_info.GetAllocator());
      js_obj.AddMember("name", value, json_info.GetAllocator());
      js_obj.AddMember("face_state", face_state, json_info.GetAllocator());
      js_obj.AddMember("voice_state", voice_state, json_info.GetAllocator());
      arr.PushBack(js_obj, json_info.GetAllocator());
      json_info.AddMember("accounts", arr, json_info.GetAllocator());
      response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
    }

    if (!CyberdogJson::Document2String(json_info, data)) {
      ERROR("error while encoding to json");
      data = "{\"error\": \"unkown encoding json error!\"}";
    }
    response->data = data;
    INFO("Search result is :%s", data.c_str());
  }
  void QueryAccountDelete(
    const protocol::srv::AccountDelete::Request::SharedPtr request,
    protocol::srv::AccountDelete::Response::SharedPtr response)
  {
    INFO("enter delete account callback");
    std::chrono::seconds timeout(3);
    // voice delete
    rclcpp::Client<protocol::srv::AudioVoiceprintEntry>::SharedPtr voice_delete_client_;
    voice_delete_client_ =
      account_info_node_->create_client<protocol::srv::AudioVoiceprintEntry>(
      "audio_voiceprint_entry");
    if (!voice_delete_client_->wait_for_service(std::chrono::seconds(2))) {
      WARN("call voice service server not avalible");
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccounVoiceprintError);
      return;
    }
    auto request_voice = std::make_shared<protocol::srv::AudioVoiceprintEntry::Request>();
    request_voice->command = 4;
    request_voice->username = request->member;
    auto future_result_voice = voice_delete_client_->async_send_request(request_voice);
    std::future_status status_voice = future_result_voice.wait_for(timeout);
    if (status_voice == std::future_status::ready) {
      INFO(
        "success to call voice delete response services.");
    } else {
      INFO(
        "Failed to call voice delete response services.");
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccounVoiceprintError);
      return;
    }

    // face delete
    rclcpp::Client<protocol::srv::FaceEntry>::SharedPtr face_delete_client_;
    face_delete_client_ =
      account_info_node_->create_client<protocol::srv::FaceEntry>("cyberdog_face_entry_srv");
    if (!face_delete_client_->wait_for_service(std::chrono::seconds(2))) {
      ERROR("call face service server not avalible");
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccountFaceError);
      return;
    }
    auto request_face = std::make_shared<protocol::srv::FaceEntry::Request>();
    request_face->command = 4;
    request_face->username = request->member;
    auto future_result_face = face_delete_client_->async_send_request(request_face);
    std::future_status status_face = future_result_face.wait_for(timeout);
    if (status_face != std::future_status::ready) {
      WARN("Failed to call face delete response services.");
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccountFaceError);
      return;
    }
    // delete account
    std::string name = request->member;
    INFO("delete_account_name is: %s", name.c_str());
    if (future_result_voice.get()->success) {
      if (future_result_face.get()->result == 0) {
        cyberdog::common::CyberdogAccountManager obj;
        if (obj.DeleteUserInformation(name)) {
          response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
        } else {
          WARN("delete account failed");
          response->code = code_ptr_->GetCode(AccountErrorCode::kAccountDatabaseError);
        }
      } else {
        WARN("delete face failed");
        response->code = code_ptr_->GetCode(AccountErrorCode::kAccountFaceError);
      }
    } else {
      WARN("delete voice failed");
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccounVoiceprintError);
    }
  }
  void ChangeUserName(
    const protocol::srv::AccountChange::Request::SharedPtr request,
    protocol::srv::AccountChange::Response::SharedPtr response)
  {
    INFO("enter change account callback");
    std::string pre_name = request->pre_name;
    std::string new_name = request->new_name;
    std::chrono::seconds timeout(3);

    int result[2]{-1, -1};
    cyberdog::common::CyberdogAccountManager obj;
    rapidjson::Value js_obj(rapidjson::kObjectType);
    if (!obj.SearchUser(pre_name, result)) {
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccountSearchError);
      return;
    }
    // face state is equal to result[1]
    if (result[1]) {
      rclcpp::Client<protocol::srv::FaceEntry>::SharedPtr face_delete_client_;
      face_delete_client_ =
        account_info_node_->create_client<protocol::srv::FaceEntry>("cyberdog_face_entry_srv");
      if (!face_delete_client_->wait_for_service(std::chrono::seconds(2))) {
        WARN("call face service server not avalible");
        response->code = code_ptr_->GetCode(AccountErrorCode::kAccountFaceError);
        return;
      }
      auto request_face = std::make_shared<protocol::srv::FaceEntry::Request>();
      request_face->command = 3;
      request_face->username = new_name;
      request_face->oriname = pre_name;
      auto future_result_face = face_delete_client_->async_send_request(request_face);
      std::future_status status_face = future_result_face.wait_for(timeout);
      if (status_face == std::future_status::ready) {
        if (future_result_face.get()->result != 0) {
          WARN("change user face name failed.");
          response->code = code_ptr_->GetCode(AccountErrorCode::kAccountFaceError);
          return;
        }
      } else {
        WARN("failed to call face services.");
        response->code = code_ptr_->GetCode(AccountErrorCode::kAccountFaceError);
        return;
      }
    }
    // change account
    INFO("change account name from %s to %s", pre_name.c_str(), new_name.c_str());
    if (obj.ModifyUserName(pre_name, new_name)) {
      response->code = code_ptr_->GetKeyCode(cyberdog::system::KeyCode::kOK);
    } else {
      WARN("change account nickname failed");
      response->code = code_ptr_->GetCode(AccountErrorCode::kAccountDatabaseError);
    }
  }

private:
  rclcpp::Node::SharedPtr account_info_node_{nullptr};
  std::shared_ptr<cyberdog::system::CyberdogCode<AccountErrorCode>> code_ptr_ {nullptr};
  rclcpp::CallbackGroup::SharedPtr account_callback_group_;
  rclcpp::Service<protocol::srv::AccountAdd>::SharedPtr account_add_srv_;
  rclcpp::Service<protocol::srv::AccountSearch>::SharedPtr account_search_srv_;
  rclcpp::Service<protocol::srv::AccountDelete>::SharedPtr account_delete_srv_;
  rclcpp::Service<protocol::srv::AllUserSearch>::SharedPtr account_search_all_;
  rclcpp::Service<protocol::srv::AccountChange>::SharedPtr account_change_srv_;
};

}  // namespace manager
}  // namespace cyberdog
#endif  // CYBERDOG_MANAGER__ACCOUNT_INFO_HPP_
