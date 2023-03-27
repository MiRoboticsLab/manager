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

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "user_info_manager/UserAccountManager.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "cyberdog_common/cyberdog_json.hpp"

// class TestSubscribe : public rclcpp::Node
// {
// public:
//   explicit TestSubscribe(std::string name)
//   : Node(name)
//   {
//     RCLCPP_INFO(this->get_logger(), "TEST NODE STARTED %s.", name.c_str());
//     // 订阅话题，增加用户
//     account_add_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_add", 10, std::bind(
//         &TestSubscribe::account_add_callback,
//         this, std::placeholders::_1));
//     // 订阅话题，搜索所有用户信息
//     account_search_all_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_search_all", 10, std::bind(
//         &TestSubscribe::account_search_all_callback,
//         this, std::placeholders::_1));
//     // 订阅话题，搜索单个用户信息
//     account_search_one_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_search_one", 10, std::bind(
//         &TestSubscribe::account_search_one_callback,
//         this, std::placeholders::_1));
//     // 订阅话题，删除一个用户
//     account_delete_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_delete", 10, std::bind(
//         &TestSubscribe::account_delete_callback,
//         this, std::placeholders::_1));
//     // 订阅话题，修改用户的voice或face状态(只能改一个)
//     account_modify_voice_or_face_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_modify_voice_or_face", 10, std::bind(
//         &TestSubscribe::
//         account_modify_voice_or_face_callback, this, std::placeholders::_1));

//     // 订阅话题，同时修改用户的voice和face状态
//     account_modify_voice_and_face_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_modify_voice_and_face", 10, std::bind(
//         &TestSubscribe::
//         account_modify_voice_and_face_callback, this, std::placeholders::_1));
//     // 订阅话题，将用户的voice状态置0
//     account_modify_voice_to0_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_modify_voice_to0", 10, std::bind(
//         &TestSubscribe::
//         account_modify_voice_to0_callback, this, std::placeholders::_1));

//     // 订阅话题，将用户的face状态置0
//     account_modify_face_to0_subscribe_ = this->create_subscription<std_msgs::msg::String>(
//       "account_modify_face_to0", 10, std::bind(
//         &TestSubscribe::account_modify_face_to0_callback,
//         this, std::placeholders::_1));
//   }

// private:
//   // 声明订阅者
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_add_subscribe_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_search_all_subscribe_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_search_one_subscribe_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_delete_subscribe_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_modify_voice_or_face_subscribe_;
// rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_modify_voice_and_face_subscribe_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_modify_voice_to0_subscribe_;
//   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr account_modify_face_to0_subscribe_;

//   // 回调函数，增加用户
//   void account_add_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     cyberdog::common::CyberdogAccountManager obj;
//     std::string name = msg->data;
//     INFO("account name %s", msg->data.c_str());

//     auto result = obj.AddMember(msg->data);
//     INFO("AddMember() return is %d", result);
//   }

//   // 回调函数，搜索所有用户
//   void account_search_all_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     INFO("search all accoiunt: %s", msg);
//     cyberdog::common::CyberdogAccountManager obj;
//     std::vector<cyberdog::common::CyberdogAccountManager::UserInformation> member_data;

//     auto result = obj.SearAllUser(member_data);
//     INFO("SearAllUser() return is %d", result);

//     rapidjson::Document json_info(rapidjson::kObjectType);
//     rapidjson::Value arr(rapidjson::kArrayType);
//     std::string data;

//     for (unsigned int i = 0; i < member_data.size(); ++i) {
//       rapidjson::Value js_obj(rapidjson::kObjectType);
//       rapidjson::Value value(member_data[i].username.c_str(), json_info.GetAllocator());
//       js_obj.AddMember("name", value, json_info.GetAllocator());
//       js_obj.AddMember("face_state", member_data[i].faceStatus, json_info.GetAllocator());
//       js_obj.AddMember("voice_state", member_data[i].voiceStatus, json_info.GetAllocator());
//       arr.PushBack(js_obj, json_info.GetAllocator());
//     }
//     json_info.AddMember("accounts", arr, json_info.GetAllocator());
//     cyberdog::common::CyberdogJson::Document2String(json_info, data);
//     INFO("All account info is: %s", data.c_str());
//   }

//   // 回调函数，搜索单一用户
//   void account_search_one_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     cyberdog::common::CyberdogAccountManager obj;
//     std::string name = msg->data;
//     INFO("account name is %s", msg->data.c_str());

//     int state[2];

//     auto result = obj.SearchUser(name, state);
//     INFO("SearchUser() return is %d", result);

// INFO("account name:%s,voice_state:%d,face_stare:%d", name.c_str(), state[0], state[1]);
//   }

//   // 回调函数，删除用户
//   void account_delete_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     cyberdog::common::CyberdogAccountManager obj;
//     std::string name = msg->data;
//     INFO("account name is %s", msg->data.c_str());

//     auto result = obj.DeleteUserInformation(name);
//     INFO("SearchOneUser() return is %d", result);
//   }

//   // 回调函数，修改用户voice或face的状态
//   void account_modify_voice_or_face_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     cyberdog::common::CyberdogAccountManager obj;
//     std::string name = msg->data;
//     int voice_state = 4;
//     int face_state = 8;
//     INFO("account name is %s", msg->data.c_str());
//     auto result = obj.ModifyUserInformation(name, voice_state, 0);
//     auto result2 = obj.ModifyUserInformation(name, face_state, 1);
//     INFO("ModifyUserInformation() modify voice return is %d", result);
//     INFO("ModifyUserInformation() modify face return is %d", result2);
//   }

//   // 回调函数，同时修改用户voice和face状态
//   void account_modify_voice_and_face_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     cyberdog::common::CyberdogAccountManager obj;
//     std::string name = msg->data;
//     int voice_state = 9;
//     int face_state = 6;
//     INFO("account name is %s", msg->data.c_str());
//     auto result = obj.ModifyInformation(msg->data, voice_state, face_state);
//     INFO("ModifyInformation() return is %d", result);
//   }

//   // 回调函数，将用户face状态置0
//   void account_modify_face_to0_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     cyberdog::common::CyberdogAccountManager obj;
//     std::string name = msg->data;
//     INFO("account name is %s", msg->data.c_str());
//     auto result = obj.DeleteFace(msg->data);
//     INFO("DeleteFace() return is %d", result);
//   }

//   // 回调函数，将用户voice状态置0
//   void account_modify_voice_to0_callback(const std_msgs::msg::String::SharedPtr msg)
//   {
//     cyberdog::common::CyberdogAccountManager obj;
//     INFO("account name is %s", msg->data.c_str());
//     auto result = obj.DeleteVoice(msg->data);
//     INFO("DeleteVoice() return is %d", result);
//   }
// };

int main(int argc, char ** argv)
{
  // rclcpp::init(argc, argv);
  // /*产生一个的节点*/
  // auto node = std::make_shared<TestSubscribe>("topic_subscribe_01");
  // /* 运行节点，并检测退出信号*/
  // rclcpp::spin(node);
  // rclcpp::shutdown();
  return 0;
}
