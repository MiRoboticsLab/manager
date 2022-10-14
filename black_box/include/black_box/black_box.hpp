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
#ifndef BLACK_BOX__BLACK_BOX_HPP_
#define BLACK_BOX__BLACK_BOX_HPP_
#include <sqlite3.h>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <protocol/msg/touch_status.hpp>
#include <rclcpp/serialization.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cyberdog_common/cyberdog_toml.hpp>
#include <cyberdog_common/cyberdog_log.hpp>
#include <fstream>
#include <vector>
#include <map>
#include <memory>
#include <string>
#include <mutex>
namespace cyberdog
{
namespace manager
{

struct Table
{
  std::string name;
  std::string key;
  std::vector<std::vector<std::string>> fields;
};

struct MemberInformaion
{
  std::string name;
  int voiceStatus;
  int faceStatus;
};

class BlackBox final
{
  using TouchStatusMsg = protocol::msg::TouchStatus;

public:
  explicit BlackBox(rclcpp::Node::SharedPtr node_ptr);
  BlackBox() {}
  BlackBox(const BlackBox &) = delete;
  BlackBox & operator=(const BlackBox &) = delete;
  ~BlackBox() {}

  bool Config();
  bool Init();

private:
  /* ros implementation, node and subscribers */
  rclcpp::Node::SharedPtr node_ptr_ {nullptr};
  rclcpp::Subscription<TouchStatusMsg>::SharedPtr touch_status_sub_ {nullptr};
  sqlite3 * ppDb_;
  std::string DB_URL_;
  std::stringstream errmsg_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> general_subs_;
  std::vector<std::string> topic_names_;
  std::map<std::string, Table> tables_;
  std::map<std::string, std::string> topics_map_;
  std::string CREAT_TABLES_SQL_;
  std::mutex query_mutex_;
  size_t db_size_threshold_;

private:
  void GeneralMsgCallback(
    std::shared_ptr<rclcpp::SerializedMessage> msg,
    const std::string topic_type);
  bool ConnetDB(std::string & DB_URL);
  bool InsertTouchStatus(const TouchStatusMsg & msg, const std::string & topic_name);
  void RollOverDB();
  std::string GetTime();

public:
  // new function
  bool write(const std::string & name);
  bool AddUser(const std::string & name);
  bool DeleteUser(const std::string & name);
  bool SearchUser(std::vector<cyberdog::manager::MemberInformaion> & UserVector);
  bool SearchSingleUser(const std::string & name, int * result);
  bool ModifyUser(const std::string & name, int status, int newStatus);
  bool HasUser(const std::string & name);
  bool DataBaseExit(const std::string DB_path);
};  // class BlackBox
}  // namespace manager
}  // namespace cyberdog

#endif  // BLACK_BOX__BLACK_BOX_HPP_
