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

#include <black_box/black_box.hpp>
#include <boost/filesystem.hpp>
#include <sys/time.h>
#include <string>
#include <vector>
#include <memory>

cyberdog::manager::BlackBox::BlackBox(rclcpp::Node::SharedPtr node_ptr)
{
  if (node_ptr != nullptr) {
    node_ptr_ = node_ptr;
  }
  std::thread{std::bind(&cyberdog::manager::BlackBox::RollOverDB, this)}.detach();
}

std::string cyberdog::manager::BlackBox::GetTime()
{
  struct timeval tv;
  char buf[32];
  gettimeofday(&tv, NULL);
  strftime(buf, 32, "%Y_%m_%d-%H_%M_%S", localtime(&tv.tv_sec));
  std::string s(buf);
  return s;
}

void cyberdog::manager::BlackBox::RollOverDB()
{
  while (rclcpp::ok()) {
    rclcpp::Rate(1).sleep();
    if (!boost::filesystem::exists(DB_URL_)) {continue;}
    if (boost::filesystem::file_size(DB_URL_) < db_size_threshold_ * 1024 * 1024) {continue;}

    std::stringstream cmd;
    cmd << "tar -cvf " << DB_URL_ << "-" << GetTime() << ".tgz" << " " << DB_URL_;
    std::stringstream query;
    for (auto table : tables_) {
      query << " DELETE FROM " << table.second.name << ";";
    }
    std::lock_guard<std::mutex> lock(query_mutex_);
    system(cmd.str().c_str());
    // TODO(Harvey): rm the path in tgz; upload tgz files
    if (ppDb_) {
      int rc = SQLITE_OK;
      rc = sqlite3_exec(ppDb_, query.str().c_str(), 0, 0, 0);
      if (rc != SQLITE_OK) {ERROR("Failed to clear %s", DB_URL_.c_str());}
    }
  }
}

bool cyberdog::manager::BlackBox::Config()
{
  // 读取toml配置， 选择订阅哪些消息数据
  toml::value value;
  auto local_share_dir = ament_index_cpp::get_package_share_directory("params");
  auto local_config_dir = local_share_dir + std::string("/toml_config/database/config.toml");
  if (!cyberdog::common::CyberdogToml::ParseFile(
      local_config_dir, value))
  {
    return false;
  }
  if (!cyberdog::common::CyberdogToml::Get(value, "topic_names", topic_names_)) {
    return false;
  }
  if (!cyberdog::common::CyberdogToml::Get(value, "DB_URL", DB_URL_)) {
    return false;
  }
  if (!cyberdog::common::CyberdogToml::Get(value, "DB_size", db_size_threshold_)) {
    return false;
  }
  toml::value tables;
  if (!cyberdog::common::CyberdogToml::Get(value, "tables", tables)) {
    return false;
  }
  std::stringstream sql;
  for (uint8_t i = 0; i < tables.size(); i++) {
    sql << "CREATE TABLE IF NOT EXISTS ";
    Table table;
    std::string topic, name, key;
    std::vector<std::vector<std::string>> fields;
    if (!cyberdog::common::CyberdogToml::Get(tables[i], "topic", topic)) {
      return false;
    }
    if (!cyberdog::common::CyberdogToml::Get(tables[i], "name", name)) {
      return false;
    }
    table.name = name;
    if (!cyberdog::common::CyberdogToml::Get(tables[i], "key", key)) {
      return false;
    }
    table.key = key;
    if (!cyberdog::common::CyberdogToml::Get(tables[i], "fields", fields)) {
      return false;
    }
    table.fields = fields;
    tables_.emplace(topic, table);
    sql << name << "(";
    for (auto field : fields) {
      for (auto f : field) {
        sql << f << " ";
      }
      sql << ", ";
    }
    sql << "PRIMARY KEY(" << key << "));";
  }
  CREAT_TABLES_SQL_ = sql.str();
  return true;
}

bool cyberdog::manager::BlackBox::Init()
{
  // 初始化数据库及ros消息回调，允许失败时返回false
  if (node_ptr_ == nullptr) {
    // error msg
    return false;
  }
  if (!Config()) {
    return false;
  }
  if (DB_URL_.empty()) {
    return false;
  }
  if (!ConnetDB(DB_URL_)) {
    return false;
  }
  topics_map_.emplace("touch_status", "protocol/msg/TouchStatus");
  // TODO(Harvey) other topics;

  for (auto topic_name : topic_names_) {
    if (topics_map_.find(topic_name) == topics_map_.end()) {
      WARN("Topic %s is not in topics map, omitted!", topic_name.c_str());
      continue;
    }
    general_subs_.push_back(
      node_ptr_->create_generic_subscription(
        topic_name,
        topics_map_[topic_name], rclcpp::SystemDefaultsQoS(),
        std::bind(
          &BlackBox::GeneralMsgCallback, this, std::placeholders::_1,
          topic_name)));
  }
  return true;
}

bool cyberdog::manager::BlackBox::ConnetDB(std::string & DB_URL)
{
  int rc = SQLITE_OK;
  bool dbFileExist = false;
  ppDb_ = 0;
  if (std::string(getenv("USER")) != "mi") {
    DB_URL = std::string(getenv("HOME")) + std::string("/black_box.db");
    WARN("Not running on Cyberdog, DB URL is redirected to %s!", DB_URL.c_str());
  }
  DB_URL_ = DB_URL;
  dbFileExist = boost::filesystem::exists(DB_URL);
  rc = sqlite3_open(DB_URL.c_str(), &ppDb_);
  if (dbFileExist && rc == SQLITE_OK) {
    // TODO(Harvey)
  } else if (!dbFileExist) {
    rc = sqlite3_exec(ppDb_, CREAT_TABLES_SQL_.c_str(), 0, 0, 0);
    if (rc != SQLITE_OK) {
      return false;
    }
  }
  return true;
}

bool cyberdog::manager::BlackBox::InsertTouchStatus(
  const TouchStatusMsg & msg,
  const std::string & topic_name)
{
  std::stringstream query;
  std::stringstream fields_tmp;
  for (auto field : tables_[topic_name].fields) {
    fields_tmp << field.front() << ",";
  }
  std::string fields = fields_tmp.str();
  fields.erase(fields.size() - 1);
  query << "INSERT INTO " << tables_[topic_name].name << "(" << fields << ")VALUES(" <<
    msg.timestamp << "," <<
    msg.touch_state << ");";
  if (!ppDb_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(query_mutex_);
  int rc = sqlite3_exec(ppDb_, query.str().c_str(), 0, 0, 0);
  if (rc != SQLITE_OK) {
    return false;
  }
  return true;
}

void cyberdog::manager::BlackBox::GeneralMsgCallback(
  std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string topic_name)
{
  // save msg data
  if (topic_name == "touch_status") {
    TouchStatusMsg touch_status_msg;
    rclcpp::Serialization<TouchStatusMsg> serializer;
    serializer.deserialize_message(msg.get(), &touch_status_msg);
    InsertTouchStatus(touch_status_msg, topic_name);
  }
}
