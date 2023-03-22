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
    if (!boost::filesystem::exists(DB_URL_)) {
      continue;
    }
    if (boost::filesystem::file_size(DB_URL_) < db_size_threshold_ * 1024 * 1024) {
      continue;
    }

    std::stringstream cmd;
    cmd << "tar -cvf " << DB_URL_ << "-" << GetTime() << ".tgz" <<
      " " << DB_URL_;
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
      if (rc != SQLITE_OK) {
        ERROR("Failed to clear %s", DB_URL_.c_str());
      }
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
    msg.timestamp << "," << msg.touch_state << ");";
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
bool cyberdog::manager::BlackBox::write(const std::string & name)
{
  std::string sql;
  std::string filename = filename_ + "/userInformation.db";
  sqlite3 * db;
  char * zErrMsg = 0;
  INFO("enter writer!!!!!!!!!");
  int n = sqlite3_open(filename.c_str(), &db);
  INFO("write  sqlite_open:%d", n);
  sql = std::string("INSERT INTO USER (NAME,VOICE,FACE) ") +
    std::string("VALUES (") + "\'" + name + "\'" + ", 0, 0 ); ";
  INFO("write sql is :%s", sql.c_str());
  int rc = sqlite3_exec(db, sql.c_str(), 0, 0, &zErrMsg);
  INFO("writer sqlite3_exec ERROR CODE: %d", rc);
  sqlite3_close(db);
  if (rc == SQLITE_OK) {
    INFO("[black_box]: add [%s] success", name.c_str());
    return true;
  } else {
    INFO("[black_box]: add [%s] failed, the error code is %d", name.c_str(), rc);
    return false;
  }
}
bool cyberdog::manager::BlackBox::ModifyUserName(
  const std::string name,
  const std::string new_name)
{
  std::string filename = filename_ + "/userInformation.db";
  if (!DataBaseExit(filename)) {
    return false;
  }
  INFO("[black_box]: enter ModifyUserName()");
  std::string query_update;
  sqlite3 * db;
  char * zErrMsg = 0;
  query_update = std::string("UPDATE USER SET NAME = ") + "\'" + new_name + "\'" +
    std::string(" WHERE NAME =") + "\'" + name + "\'";
  INFO("[black_box]: %s", query_update.c_str());
  int rc = sqlite3_open(filename.c_str(), &db);
  INFO("[black_box]: modify sqlite_open() return Code is :%d ", rc);
  if (rc == SQLITE_OK) {
    if (!HasUser(name)) {
      return false;
    }
    int hc = sqlite3_exec(db, query_update.c_str(), 0, 0, &zErrMsg);
    INFO("[black_box]: modify sqlite_exec() return Code is : %d", hc);
    if (hc == SQLITE_OK) {
      INFO("[black_box]: modify success!!!");
      sqlite3_close(db);
      return true;
    }
  }
}
/**
 * @brief add a user into database
 *
 * @version v1.0(2022/09/28)
 * @param name
 * @return true
 * @return false
 */
bool cyberdog::manager::BlackBox::AddUser(const std::string & name)
{
  std::string filename = filename_ + "/userInformation.db";
  sqlite3 * db;
  char * sql;
  char * zErrMsg = 0;
  INFO("[black_box]: add %s", name.c_str());
  if (!boost::filesystem::exists(filename)) {
    int rc = sqlite3_open(filename.c_str(), &db);
    INFO("[black_box]: ADDUser sqlite_open = %d", rc);
    INFO("%s", filename.c_str());
    if (rc != SQLITE_OK) {
      INFO("[black_box]: open database error");
      return false;
    }
    sql = "CREATE TABLE USER("
      "ID             INTEGER PRIMARY KEY autoincrement,"
      "NAME           TEXT    NOT NULL,"
      "VOICE          INT     NOT NULL,"
      "FACE           INT     NOT NULL );";
    int hc = sqlite3_exec(db, sql, 0, 0, &zErrMsg);
    INFO("[black_box]: test open database ERROR CODE:%d", hc);
    if (hc == SQLITE_OK) {
      if (write(name)) {
        return true;
      } else {
        return false;
      }
    } else {
      INFO("[black_box]: add user failed!!!");
      return false;
    }
  } else {
    INFO("[black_box]:  the file exists");
    INFO("[black_box]: add %s", name.c_str());
    if (HasUser(name)) {
      INFO("%s", name.c_str());
      INFO("[black_box] : the %s has exist!", name.c_str());
      return true;
    }
    if (write(name)) {
      return true;
    } else {
      return false;
    }
  }
}

/**
 * @brief
 *
 * @version v1.0(2022/09/28)
 * @param name
 * @return true
 * @return false
 */
bool cyberdog::manager::BlackBox::DeleteUser(const std::string & name)
{
  std::string filename = filename_ + "/userInformation.db";
  sqlite3 * db;
  char * zErrMsg = 0;
  INFO("[black_box]: will delete %s!!!!!!!", name.c_str());
  std::string query = std::string("DELETE FROM USER WHERE NAME = ") + std::string("\'") + name +
    "\'" + ";";
  INFO("[black_box]: %s", query.c_str());
  if (!DataBaseExit(filename)) {
    return false;
  }
  int rc;
  rc = sqlite3_open(filename.c_str(), &db);
  if (rc == SQLITE_OK) {
    if (!HasUser(name)) {
      return false;
    }
    int flag = sqlite3_exec(db, query.c_str(), 0, 0, &zErrMsg);
    if (flag == SQLITE_OK) {
      INFO("[black_box]: delete %s success", name.c_str());
      return true;
    } else {
      INFO("[black_box]: delete  sqlite_exec() return code :%d", flag);
      INFO("[black_box]: delete %s failed!!!", name.c_str());
      return false;
    }
  }
}

/**
 * @brief
 *
 * @version v1.0(2022/09/28)
 * @return true
 * @return false
 */
bool cyberdog::manager::BlackBox::SearchUser(
  std::vector<cyberdog::manager::MemberInformaion> & UserVector)
{
  sqlite3 * db;
  std::string sql;
  char ** dbResult;
  int nRow;
  int nColumn;
  cyberdog::manager::MemberInformaion memberInformation_;
  std::string filename = filename_ + "/userInformation.db";
  if (!DataBaseExit(filename)) {
    return false;
  }
  int rc = sqlite3_open(filename.c_str(), &db);
  if (rc) {
    INFO("[black_box:] %s", sqlite3_errmsg(db));
    return false;
  }

  sql = "SELECT * FROM USER";
  rc = sqlite3_get_table(db, sql.c_str(), &dbResult, &nRow, &nColumn, 0);

  if (rc == SQLITE_OK) {
    for (int i = 1; i <= nRow; i++) {
      memberInformation_.name = dbResult[i * nColumn + 1];
      memberInformation_.voiceStatus = std::atoi(dbResult[i * nColumn + 2]);
      memberInformation_.faceStatus = std::atoi(dbResult[i * nColumn + 3]);
      UserVector.push_back(memberInformation_);
      INFO(
        "[black_box]: Search all:%s:%d %d", memberInformation_.name.c_str(),
        memberInformation_.voiceStatus, memberInformation_.faceStatus);
    }
    sqlite3_free_table(dbResult);
    sqlite3_close(db);
    return true;
  } else {
    INFO("[black_box:] get datebase table failed!!!");
    sqlite3_close(db);
    return false;
  }
}

bool cyberdog::manager::BlackBox::HasUser(const std::string & name)
{
  sqlite3 * db;
  std::string sql;
  char ** dbResult;
  int nRow;
  int nColumn;
  std::string filename = filename_ + "/userInformation.db";
  if (!DataBaseExit(filename)) {
    return false;
  }
  int rc = sqlite3_open(filename.c_str(), &db);
  if (rc) {
    INFO("[black_box:] %s", sqlite3_errmsg(db));
    return false;
  }
  sql = std::string("SELECT * FROM USER") + std::string(" WHERE NAME =") + "\'" + name + "\'";
  rc = sqlite3_get_table(db, sql.c_str(), &dbResult, &nRow, &nColumn, 0);
  INFO("[black_box]: nRow:%d, nColumn:%d", nRow, nColumn);
  sqlite3_free_table(dbResult);
  sqlite3_close(db);
  if (nRow == 0) {
    INFO("[black_box]: can't find %s", name.c_str());
    return false;
  } else {
    return true;
  }
}
/**
 * @brief
 *
 * @version v1.0(2022/09/28)
 * @param name
 * @return true
 * @return false
 */
bool cyberdog::manager::BlackBox::SearchSingleUser(const std::string & name, int * result)
{
  sqlite3 * db;
  std::string sql;
  char ** dbResult;
  int nRow;
  int nColumn;
  cyberdog::manager::MemberInformaion memberInformation_;
  std::string filename = filename_ + "/userInformation.db";
  if (!DataBaseExit(filename)) {
    // INFO("[manager:black_box]: the %s not exit", filename.c_str());
    return false;
  }
  int rc = sqlite3_open(filename.c_str(), &db);
  if (rc) {
    INFO("[black_box:] %s", sqlite3_errmsg(db));
    return false;
  }
  sql = std::string("SELECT * FROM USER") + std::string(" WHERE NAME =") + "\'" + name + "\'";
  rc = sqlite3_get_table(db, sql.c_str(), &dbResult, &nRow, &nColumn, 0);
  INFO("[black_box]: nRow:%d, nColumn:%d", nRow, nColumn);
  if (rc == SQLITE_OK & nRow > 0) {
    result[0] = std::atoi(dbResult[nColumn + 2]);
    result[1] = std::atoi(dbResult[nColumn + 3]);
    sqlite3_free_table(dbResult);
    sqlite3_close(db);
    INFO(
      "[black_box]: search single user :voice:%d,face:%d", result[0], result[1]);
    return true;
  } else {
    INFO("[black_box]: search failed");
    return false;
  }
}

/**
 * @brief
 *
 * @version v1.0(2022/09/28)
 * @param name
 * @param status
 * @param newStatus
 * @return true
 * @return false
 */
bool cyberdog::manager::BlackBox::ModifyUser(const std::string & name, int status, int newStatus)
{
  std::string filename = filename_ + "/userInformation.db";
  if (!DataBaseExit(filename)) {
    return false;
  }
  INFO("[black_box]: enter modify !!!!!!!!!!");
  std::string query_update;
  sqlite3 * db;
  char * zErrMsg = 0;
  if (status == 0) {
    query_update = std::string("UPDATE USER SET VOICE = ") + std::to_string(newStatus) +
      std::string(" WHERE NAME =") + "\'" + name + "\'";
  } else if (status == 1) {
    query_update = std::string("UPDATE USER SET FACE = ") + std::to_string(newStatus) +
      std::string(" WHERE NAME =") + "\'" + name + "\'";
  }
  INFO("[black_box]: %s", query_update.c_str());
  int rc = sqlite3_open(filename.c_str(), &db);
  INFO("[black_box]: modify sqlite_open() return Code is :%d ", rc);
  if (rc == SQLITE_OK) {
    if (!HasUser(name)) {
      return false;
    }
    int hc = sqlite3_exec(db, query_update.c_str(), 0, 0, &zErrMsg);
    INFO("[black_box]: modify sqlite_exec() return Code is : %d", hc);
    if (hc == SQLITE_OK) {
      INFO("[black_box]: modify success!!!");
      sqlite3_close(db);
      return true;
    }
  }
}

/**
 * @brief
 *
 * @version v1.0(2022/09/28)
 * @param DB_path
 * @return true
 * @return false
 */
bool cyberdog::manager::BlackBox::DataBaseExit(const std::string DB_path)
{
  if (boost::filesystem::exists(DB_path)) {
    return true;
  } else {
    INFO("[black_box]:the %s not exit", DB_path.c_str());
    return false;
  }
}
bool cyberdog::manager::BlackBox::ModifyUnlockStatus(const std::string & details, int status)
{
  std::string filename = filename_ + "/UnlockStatus.db";
  sqlite3 * db;
  std::string query_update;
  std::string details_ = "REBOOTSTATUS";
  char * zErrMsg = 0;
  if (std::strcmp(details_.c_str(), details.c_str()) == 0) {
    query_update = std::string("UPDATE UnlockStatus SET REBOOTSTATUS = ") + std::to_string(status) +
      std::string(" WHERE NAME =") + "\'" + item + "\'";
  } else {
    query_update = std::string("UPDATE UnlockStatus SET UPSENDSTATUS = ") + std::to_string(status) +
      std::string(" WHERE NAME =") + "\'" + item + "\'";
  }
  INFO("[black_box]: %s", query_update.c_str());
  int rc = sqlite3_open(filename.c_str(), &db);
  INFO("[black_box]: modify return Code is :%d ", rc);
  int rc_ = sqlite3_exec(db, query_update.c_str(), 0, 0, &zErrMsg);
  INFO("[black_box]: modify exec return Code is :%d ", rc_);
  sqlite3_close(db);
  return true;
}
bool cyberdog::manager::BlackBox::readUnlockStatus(int * result)
{
  INFO("enter readUnlockStatus");
  sqlite3 * db;
  std::string sql;
  char ** dbResult;
  int nRow;
  int nColumn;
  cyberdog::manager::MemberInformaion memberInformation_;
  std::string filename = filename_ + "/UnlockStatus.db";
  if (!DataBaseExit(filename)) {
    INFO("the unlock status record DB is not exit!!!");
    return false;
  } else {
    int rc = sqlite3_open(filename.c_str(), &db);
    if (rc) {
      INFO("[black_box:] %s", sqlite3_errmsg(db));
      return false;
    }
    sql = std::string("SELECT * FROM UnlockStatus") + std::string(" WHERE NAME =") + "\'" + item +
      "\'";
    rc = sqlite3_get_table(db, sql.c_str(), &dbResult, &nRow, &nColumn, 0);
    if (rc == SQLITE_OK & nRow > 0) {
      result[0] = std::atoi(dbResult[nColumn + 1]);
      result[1] = std::atoi(dbResult[nColumn + 2]);
      sqlite3_free_table(dbResult);
      sqlite3_close(db);
      INFO(
        "[black_box]: UnlockStatus :REBOOTSTATUS:%d,UPSENDSTATUS:%d", result[0], result[1]);
      return true;
    } else {
      INFO("[black_box]: UnlockStatus read failed");
      return false;
    }
  }
}
bool cyberdog::manager::BlackBox::CreateUnlockStatusDB()
{
  std::string filename = filename_ + "/UnlockStatus.db";
  sqlite3 * db;
  char * sql;
  char * zErrMsg = 0;
  INFO("[black_box]: enter createUnlockStatusDB");
  if (!DataBaseExit(filename)) {
    int rc = sqlite3_open(filename.c_str(), &db);
    INFO("[black_box]: create UnlockStatus sqlite_open = %d", rc);
    INFO("%s", filename.c_str());
    if (rc != SQLITE_OK) {
      INFO("[black_box]: open database error");
      return false;
    }
    sql = "CREATE TABLE UnlockStatus("
      "NAME           TEXT PRIMARY KEY    NOT NULL,"
      "REBOOTSTATUS          INT     NOT NULL,"
      "UPSENDSTATUS          INT     NOT NULL );";
    int hc = sqlite3_exec(db, sql, 0, 0, &zErrMsg);
    INFO("[black_box]: test open UnlockStatus database ERROR CODE:%d", hc);
    if (hc == SQLITE_OK) {
      INFO("create unlock DB, insert information");
      std::string sql_;
      sql_ = std::string("INSERT INTO UnlockStatus (NAME,REBOOTSTATUS,UPSENDSTATUS) ") +
        std::string("VALUES (") + "\'" + item + "\'" + ", 0, 0 ); ";

      INFO("write sql is :%s", sql_.c_str());
      int rc = sqlite3_exec(db, sql_.c_str(), 0, 0, &zErrMsg);
      INFO("writer sqlite3_exec ERROR CODE: %d", rc);
      sqlite3_close(db);
      return true;
    } else {
      INFO("[black_box]: add user UnlockStatus failed!!!");
      return false;
    }
  } else {
    INFO("[black_box]:  the UnlockStatus.db exists");
    ModifyUnlockStatus("REBOOTSTATUS", 0);
    ModifyUnlockStatus("UPSENDSTATUS", 0);
    return true;
  }
}
