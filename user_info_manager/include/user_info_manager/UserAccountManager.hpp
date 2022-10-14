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
#ifndef USER_INFO_MANAGER__USERACCOUNTMANAGER_HPP_
#define USER_INFO_MANAGER__USERACCOUNTMANAGER_HPP_
#include <unistd.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include <exception>
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/writer.h"
#include "cyberdog_common/cyberdog_log.hpp"
#include "black_box/black_box.hpp"

#define CYBETDOGMANAGER "CyberdogAccountManager"

namespace cyberdog
{
namespace common
{
namespace json = rapidjson;
class CyberdogAccountManager
{
public:
  CyberdogAccountManager() {}
  ~CyberdogAccountManager() {}
  struct UserInformation
  {
    std::string username;
    int voiceStatus;
    int faceStatus;
  } userinformation;
  std::vector<cyberdog::common::CyberdogAccountManager::UserInformation> vectorUser;
  int result[2];

public:
  // const std::string base_account_dir = "/opt/ros2/cyberdog/share/params/toml_config/manager";
  bool AddMember(std::string username)
  {
    INFO("enter ADDMember");
    cyberdog::manager::BlackBox black_box_;
    return black_box_.AddUser(username);
  }

  bool SearAllUser(
    std::vector<cyberdog::common::CyberdogAccountManager::UserInformation> & vectorUser)
  {
    INFO("enter serachAllUser");
    cyberdog::manager::BlackBox black_box;
    std::vector<cyberdog::manager::MemberInformaion> vectorUser_;
    if (black_box.SearchUser(vectorUser_) ) {
      INFO("enter serachAllUser [if]");
      for (int i = 0; i < vectorUser.size(); i++) {
        INFO("enter serachAllUser [for] cycle");
        INFO("%s:%d %d", vectorUser_[i].name.c_str(),
          vectorUser_[i].voiceStatus, vectorUser_[i].faceStatus);
        vectorUser[i].username = vectorUser_[i].name;
        vectorUser[i].voiceStatus = vectorUser_[i].voiceStatus;
        vectorUser[i].faceStatus = vectorUser_[i].faceStatus;
      }
      return true;
    } else {
      INFO("search all user failed");
      return false;
    }
  }

  bool SearchUser(const std::string username, int * result)
  {
    INFO("enter SearchUser");
    cyberdog::manager::BlackBox black_box;
    if (black_box.SearchSingleUser(username, result) ) {
      INFO("enter SearchUser [if]");
      INFO("search singleuser's information is : %d, %d", result[0], result[1]);
      return true;
    } else {
      INFO("search single user failed");
      return false;
    }
  }

  int SearchOneUser(std::string username, int flag)
  {
    int result[2] = {0};
    if (SearchUser(username, result)) {
      INFO("[%s:] search success", CYBETDOGMANAGER);
      return result[flag];
    } else {
      INFO("[%s:] the %s not exit", CYBETDOGMANAGER, username.c_str());
      return -1;
    }
  }

  bool ModifyInformation(std::string username, int NewVoiceStatus, int NewFaceStatus)
  {
  }

  bool ModifyUserInformation(std::string username, int status, int flag)
  {
    INFO("enter ModifyUserInformation");
    cyberdog::manager::BlackBox black_box;
    if (black_box.ModifyUser(username, flag, status)) {
      INFO("enter Modify user [if]");
      INFO("modify username:%s, flag:%d, status:%d", username.c_str(), flag, status);
      return true;
    } else {
      INFO("modify user information failed");
      return false;
    }
  }
  bool DeleteUserInformation(std::string username)
  {
    INFO("enter DeleteUserInformation");
    cyberdog::manager::BlackBox black_box;
    if (black_box.DeleteUser(username)) {
      INFO("delete user success");
      return true;
    } else {
      INFO("delete user failed!!!!");
      return false;
    }
  }
  bool DeleteVoice(std::string username)
  {
    if (ModifyUserInformation(username, 0, 0)) {
      INFO("[%s:] delete %s's voice status success", CYBETDOGMANAGER, username.c_str());
      return true;
    } else {
      INFO("[%s:] delete %s's voice status failed", CYBETDOGMANAGER, username.c_str());
      return false;
    }
  }
/**
 * @brief  delete user's information of voice,set face status to 0
 * param   username
 */
  bool DeleteFace(std::string username)
  {
    if (ModifyUserInformation(username, 0, 1)) {
      INFO("[%s:] delete %s's voice status success", CYBETDOGMANAGER, username.c_str());
      return true;
    } else {
      INFO("[%s:] delete %s's voice status failed", CYBETDOGMANAGER, username.c_str());
      return false;
    }
  }
};
}  // namespace common
}  // namespace cyberdog
#endif  // USER_INFO_MANAGER__USERACCOUNTMANAGER_HPP_
