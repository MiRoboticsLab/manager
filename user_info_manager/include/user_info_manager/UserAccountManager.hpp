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
/**
 * @brief  add a user to database
*/
  bool AddMember(std::string username)
  {
    INFO("[UserAccountManager]: enter ADDMember()");
    cyberdog::manager::BlackBox black_box_;
    return black_box_.AddUser(username);
  }
/**
 * @brief  search all user form database
*/
  bool SearAllUser(
    std::vector<cyberdog::common::CyberdogAccountManager::UserInformation> & vectorUser)
  {
    INFO("[UserAccountManager]: enter serachAllUser()");
    cyberdog::manager::BlackBox black_box;
    std::vector<cyberdog::manager::MemberInformaion> vectorUser_;
    if (black_box.SearchUser(vectorUser_) ) {
      INFO("[UserAccountManager]: enter serachAllUser [if]");
      for (int i = 0; i < vectorUser.size(); i++) {
        INFO("[UserAccountManager]: enter serachAllUser [for] cycle");
        INFO(
          "[UserAccountManager]: %s:%d %d", vectorUser_[i].name.c_str(),
          vectorUser_[i].voiceStatus, vectorUser_[i].faceStatus);
        vectorUser[i].username = vectorUser_[i].name;
        vectorUser[i].voiceStatus = vectorUser_[i].voiceStatus;
        vectorUser[i].faceStatus = vectorUser_[i].faceStatus;
      }
      return true;
    } else {
      INFO("[UserAccountManager]: search all user failed");
      return false;
    }
  }
/**
 * @brief  search a user information by username
 * param： username (the user's name), result (the array of user's information)
*/
  bool SearchUser(const std::string username, int * result)
  {
    INFO("[UserAccountManager]: enter SearchUser");
    cyberdog::manager::BlackBox black_box;
    if (black_box.SearchSingleUser(username, result) ) {
      INFO("[UserAccountManager]: enter SearchUser [if]");
      INFO("[UserAccountManager]: search singleuser's information is : %d, %d", result[0], result[1]);
      return true;
    } else {
      INFO("[UserAccountManager]: search single user failed");
      return false;
    }
  }
/**
 * @brief  search a user information by username
 * param： username (the user's name), 
 * param:  flag (flag=0 return the user's voice status; flag=1 return the user's face status)
 * return: the status of user's voice or face
*/
  int SearchOneUser(std::string username, int flag)
  {
    int result[2] = {0};
    if (SearchUser(username, result)) {
      INFO("[UserAccountManager]: search success");
      return result[flag];
    } else {
      INFO("[UserAccountManager]: the %s not exit", username.c_str());
      return -1;
    }
  }
/**
 * @brief  modify the indoformation of user
 * param: status. the new status(voice | face) of user
 * param: flag. flag =0 ,status = voice; flag =1 , status = face;
*/
  bool ModifyUserInformation(std::string username, int status, int flag)
  {
    INFO("[UserAccountManager]: enter ModifyUserInformation");
    cyberdog::manager::BlackBox black_box;
    if (black_box.ModifyUser(username, flag, status)) {
      INFO("[UserAccountManager]: enter Modify user [if]");
      INFO("[UserAccountManager]: modify username:%s, flag:%d, status:%d", username.c_str(), flag, status);
      return true;
    } else {
      INFO("[UserAccountManager]: modify user information failed");
      return false;
    }
  }
/**
 *@brief  delete the information of user
*/
  bool DeleteUserInformation(std::string username)
  {
    INFO("[UserAccountManager]: enter DeleteUserInformation");
    cyberdog::manager::BlackBox black_box;
    if (black_box.DeleteUser(username)) {
      INFO("[UserAccountManager]: delete user success");
      return true;
    } else {
      INFO("[UserAccountManager]: delete user failed!!!!");
      return false;
    }
  }
/**
 * @brief  delete the voice status of user
*/
  bool DeleteVoice(std::string username)
  {
    if (ModifyUserInformation(username, 0, 0)) {
      INFO("[UserAccountManager]: delete %s's voice status success", username.c_str());
      return true;
    } else {
      INFO("[UserAccountManager]: delete %s's voice status failed", username.c_str());
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
      INFO("[UserAccountManager]: delete %s's voice status success", username.c_str());
      return true;
    } else {
      INFO("[UserAccountManager]: delete %s's voice status failed", username.c_str());
      return false;
    }
  }
};
}  // namespace common
}  // namespace cyberdog
#endif  // USER_INFO_MANAGER__USERACCOUNTMANAGER_HPP_
