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
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/writer.h"
#include "cyberdog_common/cyberdog_log.hpp"

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

private:
/**
 * @brief read json from a file
 *
 */
  std::string readJsonFile()
  {
    // read json content into string.
    std::string stringFromStream;
    std::ifstream in;
    std::string filename = "./UserAccount.json";
    in.open(filename, std::ifstream::in);
    if (!in.is_open()) {
      std::ofstream ofs(filename);
      INFO("[%s:] the %s has been created", CYBETDOGMANAGER, filename.c_str());
      json::Document doc;
      doc.SetObject();
      json::Document::AllocatorType & a = doc.GetAllocator();
      json::Value a1(json::kArrayType);
      doc.AddMember("content", a1, a);
      WriteDocumentToFile(doc);
    }
    std::string line;
    while (getline(in, line)) {
      stringFromStream.append(line + "\n");
    }
    in.close();
    return stringFromStream;
  }

/**
 * @brief switch string to json
 *
 */
  json::Document JsonToString(std::string stringFromStream)
  {
    json::Document doc;
    doc.Parse<0>(stringFromStream.c_str());
    if (doc.HasParseError()) {
      json::ParseErrorCode code = doc.GetParseError();
      std::cout << code << std::endl;
      ERROR("ParseError, Error code is %d", code);
    }
    return doc;
  }

  /**
 * @brief create a json string
 *
 */
  json::Document createJson(std::string username, int VoiceRecordStatus, int FaceRecordStatus)
  {
    std::string stringFromStream = readJsonFile();
    json::Document doc = JsonToString(stringFromStream);
    json::Value & contents = doc["content"];
    doc.SetObject();
    json::Document::AllocatorType & allocator = doc.GetAllocator();
    json::Value str_value(json::kStringType);
    str_value.SetString(username.c_str(), username.size());

    json::Value array_obj1(json::kObjectType);
    array_obj1.AddMember("username", str_value, allocator);
    array_obj1.AddMember("VoiceRecordStatus", VoiceRecordStatus, allocator);
    array_obj1.AddMember("FaceRecordStatus", FaceRecordStatus, allocator);
    contents.PushBack(array_obj1, allocator);
    doc.AddMember("content", contents, allocator);
    json::StringBuffer s;
    json::Writer<json::StringBuffer> writer(s);
    doc.Accept(writer);
    INFO("[%s:] the %s has been created", CYBETDOGMANAGER, username.c_str());
    return doc;
  }

  bool WriteDocumentToFile(const json::Document & doc)
  {
    FILE * fp = fopen("./UserAccount.json", "w");
    if (fp == NULL) {
      std::cout << "[UserAccountManager: ] write file error" << std::endl;
      ERROR("[%s:] write document to file error", CYBETDOGMANAGER);
      return false;
    }
    char writeBuffer[65535];
    json::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));
    json::PrettyWriter<json::FileWriteStream> writer(os);
    doc.Accept(writer);
    fclose(fp);

    return true;
  }

public:
/**
 * @brief  add  the information of a member
 *
 */
  bool AddMember(std::string username)
  {
    json::Document userInformation = createJson(username, 0, 0);
    if (WriteDocumentToFile(userInformation)) {
      INFO("[%s:] Add %s success", CYBETDOGMANAGER, username.c_str());
      return true;
    } else {
      INFO("[%s:] Add %s failed", CYBETDOGMANAGER, username.c_str());
      return false;
    }
  }

/**
 * @brief search all user
 *
 */
  bool SearAllUser(
    std::vector<cyberdog::common::CyberdogAccountManager::UserInformation> & vectorUser)
  {
    std::string stringFromStream = readJsonFile();
    // parse json from string.
    json::Document doc = JsonToString(stringFromStream);
    json::Value & contents = doc["content"];
    if (contents.IsArray()) {
      for (size_t i = 0; i < contents.Size(); ++i) {
        json::Value & v = contents[i];
        cyberdog::common::CyberdogAccountManager::UserInformation info;
        info.username = v["username"].GetString();
        info.voiceStatus = v["VoiceRecordStatus"].GetInt();
        info.faceStatus = v["FaceRecordStatus"].GetInt();
        vectorUser.push_back(info);
      }
    } else {
      INFO("[%s:] the content isn't an array", CYBETDOGMANAGER);
      return false;
    }
    INFO("[%s:] search success", CYBETDOGMANAGER);
    return true;
  }

/**
 * @brief search the information of a user,
 * param: username
 * param: flag
 * return:  flag=0,return voice status of username; flag=1,return face status of username
 */
  int SearchOneUser(std::string username, int flag)
  {
    int result[2] = {NULL, NULL};
    if (SearchUser(username, result)) {
      INFO("[%s:] search success", CYBETDOGMANAGER);
      return result[flag];
    } else {
      INFO("[%s:] the %s not exit", CYBETDOGMANAGER, username.c_str());
      return -1;
    }
  }

/**
 * @brief search the information of a user
 * param: username
 * return: a array including user's voice status and face status
 */
  bool SearchUser(const std::string username, int * result)
  {
    std::string stringFromStream = readJsonFile();
    json::Document doc = JsonToString(stringFromStream);
    bool flag = true;
    json::Value & contents = doc["content"];
    if (contents.IsArray()) {
      for (size_t i = 0; i < contents.Size(); ++i) {
        json::Value & v = contents[i];
        assert(v.IsObject());
        if (v.HasMember("username") && v["username"].IsString()) {
          std::string usernameCopy = v["username"].GetString();
          if (strcmp(usernameCopy.c_str(), username.c_str()) == 0) {
            result[0] = v["VoiceRecordStatus"].GetInt();
            result[1] = v["FaceRecordStatus"].GetInt();
            flag = false;
          }
        }
      }
    }
    if (flag) {
      INFO("[%s:] search success", CYBETDOGMANAGER);
      return true;
    } else {
      INFO("[%s:] the %s not exit", CYBETDOGMANAGER, username.c_str());
      return false;
    }
  }

/**
 * @brief  modify information of user
 * param:  username
 * param:  new VoiceStatus
 * note:   0:Entered, 1:No entries ,2:Entering
 */
  void ModifyInformation(std::string username, int NewVoiceStatus, int NewFaceStatus)
  {
    std::string stringFromStream = readJsonFile();
    // parse json from string.
    json::Document doc = JsonToString(stringFromStream);
    json::Value & contents = doc["content"];
    if (contents.IsArray()) {
      for (size_t i = 0; i < contents.Size(); ++i) {
        json::Value & v = contents[i];
        assert(v.IsObject());
        if (v.HasMember("username") && v["username"].IsString() ) {
          std::string str = v["username"].GetString();
          if (strcmp(str.c_str(), username.c_str()) == 0) {
            v["VoiceRecordStatus"].SetInt(NewVoiceStatus);
            v["FaceRecordStatus"].SetInt(NewFaceStatus);
          }
        }
      }
    }
    json::StringBuffer userInformation;
    json::Writer<json::StringBuffer> writer(userInformation);
    doc.Accept(writer);
    std::string str = userInformation.GetString();
    WriteDocumentToFile(doc);
  }

/**
 * @brief  modify the voice status or face status of a user
 * para: username
 * para: status    the new status of voice or face
 * para: flag   flag=0 ,will modify the voice status; falg=1 will modify the face status
 */
  bool ModifyUserInformation(std::string username, int status, int flag)
  {
    std::string stringFromStream = readJsonFile();
    // parse json from string.
    json::Document doc = JsonToString(stringFromStream);
    json::Value & contents = doc["content"];
    if (contents.IsArray()) {
      bool flag_bool = true;
      for (size_t i = 0; i < contents.Size(); ++i) {
        json::Value & v = contents[i];
        assert(v.IsObject());
        if (v.HasMember("username") && v["username"].IsString() ) {
          std::string str = v["username"].GetString();
          if (strcmp(str.c_str(), username.c_str()) == 0) {
            if (flag == 0) {
              v["VoiceRecordStatus"].SetInt(status);
            } else if (flag == 1) {
              v["FaceRecordStatus"].SetInt(status);
            }

            flag_bool = false;
          }
        } else {
          INFO("[%s:]  \"username\"filed not exit", CYBETDOGMANAGER);
          return false;
        }
      }
      if (flag_bool) {
        INFO("[%s:]  the %s not exit", CYBETDOGMANAGER, username.c_str());
        return false;
      }
    } else {
      INFO("[%s:] the content isn't an array", CYBETDOGMANAGER);
      return false;
    }
    json::StringBuffer userInformation;
    json::Writer<json::StringBuffer> writer(userInformation);
    doc.Accept(writer);
    std::string str = userInformation.GetString();
    WriteDocumentToFile(doc);
    INFO("[%s:] Modify success", CYBETDOGMANAGER);
    return true;
  }

/**
 * @brief  delete information of user
 *
 */
  void DeleteUserInformation(std::string username)
  {
    json::Document docu;
    docu.SetObject();
    json::Document::AllocatorType & alloctor_ = docu.GetAllocator();
    json::Value newArray(json::kArrayType);  // 新建一个数组
    std::string stringFromStream = readJsonFile();
    json::Document doc = JsonToString(stringFromStream);
    json::Value & contents = doc["content"];
    if (contents.IsArray()) {
      for (size_t i = 0; i < contents.Size(); ++i) {
        json::Value & v = contents[i];
        assert(v.IsObject());
        if (v.HasMember("username") && v["username"].IsString()) {
          std::string usernameCopy = v["username"].GetString();
          if (strcmp(usernameCopy.c_str(), username.c_str()) == 0) {
            size_t j = i;
            for (size_t h = 0; h < j; h++) {
              newArray.PushBack(contents[h].GetObject(), alloctor_);
            }
            for (size_t h = j + 1; h < contents.Size(); h++) {
              newArray.PushBack(contents[h].GetObject(), alloctor_);
            }
          }
        }
      }
    }
    docu.AddMember("content", newArray, alloctor_);
    json::StringBuffer userInformation;
    json::Writer<json::StringBuffer> writer(userInformation);
    docu.Accept(writer);
    std::string str = userInformation.GetString();
    WriteDocumentToFile(docu);
    INFO("[%s:] delete success", CYBETDOGMANAGER);
  }

/**
 * @brief delete user's information of face,set voice status to 0
 * param  username
 */
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
