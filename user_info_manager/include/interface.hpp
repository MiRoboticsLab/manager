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
#ifndef INTERFACE_HPP_
#define INTERFACE_HPP_
#include <iostream>
#include <string>
#include <fstream>
#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/prettywriter.h"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{
/* 要求使用json代替rapidjson命名空间 */
namespace json = rapidjson;
/**
 * @brief 封装用户状态信息
 */
class UserInfoState
{
public:
  UserInfoState() {}
  ~UserInfoState() {}

public:
  /**
   * @brief 添加一个新的用户(object)
   *          1. 默认该object包含两个key：FaceState、VoiceState
   *          2. 默认新建用户的的key值为都0.
   * @param user_name 为新建的用户名，要求为std::string类型
   * @return  是否执行成功,若用户已经存在返回flase
   */
  static bool AddUser(std::string user_name)
  {
    ReadJsonFile(file_name);
    if (d.HasMember(user_name.c_str())) {
      WARN("the user %s already exists", user_name.c_str());
      return false;
    }
    AddJsonObject(user_name);
    WritejsonFile(file_name);
    return true;
  }

public:
  /**
   * @brief 删除一个已经存在的用户(object)
   *          1. 查找用户名，将整个object直接删除
   * @param user_name 为需删除的用户名，要求为std::string类型
   * @return  是否执行成功, 用户不存在则返回flase，删除完成则返回true.
   */
  static bool DeletUser(const std::string user_name)
  {
    ReadJsonFile(file_name);
    if (!d.HasMember(user_name.c_str())) {
      WARN("the user %s does not exist", user_name.c_str());
      return false;
    }
    DeletJsonObject(user_name);
    INFO("the user %s has been deleted", user_name.c_str());
    DeletJsonObject(user_name);
    WritejsonFile(file_name);
    return true;
  }

public:
  /**
   * @brief获取一个已经存在的用户(object)的FaceState状态信息
   * @param user_name 需查询的用户名，要求为std::string类型
   * @return  返回FaceState状态的状态值,若该用户不存在则返回404.
   */
  static int InquireUserFaceState(const std::string user_name)
  {
    ReadJsonFile(file_name);
    if (!d.HasMember(user_name.c_str())) {
      WARN("the user %s does not exist", user_name.c_str());
      value = 404;
      return value;
    }
    InquireJsonkey(user_name, key1, value);
    INFO("%s->%s: %d", user_name.c_str(), key1, value);
    return value;
  }

  /**
   * @brief获取一个已经存在的用户(object)的VoiceState状态信息
   * @param user_name 需查询的用户名，要求为std::string类型
   * @return  返回VoiceState状态的状态值,若该用户不存在则返回404.
   */
  static int InquireUserVoiceState(const std::string user_name)
  {
    ReadJsonFile(file_name);
    if (!d.HasMember(user_name.c_str())) {
      WARN("the user %s does not exist", user_name.c_str());
      value = 404;
      return value;
    }
    InquireJsonkey(user_name, key2, value);
    INFO("%s->%s: %d", user_name.c_str(), key2, value);
    return value;
  }

public:
  /**
   * @brief修改一个已经存在的用户(object)的FaceState状态信息
   * @param user_name 需查询的用户名，要求为std::string类型
   * @return  成功修改返回true,若该用户不存在则返回flase.
   */
  static bool ModifyUserFaceState(const std::string user_name, int value)
  {
    ReadJsonFile(file_name);
    if (!d.HasMember(user_name.c_str())) {
      WARN("the user %s does not exist", user_name.c_str());
      return false;
    }
    ModifyJsonKey(user_name, key1, value);
    WritejsonFile(file_name);
    return true;
  }

public:
  /**
   * @brief修改一个已经存在的用户(object)的VoiceState状态信息
   * @param user_name 需查询的用户名，要求为std::string类型
   * @return  成功修改返回true,若该用户不存在则返回flase.
   */
  static bool ModifyUserVoiceState(const std::string user_name, int value)
  {
    ReadJsonFile(file_name);
    if (!d.HasMember(user_name.c_str())) {
      WARN("the user %s does not exist", user_name.c_str());
      return false;
    }
    ModifyJsonKey(user_name, key2, value);
    WritejsonFile(file_name);
    return true;
  }

public:
  /**
   * @brief遍历已存在的所有用户姓名
   * @return  是否执行成功.
   */
  static bool PrintAllUserName()
  {
    ReadJsonFile(file_name);
    INFO("[1]:Users list: ");
    for (json::Value::ConstMemberIterator itr = d.MemberBegin();
      itr != d.MemberEnd(); ++itr)
    {
      std::cout << itr->name.GetString() << '\n';
    }
    return true;
  }

private:
  static json::Document d;
  static int value;
  static const char * file_name;
  static const char * key1;
  static const char * key2;

  static bool ReadJsonFile(const std::string file_name)
  {
    std::ifstream ifs;
    ifs.open(file_name);
    if (!ifs.is_open()) {
      WARN("the file %s does not exist, and it has been automatically created", file_name.c_str());
      std::ofstream ofs(file_name);
      const char * str = "{ }";
      ofs << str;
    }
    ifs.close();
    ifs.open(file_name);
    json::IStreamWrapper isw(ifs);
    d.ParseStream(isw);
    return true;
  }

  static bool WritejsonFile(const std::string file_name)
  {
    std::ofstream ofs(file_name);
    json::OStreamWrapper osw(ofs);
    rapidjson::PrettyWriter<json::OStreamWrapper> writer(osw);
    d.Accept(writer);
    return true;
  }

  static bool InquireJsonkey(const std::string & object, const std::string & key, int & value)
  {
    value = d[object.c_str()][key.c_str()].GetInt();
    return true;
  }
  static bool ModifyJsonKey(const std::string & object, const std::string & key, int & value)
  {
    d[object.c_str()][key.c_str()].SetInt(value);
    return true;
  }

  static bool AddJsonObject(std::string object)
  {
    json::Value obj_new(json::kObjectType);
    obj_new.AddMember("FaceState", 0, d.GetAllocator());
    obj_new.AddMember("VoiceState", 0, d.GetAllocator());
    json::Value obj(object.c_str(), object.size(), d.GetAllocator());
    d.AddMember(obj, obj_new, d.GetAllocator());
    return true;
  }

  static bool DeletJsonObject(std::string object)
  {
    d.RemoveMember(object.c_str());
    return true;
  }
};          // class UserInfoState

json::Document UserInfoState::d;
int UserInfoState::value = 404;
const char * UserInfoState::key1 = "FaceState";
const char * UserInfoState::key2 = "VoiceState";
const char * UserInfoState::file_name = "example.json";
json::Document d;

}           // namespace manager
}           // namespace cyberdog
#endif      // INTERFACE_HPP_
