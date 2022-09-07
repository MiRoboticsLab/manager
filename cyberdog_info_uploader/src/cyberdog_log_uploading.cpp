// Copyright (c) 2022 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#include "cyberdog_info_uploader/cyberdog_log_uploading.hpp"

#include <stdlib.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <array>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>

namespace cyberdog
{
namespace manager
{
LogUploading::LogUploading(rclcpp::Node * ros_node)
: ros_node_(ros_node)
{
  upload_file_client_ =
    ros_node_->create_client<protocol::srv::BesHttpSendFile>("bes_http_send_file_srv");
}

bool LogUploading::CompressAndUploadLog(std::string & response)
{
  std::string file_name;
  bool result(false);
  if (compressLogFiles(file_name)) {
    if (uploadLog(file_name, response)) {
      result = true;
    }
    remove(file_name.c_str());
  }
  return result;
}

std::vector<std::string> LogUploading::getShellEcho(const std::string & cmd) const
{
  std::array<char, 128> buffer;
  std::vector<std::string> result;
  std::unique_ptr<FILE, decltype(& pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    std::string segment_str(buffer.data());
    size_t carriage_return_position = segment_str.find("\n");
    if (carriage_return_position != std::string::npos) {
      segment_str = segment_str.substr(0, carriage_return_position);
    }
    result.push_back(segment_str);
  }
  return result;
}

std::string LogUploading::getTimeStamp() const
{
  std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::stringstream time_ss;
  time_ss << std::put_time(std::localtime(&tt), "%C%m%d%H%M%S");
  return time_ss.str();
}

bool LogUploading::compressLogFiles(std::string & copressed_file_name) const
{
  std::string cmd("tar -zcvf ");
  copressed_file_name = "/home/mi/syslog" + getTimeStamp() + ".tgz";
  cmd += copressed_file_name;
  std::vector<std::string> file_list(getShellEcho("ls " + log_path_ + " | grep syslog"));
  if (file_list.empty()) {
    return false;
  }
  for (auto & each_file : file_list) {
    cmd += " " + each_file;
  }
  cmd = "cd " + log_path_ + " && " + cmd;
  return system(cmd.c_str()) == 0 ? true : false;
}

bool LogUploading::uploadLog(const std::string & compressed_file_name, std::string & response)
{
  if (!upload_file_client_->wait_for_service(std::chrono::seconds(3))) {
    WARN("bes_http_send_file_srv server not avalible!");
    return false;
  }
  auto req = std::make_shared<protocol::srv::BesHttpSendFile::Request>();
  req->method = protocol::srv::BesHttpSendFile::Request::HTTP_METHOD_POST;
  req->url = "";  // undetermined
  req->file_name = compressed_file_name;
  req->content_type = "application/x-tar";
  req->milsecs = 60000;  // 60s
  auto future_result = upload_file_client_->async_send_request(req);
  std::future_status status = future_result.wait_for(std::chrono::seconds(63));
  if (status == std::future_status::ready) {
    response = future_result.get()->data;
  } else {
    WARN("calling bes_http_send_file_srv service timeout!");
    return false;
  }
  return true;
}
}  // namespace manager
}  // namespace cyberdog
