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

#ifndef CYBERDOG_INFO_UPLOADER__CYBERDOG_LOG_UPLOADING_HPP_
#define CYBERDOG_INFO_UPLOADER__CYBERDOG_LOG_UPLOADING_HPP_

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"
#include "protocol/srv/bes_http_send_file.hpp"

namespace cyberdog
{
namespace manager
{
class LogUploading
{
public:
  explicit LogUploading(rclcpp::Node::SharedPtr node);
  bool CompressAndUploadLog(std::string & response);

private:
  std::vector<std::string> getShellEcho(const std::string & cmd) const;
  std::string getTimeStamp() const;
  bool compressLogFiles(std::string & copressed_file_name) const;
  bool uploadLog(const std::string & compressed_file_name, std::string & response);
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Client<protocol::srv::BesHttpSendFile>::SharedPtr client_ptr_;
  const std::string log_path_ {"/var/log/"};
  const std::string upload_url_ {"device/system/log"};

  LOGGER_MINOR_INSTANCE("LogUploading");
};
}  // namespace manager
}  // namespace cyberdog
#endif  // CYBERDOG_INFO_UPLOADER__CYBERDOG_LOG_UPLOADING_HPP_
