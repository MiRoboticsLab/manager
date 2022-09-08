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

#ifndef CYBERDOG_INFO_UPLOADER__CYBERDOG_INFO_UPLOADER_HPP_
#define CYBERDOG_INFO_UPLOADER__CYBERDOG_INFO_UPLOADER_HPP_

#include <string>
#include "cyberdog_info_uploader/cyberdog_log_uploading.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cyberdog
{
namespace manager
{
class InfoUploader
{
public:
  explicit InfoUploader(rclcpp::Node * ros_node);
  bool UploadLog(std::string & response);

private:
  LogUploading log_uploading_;
};
}  // namespace manager
}  // namespace cyberdog
#endif  // CYBERDOG_INFO_UPLOADER__CYBERDOG_INFO_UPLOADER_HPP_