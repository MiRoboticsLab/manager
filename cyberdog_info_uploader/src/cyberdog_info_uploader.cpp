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

#include "cyberdog_info_uploader/cyberdog_info_uploader.hpp"
#include <string>

namespace cyberdog
{
namespace manager
{
InfoUploader::InfoUploader(rclcpp::Node::SharedPtr node)
: node_ptr_(node), log_uploading_(node)
{}
bool InfoUploader::UploadLog(std::string & response)
{
  return log_uploading_.CompressAndUploadLog(response);
}
}  // namespace manager
}  // namespace cyberdog
