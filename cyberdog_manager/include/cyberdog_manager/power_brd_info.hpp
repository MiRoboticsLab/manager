// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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

#ifndef CYBERDOG_MANAGER__POWER_BRD_INFO_HPP_
#define CYBERDOG_MANAGER__POWER_BRD_INFO_HPP_


#include <dirent.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/input.h>
#include <thread>
#include <string>
#include <memory>

#include "std_srvs/srv/trigger.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "protocol/srv/led_execute.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{
class PowerboardIndoNode
{
public:
  explicit PowerboardIndoNode(rclcpp::Node::SharedPtr node_ptr)
  : power_brd_info_node_(node_ptr)
  {
    power_brd_callback_group_ =
      power_brd_info_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    motion_excute_client_ =
      power_brd_info_node_->create_client<protocol::srv::MotionResultCmd>(
      "motion_result_cmd",
      rmw_qos_profile_services_default, power_brd_callback_group_);
    shutdown_client_ = power_brd_info_node_->create_client<std_srvs::srv::Trigger>(
      "poweroff",
      rmw_qos_profile_services_default, power_brd_callback_group_);
    led_excute_client_ =
      power_brd_info_node_->create_client<protocol::srv::LedExecute>(
      "led_execute",
      rmw_qos_profile_services_default, power_brd_callback_group_);
    Init();
  }

  void Init()
  {
    std::thread power_brd_thread(&PowerboardIndoNode::PowerBrdProcess, this, nullptr);
    power_brd_thread.detach();
  }

private:
  int openInput(const std::string & input_name)
  {
    int fd = -1;
    const char * dirname = "/dev/input";
    char devname[64];
    char * filename;
    DIR * dir;
    struct dirent * de;
    dir = opendir(dirname);
    if (dir == nullptr) {
      INFO("[power_bre_button]: opendir error code: %d,%s", errno, strerror(errno));
      return -1;
    }

    snprintf(devname, strlen(dirname) + 1, "%s", dirname);
    filename = devname + strlen(devname);
    *filename++ = '/';
    while ((de = readdir(dir))) {
      if (de->d_name[0] == '.' &&
        (de->d_name[1] == '\0' || (de->d_name[1] == '.' && de->d_name[2] == '\0')))
      {
        continue;
      }
      snprintf(filename, strlen(de->d_name) + 1, "%s", de->d_name);
      fd = open(devname, O_RDONLY | O_NONBLOCK);
      if (fd >= 0) {
        char name[80];
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
          name[0] = '\0';
        }

        if (!strcmp(name, input_name.c_str())) {
          INFO("get wanted input device(%s)", devname);
          break;
        } else {
          close(fd);
          fd = -1;
        }
      } else {
        INFO("[power_button]: input device open error code: %d,%s", errno, strerror(errno));
      }
    }
    closedir(dir);
    return fd;
  }

  int WaitPowerKeyEvent(int fd, int time_sec, int time_ms)
  {
    fd_set rdfds;
    struct timeval tv;
    int ret;
    FD_ZERO(&rdfds);
    FD_SET(fd, &rdfds);
    tv.tv_sec = time_sec;
    tv.tv_usec = time_ms * 1000;
    ret = select(fd + 1, &rdfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      return -1;
    } else if (ret == 0) {
      return 0;
    }
    return 1;
  }

  void PowerBrdProcess(void * pd)
  {
    (void)pd;
    struct input_event e, esync;
    int ret;
    // pthread_t id;

    int fd = openInput("gpio-keys");
    if (fd < 0) {
      INFO("[power_button]: open powerkey dev err:%s", strerror(errno));
      return;
    }

    while (1) {
      if (WaitPowerKeyEvent(fd, 1, 0) <= 0) {
        continue;
      }
      ret = read(fd, &e, sizeof(e));
      if (ret < 0) {
        continue;
      }
      read(fd, &esync, sizeof(e));  // read sync event
      if (e.type == EV_KEY) {
        if (e.value == 1) {  // press down
          if (WaitPowerKeyEvent(fd, 3, 0) <= 0) {
            INFO("[power_button]:powerkey long press");
            MotionContrl(102);  // Control the dog to lie down
            INFO("[power_button]: The dog has lied down");
            Shutdown();
            std::this_thread::sleep_for(std::chrono::seconds(1));
          } else {
            INFO("[power_button]:powerkey short press");
          }
        } else {}  // press up
      }
    }
  }

  bool MotionContrl(const int motion_id)
  {
    if (!motion_excute_client_->wait_for_service(std::chrono::seconds(3))) {
      ERROR("[power_button]:call led_excute server not avalible");
      return false;
    }
    auto request_motion = std::make_shared<protocol::srv::MotionResultCmd::Request>();
    request_motion->motion_id = motion_id;
    request_motion->cmd_source = 0;
    auto future_result_motion = motion_excute_client_->async_send_request(request_motion);
    std::future_status status_motion = future_result_motion.wait_for(std::chrono::seconds(10));
    if (status_motion == std::future_status::timeout) {
      ERROR("[power_button]:call motion service failed");
    }
    if (future_result_motion.get()->code != 0) {
      ERROR(
        "[power_button]:control motion fialed, error code is:%d",
        future_result_motion.get()->code);
    }
    return true;
  }

  bool Shutdown()
  {
    // 关机服务先返回结果，再执行状态切换及关机操作
    if (!shutdown_client_->wait_for_service(std::chrono::seconds(5))) {
      ERROR("call shutdown server not avalible");
      return false;
    }
    auto request_shutdown = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future_result_shutdown = shutdown_client_->async_send_request(request_shutdown);
    std::future_status status_shutdown = future_result_shutdown.wait_for(std::chrono::seconds(10));
    if (status_shutdown == std::future_status::timeout) {
      ERROR("call shutdown service failed");
    }
    if (future_result_shutdown.get()->success != true) {
      ERROR("shutdown fialed, error code is %d", future_result_shutdown.get()->success);
    }
    return 0;
  }

private:
  rclcpp::Node::SharedPtr power_brd_info_node_{nullptr};
  rclcpp::CallbackGroup::SharedPtr power_brd_callback_group_;
  rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_excute_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shutdown_client_;
  rclcpp::Client<protocol::srv::LedExecute>::SharedPtr led_excute_client_ {nullptr};
};
}  // namespace manager
}  // namespace cyberdog


#endif  // CYBERDOG_MANAGER__POWER_BRD_INFO_HPP_
