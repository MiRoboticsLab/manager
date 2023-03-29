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
#ifndef LOW_POWER_CONSUMPTION__PM_IF_H_
#define LOW_POWER_CONSUMPTION__PM_IF_H_


#ifdef __cplusplus
extern "C"
{
#endif


enum PM_DEV
{
  // bit 0-7 camera
  PM_CAM_REALSNS  = 1 << 0,  // 深度相机
  PM_CAM_AI       = 1 << 1,  // AI相机组件
  PM_CAM_ALL      = 0Xff,

  // bit 8-11  motion
  PM_MOTION       = 1 << 8,  // 运动组件
  // other
  PM_GPS          = 1 << 12,  // GPS 组件
  PM_LIDAR        = 1 << 13,  // 激光雷达组件,
  PM_TOF          = 1 << 14,  // TOF 组件
  PM_ULTRA        = 1 << 15,  // 超声波组件
  PM_ALL          = 0xffffffff,
  PM_ALL_NO_MOTION = PM_ALL & (~PM_MOTION),
  PM_ALL_NO_TOF = PM_ALL & (~PM_TOF),
  PM_NO_MOTION_TOF = PM_ALL & (~PM_MOTION) & (~PM_TOF)
};

int PmRequest(unsigned int devs, unsigned int * err);
int PmRelease(unsigned int devs, unsigned int * err);
int PmQuery(unsigned int dev);


enum PM_SYS
{
  PM_SYS_SLEEP = 0,
  PM_SYS_REBOOT,
  PM_SYS_SHUTDOWN,
};

int PmSysRequest(int op);


#ifdef __cplusplus
}
#endif


#endif  // LOW_POWER_CONSUMPTION__PM_IF_H_
