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
#ifndef LOW_POWER_CONSUMPTION__LOW_POWER_CONSUMPTION_HPP_
#define LOW_POWER_CONSUMPTION__LOW_POWER_CONSUMPTION_HPP_

#include <dlfcn.h>
#include "low_power_consumption/pm_if.h"
#include "cyberdog_common/cyberdog_log.hpp"

namespace cyberdog
{
namespace manager
{

typedef int (* Func_PmRequest)(unsigned int devs, unsigned int * err);
typedef int (* Func_PmRelease)(unsigned int devs, unsigned int * err);
typedef int (* Func_PmQuery)(unsigned int dev);
typedef int (* Func_PmSysRequest)(int op);

class LowPowerConsumption
{
public:
  LowPowerConsumption()
  {
    Dll_Open();
  }
  ~LowPowerConsumption()
  {
    Dll_Close();
  }
  int LpcRequest(PM_DEV devs, unsigned int * err)
  {
    return Pm_Request(devs, err);
  }
  int LpcRelease(PM_DEV devs, unsigned int * err)
  {
    return Pm_Release(devs, err);
  }
  int LpcQuery(PM_DEV dev)
  {
    return Pm_Query(dev);
  }
  int LpcSysRequest(PM_SYS op)
  {
    return Pm_SysRequest(op);
  }

private:
  bool Dll_Open()
  {
    dll_handle = dlopen(dll_path, RTLD_LAZY);
    if (!dll_handle) {
      INFO("dlopen get error: %s", dlerror() );
      dll_open = false;
    } else {
      INFO("dll open.");
      dll_open = true;
      pm_request = (Func_PmRequest)dlsym(dll_handle, "PmRequest");
      pm_release = (Func_PmRelease)dlsym(dll_handle, "PmRelease");
      pm_query = (Func_PmQuery)dlsym(dll_handle, "PmQuery");
      pm_sys_request = (Func_PmSysRequest)dlsym(dll_handle, "PmSysRequest");
      if (pm_request) {
        INFO("pm_request get.");
      }
      if (pm_release) {
        INFO("pm_release get.");
      }
      if (pm_query) {
        INFO("pm_query get.");
      }
      if (pm_sys_request) {
        INFO("pm_sys_request get.");
      }
    }
    return dll_open;
  }
  void Dll_Close()
  {
    if (dll_open) {
      if (dll_handle) {
        dlclose(dll_handle);
      }
    }
    dll_open = false;
  }
  int Pm_Request(unsigned int devs, unsigned int * err)
  {
    if (pm_request) {
      INFO("Pm_Request enter!----");
      return pm_request(devs, err);
    }
    return 255;
  }
  int Pm_Release(unsigned int devs, unsigned int * err)
  {
    if (pm_release) {
      INFO("Pm_Release enter!-----");
      return pm_release(devs, err);
    }
    return 255;
  }
  int Pm_Query(unsigned int dev)
  {
    if (pm_query) {
      INFO("Pm_Query enter!");
      return pm_query(dev);
    }
    return 255;
  }
  int Pm_SysRequest(int op)
  {
    if (pm_sys_request) {
      return pm_sys_request(op);
    }
    return 255;
  }

private:
  const char * dll_path = "/usr/lib/libpm_client.so";
  bool dll_open {false};
  void * dll_handle {nullptr};
  Func_PmRequest pm_request;
  Func_PmRelease pm_release;
  Func_PmQuery pm_query;
  Func_PmSysRequest pm_sys_request;
};    // class LowPowerConsumption
}  // namespace manager
}  // namespace cyberdog

#endif  // LOW_POWER_CONSUMPTION__LOW_POWER_CONSUMPTION_HPP_
