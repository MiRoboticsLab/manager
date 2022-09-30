#ifndef PM_IF_H_
        #define PM_IF_H_


#ifdef __cplusplus
extern "C"
{
#endif


enum PM_DEV
{
//bit 0-7 camera
  PM_CAM_REALSNS  = 1 << 0,// 深度相机
  PM_CAM_AI       = 1 << 1,// AI相机组件
  PM_CAM_ALL      = 0Xff,

//bit 8-11  motion
  PM_MOTION       = 1 << 8,//运动组件
// other
  PM_GPS          = 1 << 12,//GPS 组件
  PM_LIDAR        = 1 << 13,// 激光雷达组件,
  PM_TOF          = 1 << 14,// TOF 组件
  PM_ULTRA        = 1 << 15,//超声波组件
};

int PmRequest(unsigned int devs, unsigned int * err);
int PmRelease(unsigned int devs, unsigned int * err);
int PmQuery(unsigned int dev);


enum PM_SYS
{
  PM_SYS_SLEEP=0,
  PM_SYS_REBOOT,
  PM_SYS_SHUTDOWN,
};

int PmSysRequest(int op);


#ifdef __cplusplus
}
#endif


#endif
