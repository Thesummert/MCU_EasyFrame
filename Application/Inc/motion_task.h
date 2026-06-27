#ifndef __MOTION_TASK_H__
#define __MOTION_TASK_H__

#include <stdint.h>
#include "ti/iqmath/include/IQmathLib.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "mahony_filter_float.h"
#include "lsm6dsvtr.h"
#include "imu_calibration.h"
#include "comm_led.h"

typedef struct MotionTask_t {
    LSM6DSVTR_t *lsm6dsvtr;
    IMU_Calibration_t imu_calibration;  // 陀螺仪校准
    Mahony_Filter_Float_t mahony;       // Mahony滤波器
    EF_Device_Comm_LED_t status_led;    // 状态灯

    float q[4];                    // 估计的四元数
    float rot_q[4];                // 转化后的四元数
    float pitch_f, roll_f, yaw_f;  // 浮点角度
    float accel_f[3];              // 浮点加速度
    float gyro_f[3];               // 浮点角速度

    // 输入参数结构体
    Axis3f_t accel_axis;
    Axis3f_t gyro_axis;
    // 加速度在绝对坐标系中的向量形式
    float xn[3];
    float yn[3];
    float zn[3];

    float motion_accel_b[3];  // 机体系加速度
    float motion_accel_n[3];  // 地球系加速度

    uint32_t ins_time;       // ins任务计数
    float dt;                // 时间间隔
    uint32_t cnt_last;       // 时间戳
    float time_line;         // 时间线
    _Bool motion_flag;       // 是否可用标志位
    _Bool is_inited;         // 初始化标志位
    _Bool cali_static_flag;  // 静态校准标志位
} MotionTask_t;

_Bool EF_App_MotionTask_Init(MotionTask_t *self);
void EF_App_MotionTask_Run(MotionTask_t *self);
_Bool EF_App_MotionTask_Calibrate(MotionTask_t *self);
MotionTask_t *EF_MotionTask_GetPtr();

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_TASK_H__ */
