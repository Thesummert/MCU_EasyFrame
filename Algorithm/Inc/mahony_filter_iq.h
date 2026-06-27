#if 0
#ifndef __MAHONY_FILTER_IQ_H__
#define __MAHONY_FILTER_IQ_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "ti/iqmath/include/IQmathLib.h"

typedef struct {
    _iq24 x;
    _iq24 y;
    _iq24 z;
} Axis3iq_t;

typedef struct
{
    float x;
    float y;
    float z;
}Axis3f_t;

typedef struct Mahony_Filter_IQ_t {
    // 输入参数
    _iq24 Kp, Ki;         // 比例和积分增益
    _iq24 dt;             // 采样时间间隔
    float dt_f;
    uint32_t cnt_last;    // 存储上次时间
    Axis3iq_t gyro, acc;  // 陀螺仪和加速度计数据
    Axis3f_t gyro_f, acc_f;

    // 过程参数
    _iq24 exInt, eyInt, ezInt;  // 积分误差累计
    _iq24 q0, q1, q2, q3;       // 四元数
    _iq24 rMat[3][3];           // 旋转矩阵

    // 输出参数
    float pitch_f, roll_f, yaw_f;           // 姿态角：俯仰角，滚转角，偏航角
    _iq24 pitch_iq, roll_iq, yaw_iq;  // 姿态角：俯仰角，滚转角，偏航角

    // 函数指针
    void (*Input)(struct Mahony_Filter_IQ_t *self, Axis3iq_t accel, Axis3iq_t gyro);
    void (*Update)(struct Mahony_Filter_IQ_t *self);
    void (*RotationMatUpdate)(struct Mahony_Filter_IQ_t *self);
    void (*CalFloat)(struct Mahony_Filter_IQ_t *self);
} Mahony_Filter_IQ_t;

void EasyFrame_Algorithm_MahonyIQ_Init(Mahony_Filter_IQ_t *self, float kp, float ki);

#ifdef __cplusplus
}
#endif

#endif /* __MAHONY_FILTER_IQ_H__ */
#endif
