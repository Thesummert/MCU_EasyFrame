#ifndef __MAHONY_FILTER_FLOAT_H__
#define __MAHONY_FILTER_FLOAT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

typedef struct
{
    float x;
    float y;
    float z;
}Axis3f_t;

typedef struct  Mahony_Filter_Float_t{
    // 输入参数
    float Kp, Ki;         // 比例和积分增益
    float dt;             // 采样时间间隔
    uint32_t cnt_last;    // 存储上次时间
    Axis3f_t gyro_f, acc_f;

    // 过程参数
    float exInt, eyInt, ezInt;  // 积分误差累计
    float q0, q1, q2, q3;       // 四元数
    float rMat[3][3];           // 旋转矩阵

    // 输出参数
    float pitch_f, roll_f, yaw_f;           // 姿态角：俯仰角，滚转角，偏航角
                                            //
    _Bool is_inited; // 是否完成初始化

    // 函数指针
    _Bool (*Input)(struct Mahony_Filter_Float_t *self, Axis3f_t accel, Axis3f_t gyro);
    _Bool (*Update)(struct Mahony_Filter_Float_t *self, float dt);
} Mahony_Filter_Float_t;

_Bool EF_Algorithm_Mahony_Init(Mahony_Filter_Float_t *self, float kp, float ki);

#ifdef __cplusplus
}
#endif

#endif /* __MAHONY_FILTER_FLOAT_H__ */
