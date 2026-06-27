#include "ti/devices/msp/peripherals/hw_mathacl.h"
#if 0
/**
 * mahony_filter_iq.c 
 *
 * @brief 这是mahony iqmath版本 默认使用24位IQ MATH
 *
 */


#include "mahony_filter_iq.h"
#include "string.h"
#include "ti/iqmath/include/IQmathLib.h"
#include "mcu_config.h"
#include "bsp_mspm0g_tim_base.h"

// function define
static void Input(Mahony_Filter_IQ_t *self, Axis3iq_t accel, Axis3iq_t gyro);
static void Update(Mahony_Filter_IQ_t *self);
static void RotationMatUpdate(Mahony_Filter_IQ_t *self);
static void CalFloat(Mahony_Filter_IQ_t *self);

/**
 * @brief Mahony滤波器初始化函数
 * @param self Mahony滤波器结构体指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @note 初始化滤波器参数、时间戳和函数指针
 */
void EasyFrame_Algorithm_MahonyIQ_Init(Mahony_Filter_IQ_t *self, float kp, float ki)
{
    memset(self, 0, sizeof(Mahony_Filter_IQ_t));
    self->q0 = _IQ24(1.0);
    self->Kp = _IQ24(kp);
    self->Ki = _IQ24(ki);

    // 初始化时间戳
    EasyFrameSysTime_GetDeltaT(&self->cnt_last);

    // 初始化函数指针
    self->Input = Input;
    self->Update = Update;
    self->RotationMatUpdate = RotationMatUpdate;
    self->CalFloat = CalFloat;
}

/**
 * @brief 输入加速度计和陀螺仪数据
 * @param self Mahony滤波器结构体指针
 * @param accel 加速度计数据
 * @param gyro 陀螺仪数据
 */
static void Input(Mahony_Filter_IQ_t *self, Axis3iq_t accel, Axis3iq_t gyro)
{
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happened in Mahony Init \r\n");
        return;
    }
    self->acc = accel;
    self->gyro = gyro;
}

/**
 * @brief Mahony滤波器主更新函数
 * @param self Mahony滤波器结构体指针
 * @note 完成姿态估算的主要计算，包括误差积分、PI修正、四元数更新和欧拉角计算
 */
static void Update(Mahony_Filter_IQ_t *self)
{
    _iq24 normalise;
    _iq24 ex, ey, ez;
    _iq24 halfT;
    self->dt_f = EasyFrameSysTime_GetDeltaT(&self->cnt_last);
    self->dt = _IQ24(self->dt_f);
    

    /* 单位化加速计测量值 */
    normalise = _IQ24sqrt(
        _IQ24mpy(self->acc.x, self->acc.x) + 
        _IQ24mpy(self->acc.y, self->acc.y) + 
        _IQ24mpy(self->acc.z, self->acc.z));

    self->acc.x = _IQ24div(self->acc.x, normalise);
    self->acc.y = _IQ24div(self->acc.y, normalise);
    self->acc.z = _IQ24div(self->acc.z, normalise);
    self->acc_f.x = _IQ24toF(self->acc.x);
    self->acc_f.y = _IQ24toF(self->acc.y);
    self->acc_f.z = _IQ24toF(self->acc.z);

    /* 加速计读取的方向与重力加速度方向的差值，用向量叉乘计算 */
    ex = _IQ24mpy(self->acc.y, self->rMat[2][2]) - 
         _IQ24mpy(self->acc.z, self->rMat[2][1]);
    ey = _IQ24mpy(self->acc.z, self->rMat[2][0]) - 
         _IQ24mpy(self->acc.x, self->rMat[2][2]);
    ez = _IQ24mpy(self->acc.x, self->rMat[2][1]) - 
         _IQ24mpy(self->acc.y, self->rMat[2][0]);

    /* 误差累计，与积分常数相乘 */
    self->exInt += _IQ24mpy(_IQ24mpy(self->Ki, ex), self->dt);
    self->eyInt += _IQ24mpy(_IQ24mpy(self->Ki, ey), self->dt);
    self->ezInt += _IQ24mpy(_IQ24mpy(self->Ki, ez), self->dt);

    /* 用叉积误差做PI修正陀螺零偏，抵消陀螺读数中的偏移量 */
    self->gyro.x += _IQ24mpy(self->Kp, ex) + self->exInt;
    self->gyro.y += _IQ24mpy(self->Kp, ey) + self->eyInt;
    self->gyro.z += _IQ24mpy(self->Kp, ez) + self->ezInt;
    self->gyro_f.x = _IQ24toF(self->gyro.x);
    self->gyro_f.y = _IQ24toF(self->gyro.y);
    self->gyro_f.z = _IQ24toF(self->gyro.z);

    /* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
    _iq24 q0Last = self->q0;
    _iq24 q1Last = self->q1;
    _iq24 q2Last = self->q2;
    _iq24 q3Last = self->q3;
    halfT = _IQ24mpy(self->dt, _IQ24(0.5));
    self->q0 += _IQ24mpy(
        -_IQ24mpy(q1Last, self->gyro.x) - _IQ24mpy(q2Last, self->gyro.y) - _IQ24mpy(q3Last, self->gyro.z),
        halfT);
    self->q1 += _IQ24mpy(
        _IQ24mpy(q0Last, self->gyro.x) + _IQ24mpy(q2Last, self->gyro.z) - _IQ24mpy(q3Last, self->gyro.y),
        halfT);
    self->q2 += _IQ24mpy(
        _IQ24mpy(q0Last, self->gyro.y) - _IQ24mpy(q1Last, self->gyro.z) + _IQ24mpy(q3Last, self->gyro.x),
        halfT);
    self->q3 += _IQ24mpy(
        _IQ24mpy(q0Last, self->gyro.z) + _IQ24mpy(q1Last, self->gyro.y) - _IQ24mpy(q2Last, self->gyro.x),
        halfT);

    /* 单位化四元数 */
    normalise = _IQ24sqrt(
        _IQ24mpy(self->q0, self->q0) + 
        _IQ24mpy(self->q1, self->q1) + 
        _IQ24mpy(self->q2, self->q2) + 
        _IQ24mpy(self->q3, self->q3));

    self->q0 = _IQ24div(self->q0, normalise);
    self->q1 = _IQ24div(self->q1, normalise);
    self->q2 = _IQ24div(self->q2, normalise);
    self->q3 = _IQ24div(self->q3, normalise);

    self->RotationMatUpdate(self); // 旋转矩阵 机体坐标系转换

    /* 计算roll pitch yaw欧拉角 */
    self->pitch_iq = -_IQ24asin(self->rMat[2][0]);
    self->roll_iq = _IQ24atan2(self->rMat[2][1], self->rMat[2][2]);
    self->yaw_iq = _IQ24atan2(self->rMat[1][0], self->rMat[0][0]);

}

/**
 * @brief 根据四元数更新旋转矩阵
 * @param self Mahony滤波器结构体指针
 */
static void RotationMatUpdate(Mahony_Filter_IQ_t *self)
{
    _iq24 q1q1 = _IQ24mpy(self->q1, self->q1);
    _iq24 q2q2 = _IQ24mpy(self->q2, self->q2);
    _iq24 q3q3 = _IQ24mpy(self->q3, self->q3);

    _iq24 q0q1 = _IQ24mpy(self->q0, self->q1);
    _iq24 q0q2 = _IQ24mpy(self->q0, self->q2);
    _iq24 q0q3 = _IQ24mpy(self->q0, self->q3);
    _iq24 q1q2 = _IQ24mpy(self->q1, self->q2);
    _iq24 q1q3 = _IQ24mpy(self->q1, self->q3);
    _iq24 q2q3 = _IQ24mpy(self->q2, self->q3);

    self->rMat[0][0] = _IQ24(1.0) - _IQ24mpy(_IQ24(2.0), q2q2) - _IQ24mpy(_IQ24(2.0), q3q3);
    self->rMat[0][1] = _IQ24mpy(_IQ24(2.0), (q1q2 - q0q3));
    self->rMat[0][2] = _IQ24mpy(_IQ24(2.0), (q1q3 + q0q2));

    self->rMat[1][0] = _IQ24mpy(_IQ24(2.0), (q1q2 + q0q3));
    self->rMat[1][1] = _IQ24(1.0) - _IQ24mpy(_IQ24(2.0), q1q1) - _IQ24mpy(_IQ24(2.0), q3q3);
    self->rMat[1][2] = _IQ24mpy(_IQ24(2.0), (q2q3 - q0q1));

    self->rMat[2][0] = _IQ24mpy(_IQ24(2.0), (q1q3 - q0q2));
    self->rMat[2][1] = _IQ24mpy(_IQ24(2.0), (q2q3 + q0q1));
    self->rMat[2][2] = _IQ24(1.0) - _IQ24mpy(_IQ24(2.0), q1q1) - _IQ24mpy(_IQ24(2.0), q2q2);
}

/**
 * @brief 计算欧拉角的浮点数值
 * @param self Mahony滤波器结构体指针
 * @note 由于浮点数计算量大，需要手动调用
 */
static void CalFloat(Mahony_Filter_IQ_t *self)
{
    // 由于浮点数计算量大 此处获取浮点数版本需要手动调用
    self->pitch_f = _IQ24toF(self->pitch_iq);
    self->roll_f = _IQ24toF(self->roll_iq);
    self->yaw_f = _IQ24toF(self->yaw_iq);
}
#endif

