#include "mahony_filter_float.h"
#include <stdlib.h>
#include <string.h>
#include "dsp/fast_math_functions.h"
#include "mcu_config.h"
#include "stdbool.h"

static _Bool Input(Mahony_Filter_Float_t *self, Axis3f_t accel, Axis3f_t gyro);
static _Bool Update(Mahony_Filter_Float_t *self, float dt);
static void RotationMatrix_update(Mahony_Filter_Float_t *self);

/**
 * @brief Mahony滤波器初始化函数（浮点版）
 * @param self Mahony滤波器结构体指针
 * @param kp 比例增益，用于陀螺仪零偏修正
 * @param ki 积分增益，用于陀螺仪零偏累计误差修正
 * @note 初始化后四元数初值为(0,1,0,0)，表示绕Y轴旋转180度（俯仰角0度）
 * @retval true 初始化成功
 * @retval false 初始化失败（空指针）
 */
_Bool EF_Algorithm_Mahony_Init(Mahony_Filter_Float_t *self, float kp, float ki) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in mahony init \r\n");
        return false;
    }
    memset(self, 0, sizeof(Mahony_Filter_Float_t));
    self->Kp = kp;
    self->Ki = ki;
    self->q1 = 1.0f;
    self->is_inited = true;
    // 设定函数指针
    self->Input = Input;
    self->Update = Update;
    return true;
}

/**
 * @brief 输入加速度计和陀螺仪数据
 * @param self Mahony滤波器结构体指针
 * @param accel 加速度计三轴数据（单位：m/s^2）
 * @param gyro 陀螺仪三轴数据（单位：度/秒）
 * @retval true 输入成功
 * @retval false 输入失败（空指针或未初始化）
 */
static _Bool Input(Mahony_Filter_Float_t *self, Axis3f_t accel, Axis3f_t gyro) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in mahony input \r\n");
        return false;
    }
    if (self->is_inited == 0) {
        RTT_Print(0, "Mahony not inited \r\n");
        return false;
    }
    self->gyro_f = gyro;
    self->acc_f = accel;

    return true;
}

/**
 * @brief Mahony姿态解算主循环（浮点版）
 * @param self Mahony滤波器结构体指针
 * @param dt 采样时间间隔（单位：秒）
 * @note 算法流程：加速度计归一化 -> 叉积误差计算 -> PI修正陀螺仪 -> 四元数积分更新 -> 旋转矩阵更新 -> 欧拉角计算
 * @retval true 解算成功
 * @retval false 解算失败（空指针或未初始化）
 */
static _Bool Update(Mahony_Filter_Float_t *self, float dt) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in mahony run \r\n");
        return false;
    }
    if (self->is_inited == 0) {
        RTT_Print(0, "Mahony not inited \r\n");
        return false;
    }
    self->dt = dt;
    float normalise;
    float ex, ey, ez;

    /*角速度，度转弧度*/
    //    mahony_filter->gyro.x *= DEG2RAD;        /* 度转弧度 */
    //    mahony_filter->gyro.y *= DEG2RAD;
    //    mahony_filter->gyro.z *= DEG2RAD;;

    /*单位化加速计测量值*/
    arm_sqrt_f32(self->acc_f.x * self->acc_f.x + self->acc_f.y * self->acc_f.y +
             self->acc_f.z * self->acc_f.z, &normalise);
    //
    // // 动态控制加速度计权重
    // err = fabsf(normalise - GRAVITY);
    // weight = expf(-(err * err) / (2.0f * ACC_WEIGHT * ACC_WEIGHT));
    // if (weight < 0.05f) weight = 0.05f;

    self->acc_f.x = self->acc_f.x / normalise;
    self->acc_f.y = self->acc_f.y / normalise;
    self->acc_f.z = self->acc_f.z / normalise;

    /*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
    // vx=self->rMat[2][0]=2.0f * (q1q3 + -q0q2)
    // vy=self->rMat[2][1]=2.0f * (q2q3 - -q0q1)
    // vz=self->rMat[2][2]=1.0f - 2.0f * q1q1 - 2.0f * q2q2
    // ex = (self->acc_f.y * self->rMat[2][2] - self->acc_f.z *
    // self->rMat[2][1]) * weight; ey = (self->acc_f.z * self->rMat[2][0] -
    // self->acc_f.x * self->rMat[2][2]) * weight; ez = (self->acc_f.x *
    // self->rMat[2][1] - self->acc_f.y * self->rMat[2][0]) * weight;

    ex = (self->acc_f.y * self->rMat[2][2] -
          self->acc_f.z * self->rMat[2][1]);
    ey = (self->acc_f.z * self->rMat[2][0] -
          self->acc_f.x * self->rMat[2][2]);
    ez = (self->acc_f.x * self->rMat[2][1] -
          self->acc_f.y * self->rMat[2][0]);

    /*误差累计，与积分常数相乘*/
    self->exInt += self->Ki * ex * dt;
    self->eyInt += self->Ki * ey * dt;
    self->ezInt += self->Ki * ez * dt;

    /*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
    self->gyro_f.x += self->Kp * ex + self->exInt;
    self->gyro_f.y += self->Kp * ey + self->eyInt;
    self->gyro_f.z += self->Kp * ez + self->ezInt;

    /* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
    float q0Last = self->q0;
    float q1Last = self->q1;
    float q2Last = self->q2;
    float q3Last = self->q3;
    float halfT = self->dt * 0.5f;
    self->q0 += (-q1Last * self->gyro_f.x - q2Last * self->gyro_f.y -
                 q3Last * self->gyro_f.z) *
                halfT;
    self->q1 += (q0Last * self->gyro_f.x + q2Last * self->gyro_f.z -
                 q3Last * self->gyro_f.y) *
                halfT;
    self->q2 += (q0Last * self->gyro_f.y - q1Last * self->gyro_f.z +
                 q3Last * self->gyro_f.x) *
                halfT;
    self->q3 += (q0Last * self->gyro_f.z + q1Last * self->gyro_f.y -
                 q2Last * self->gyro_f.x) *
                halfT;

    /*单位化四元数*/
    normalise = sqrt(self->q0 * self->q0 + self->q1 * self->q1 +
                     self->q2 * self->q2 + self->q3 * self->q3);

    self->q0 = self->q0 / normalise;
    self->q1 = self->q1 / normalise;
    self->q2 = self->q2 / normalise;
    self->q3 = self->q3 / normalise;

    // 旋转矩阵
    RotationMatrix_update(self);

    // 提取角度
    self->pitch_f = -asinf(self->rMat[2][0]);
    self->roll_f = atan2f(self->rMat[2][1], self->rMat[2][2]);
    self->yaw_f = atan2f(self->rMat[1][0], self->rMat[0][0]);

    return true;
}

/**
 * @brief 根据四元数更新旋转矩阵
 * @param self Mahony滤波器结构体指针
 * @note 旋转矩阵用于后续叉积误差计算和欧拉角提取，表示机体坐标系到地理坐标系的旋转
 */
static void RotationMatrix_update(Mahony_Filter_Float_t *self) {
    float q1q1 = self->q1 * self->q1;
    float q2q2 = self->q2 * self->q2;
    float q3q3 = self->q3 * self->q3;

    float q0q1 = self->q0 * self->q1;
    float q0q2 = self->q0 * self->q2;
    float q0q3 = self->q0 * self->q3;
    float q1q2 = self->q1 * self->q2;
    float q1q3 = self->q1 * self->q3;
    float q2q3 = self->q2 * self->q3;

    self->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    self->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    self->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    self->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    self->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    self->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    self->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    self->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    self->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}
