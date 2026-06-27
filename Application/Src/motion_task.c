#include "motion_task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "bsp_mspm0g_tim_base.h"
#include "comm_led.h"
#include "imu_calibration.h"
#include "mahony_filter_float.h"
#include "mahony_filter_iq.h"
#include "mcu_device.h"
#include "ti/iqmath/include/IQmathLib.h"
#include "mcu_config.h"
#include "ti_msp_dl_config.h"

// 是否进行静态校准
#define MOTION_TASK_STATIC_CALI 1

#define MAHONY_KP 1.0f
#define MAHONY_KI 0.1f

// 加速度计低通滤波
#define MOTION_TASK_ACCEL_LPF    0.3f
#define MOTION_TASK_ACCEL_LPF_IQ _IQ24(MOTION_TASK_ACCEL_LPF)

static MotionTask_t motion_task;

static const float GRAVITY_F[3] = {0, 0, 9.81f};
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

_Bool EF_App_MotionTask_Init(MotionTask_t *self) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in motion task init \r\n");
        return false;
    }
    memset(self, 0, sizeof(MotionTask_t));
    EF_Algorithm_Mahony_Init(&self->mahony, MAHONY_KP, MAHONY_KI);
    EF_Algorithm_IMU_Calibration_Init(&self->imu_calibration);
    self->lsm6dsvtr = EasyFrameDevice_Get_LSM6DSV();
    self->is_inited = true;
    EF_Device_Comm_LED_Init(&self->status_led, LED_PORT_PORT, LED_PORT_LED_PIN_PIN,
                            1);  // 初始化状态 LED
    EasyFrameSysTime_GetDeltaT(&self->cnt_last);
    self->time_line = EasyFrameSysTime_GetTimeline_s();
    self->status_led.Set(&self->status_led, EF_COMM_LED_ALWAYS_ON, 0, self->time_line);
    self->status_led.Show(&self->status_led, self->time_line);
    return true;
}

void EF_App_MotionTask_Run(MotionTask_t *self) {
    if (self->is_inited == false) {
        RTT_Print(0, "Motion task not inited \r\n");
        return;
    }
    // 如果还没校准 则触发一次校准
    if (MOTION_TASK_STATIC_CALI == 1 && self->cali_static_flag == 0) {
        // 设定校准时 状态指示高频闪烁
        self->time_line = EasyFrameSysTime_GetTimeline_s();
        self->status_led.Set(&self->status_led, EF_COMM_LED_TWINKLE, 20, self->time_line);
        self->status_led.Show(&self->status_led, self->time_line);
        EF_App_MotionTask_Calibrate(self);
        self->cali_static_flag = 1;
        self->time_line = EasyFrameSysTime_GetTimeline_s();  // 正常状态每秒闪烁2次
    }
    self->time_line = EasyFrameSysTime_GetTimeline_s();
    self->status_led.Show(&self->status_led, self->time_line);
    self->lsm6dsvtr->function.ReadIMU_NoDMA(self->lsm6dsvtr);  // 读取传感器
    // 复制到MotionTask结构体中
    self->accel_f[X] = self->lsm6dsvtr->data.accel_f[X];
    self->accel_f[Y] = self->lsm6dsvtr->data.accel_f[Y];
    self->accel_f[Z] = self->lsm6dsvtr->data.accel_f[Z];
    self->gyro_f[X] = self->lsm6dsvtr->data.gyro_f[X];
    self->gyro_f[Y] = self->lsm6dsvtr->data.gyro_f[Y];
    self->gyro_f[Z] = self->lsm6dsvtr->data.gyro_f[Z];

    self->accel_axis.x = self->accel_f[X];
    self->accel_axis.y = self->accel_f[Y];
    self->accel_axis.z = self->accel_f[Z];
    self->gyro_axis.x = self->gyro_f[X];
    self->gyro_axis.y = self->gyro_f[Y];
    self->gyro_axis.z = self->gyro_f[Z];

    self->dt = EasyFrameSysTime_GetDeltaT(&self->cnt_last);
    // 输入到Mahony滤波器中
    self->mahony.Input(&self->mahony, self->accel_axis, self->gyro_axis);
    self->mahony.Update(&self->mahony, self->dt);

    // 将输入写入MotionTask结构体
    self->q[0] = self->mahony.q0;
    self->q[1] = self->mahony.q1;
    self->q[2] = self->mahony.q2;
    self->q[3] = self->mahony.q3;

    // 转换机体坐标系 获取加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(GRAVITY_F, gravity_b, self->q);
    for (uint8_t i = 0; i < 3; i++) {  // 低通滤波
        self->motion_accel_b[i] =
            (self->accel_f[i] - gravity_b[i]) * self->dt / (MOTION_TASK_ACCEL_LPF + self->dt) +
            self->motion_accel_b[i] * MOTION_TASK_ACCEL_LPF / (MOTION_TASK_ACCEL_LPF + self->dt);
    }
    BodyFrameToEarthFrame(self->motion_accel_b, self->motion_accel_n, self->q);
    // 死区处理
    for (uint8_t i = 0; i < 3; i++){
        if (fabsf(self->motion_accel_n[i]) < 0.2f) {
            self->motion_accel_n[i] = 0.0f;
        }
    }

    // 当运行次数超过3000次 可以认为Mahony收敛
    if (self->ins_time > 3000) {
        self->status_led.Set(&self->status_led, EF_COMM_LED_TWINKLE, 2, self->time_line);
        self->motion_flag = true;
        self->pitch_f = self->mahony.pitch_f;
        self->roll_f = self->mahony.roll_f;
        self->yaw_f = self->mahony.yaw_f;
    } else {
        self->ins_time++;
    }
    EasyFrameSysTime_Delay(0.001);
}

_Bool EF_App_MotionTask_Calibrate(MotionTask_t *self) {
    if (self == NULL) {
        return false;
    }
    // 等待读书稳定
    for (uint16_t i = 0; i < 500; i++) {
        self->lsm6dsvtr->function.ReadIMU_NoDMA(self->lsm6dsvtr);
    }
    float time = EasyFrameSysTime_GetTimeline_s();  // 获取时间戳
    IMU_Cali_Result_e imu_calibration_result = IMU_CALIBRATION_NEXT;
    self->imu_calibration.StartStaticCali(&self->imu_calibration, time);
    do {
        self->lsm6dsvtr->function.ReadIMU_NoDMA(self->lsm6dsvtr);
        time = EasyFrameSysTime_GetTimeline_s();  // 获取时间戳
        imu_calibration_result =
            self->imu_calibration.AddSamples(&self->imu_calibration, 0, self->lsm6dsvtr->data.gyro_f,
                                             self->lsm6dsvtr->data.accel_f, time);
        self->status_led.Show(&self->status_led, time);
        // EasyFrameSysTime_Delay(0.005f);
    } while (imu_calibration_result == IMU_CALIBRATION_NEXT);
    if (imu_calibration_result == IMU_CALIBRATION_SUCCESS) {
        // 写入校准数据
        // self->imu_calibration.static_cali.accel[Z] =
        //     0.0f;  // 加速度计Z轴不校准 此后对于坐标系中的Z方向 都不校准
        for (uint8_t i = 0; i < 3; i++) {
            self->imu_calibration.static_cali.accel[i] =
                0.0f;  // 难以获得真正的水平环境 所以在此处不对加速度计校准
        }
        self->lsm6dsvtr->function.SetOffest(self->lsm6dsvtr, self->imu_calibration.static_cali.accel,
                                            self->imu_calibration.static_cali.gyro);
        return true;
    } else {
        return false;
    }
}

MotionTask_t *EF_MotionTask_GetPtr() { return &motion_task; }

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q) {
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] =
        2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q) {
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] =
        2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
