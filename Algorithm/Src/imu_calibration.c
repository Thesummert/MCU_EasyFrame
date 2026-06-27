/**
 *
 * @brief 陀螺仪校准算法
 *
 */
#include "imu_calibration.h"
#include <stdbool.h>
#include <stdint.h>
#include "mcu_config.h"
#include "string.h"

// 校准静态校准采样时间
#define IMU_STATIC_CALI_SAMPLE_TIME 5.0f
// 静态校准所需的信任采集次数
#define IMU_STATIC_CALI_SAMPLE_NUM 6000

// 校准过程中允许的最大误差
#define IMU_CALI_ACCEL_DIFF 0.2f
#define IMU_CALI_GYRO_DIFF  0.2f

static void StartStaticCali(IMU_Calibration_t *self, float start_time);
static IMU_Cali_Result_e AddSamples(IMU_Calibration_t *self, uint8_t stat_or_dyna, float gyro[3],
                                    float accel[3], float run_time);

/**
 * @brief 初始化IMU校准结构体
 * @param self 指向IMU_Calibration_t结构体的指针
 * @retval true 初始化成功
 * @retval false 初始化失败（指针为空）
 */
_Bool EF_Algorithm_IMU_Calibration_Init(IMU_Calibration_t *self) {
    if (self == NULL) {
        RTT_Print(0, "Null point error happen in imu calibration init \r\n");
        return false;
    }
    memset(self, 0, sizeof(IMU_Calibration_t));
    // 添加函数指针
    self->StartStaticCali = StartStaticCali;
    self->AddSamples = AddSamples;
    return true;
}

/**
 * @brief 启动静态校准，初始化相关参数
 * @param self 指向IMU_Calibration_t结构体的指针
 * @param start_time 校准开始时间（单位：秒）
 */
static void StartStaticCali(IMU_Calibration_t *self, float start_time) {
    if (self == NULL) {
        RTT_Print(0, "Null point error happen in imu calibration init \r\n");
        return;
    }
    // 清空数值
    for (uint8_t i = 0; i < 3; i++) {
        self->static_cali.accel[i] = 0.0f;
        self->static_cali.gyro[i] = 0.0f;
        self->static_cali.accel_sum[i] = 0.0f;
        self->static_cali.gyro_sum[i] = 0.0f;
    }
    self->static_cali.cali_flag = false;
    // 计算所需时间
    self->static_cali.sample_start = start_time;
    self->static_cali.sample_except_end = start_time + IMU_STATIC_CALI_SAMPLE_TIME;
}

/**
 * @brief 添加采样数据并进行静态校准判断
 * @param self 指向IMU_Calibration_t结构体的指针
 * @param stat_or_dyna 0表示静态校准，其他为动态
 * @param gyro 陀螺仪三轴数据数组
 * @param accel 加速度计三轴数据数组
 * @param run_time 当前运行时间（单位：秒）
 * @retval IMU_CALIBRATION_SUCCESS 校准成功
 * @retval IMU_CALIBRATION_Failed 校准失败
 * @retval IMU_CALIBRATION_NEXT 继续采样
 */
static IMU_Cali_Result_e AddSamples(IMU_Calibration_t *self, uint8_t stat_or_dyna, float gyro[3],
                                    float accel[3], float run_time) {
    if (self == NULL) {
        return IMU_CALIBRATION_NULLPOINTER_ERROR;
    }
    // 输入0 是输入静态校准数据
    if (stat_or_dyna == 0) {
        if (self->static_cali.first_run == 0) {
            for (uint8_t i = 0; i < 3; i++){
                // 首次运行设置最大值和最小值
                self->static_cali.accel_min[i] = accel[i];
                self->static_cali.accel_max[i] = accel[i];
                self->static_cali.gyro_min[i] = gyro[i];
                self->static_cali.gyro_max[i] = gyro[i];
                self->static_cali.first_run = 1;
            }
        }
        for (uint8_t i = 0; i < 3; i++) {
            self->static_cali.accel_sum[i] += accel[i];
            self->static_cali.gyro_sum[i] += gyro[i];

            // 存储最大值和最小值 用于确定是否在校准的时候发生运动
            if (self->static_cali.accel_min[i] > accel[i]) {
                self->static_cali.accel_min[i] = accel[i];
            }
            if (self->static_cali.accel_max[i] < accel[i]) {
                self->static_cali.accel_max[i] = accel[i];
            }

            if (self->static_cali.gyro_min[i] > gyro[i]) {
                self->static_cali.gyro_min[i] = gyro[i];
            }
            if (self->static_cali.gyro_max[i] < gyro[i]) {
                self->static_cali.gyro_max[i] = gyro[i];
            }

            // 如果超过阈值 则表示运动不合格
            if (self->static_cali.accel_max[i] - self->static_cali.accel_min[i] > IMU_CALI_ACCEL_DIFF ||
                self->static_cali.gyro_max[i] - self->static_cali.gyro_min[i] > IMU_CALI_GYRO_DIFF) {
                RTT_Print(0, "IMU calibration failed \r\n");
                return IMU_CALIBRATION_Failed;
            }
        }

        self->static_cali.sample_time++;
        // 达到目标时间 退出校准
        // 判断数量是否达标
        if (run_time > self->static_cali.sample_except_end) {
            if (self->static_cali.sample_time > IMU_STATIC_CALI_SAMPLE_NUM) {
                // 校准成功计算平均值获得漂移值
                for (uint8_t i = 0; i < 3; i++){
                    self->static_cali.accel[i] = self->static_cali.accel_sum[i] / self->static_cali.sample_time;
                    self->static_cali.gyro[i] = self->static_cali.gyro_sum[i] / self->static_cali.sample_time;
                }
                return IMU_CALIBRATION_SUCCESS;
            } else {
                RTT_Print(0, "IMU calibration failed \r\n");
                return IMU_CALIBRATION_Failed;
            }
        }
    }
    return IMU_CALIBRATION_NEXT;
}
