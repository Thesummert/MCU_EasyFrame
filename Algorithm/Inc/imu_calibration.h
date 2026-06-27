#ifndef __IMU_CALIBRATION_H__
#define __IMU_CALIBRATION_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "mcu_config.h"
#include "stdbool.h"

#if USING_IQMATH_IMU == 1
#include "ti/iqmath/include/IQmathLib.h"
#endif

typedef enum {
    IMU_CALIBRATION_NULLPOINTER_ERROR,  // 空指针错误
    IMU_CALIBRATION_SUCCESS,            // 成功校准
    IMU_CALIBRATION_Failed,             // 校准失败
    IMU_CALIBRATION_NEXT,               // 可以进入下一轮校准
} IMU_Cali_Result_e;

typedef struct IMU_Calibration_t {
    struct {
        // 存储加速度计和角速度计漂移值
        float accel[3];
        float gyro[3];

        // 存储加速度计和角速度计漂移计算中间值
        float accel_sum[3];
        float gyro_sum[3];

        // 存储最大值和最小值
        float accel_min[3];
        float gyro_min[3];
        float accel_max[3];
        float gyro_max[3];

        uint32_t sample_time;     // 采样次数
        float sample_start;       // 采样开始时间
        float sample_except_end;  // 采样期望结束时间
        _Bool cali_flag;          // 校准标志位
        _Bool first_run; // 首次运行标志位
    } static_cali;                // 用于手动触发的校准
    struct {
    } dynamic_cali;  // 用于上电后自动校准

    void (*StartStaticCali)(struct IMU_Calibration_t *self, float start_time);
    IMU_Cali_Result_e (*AddSamples)(struct IMU_Calibration_t *self, uint8_t stat_or_dyna, float gyro[3],
                                    float accel[3], float run_time);
} IMU_Calibration_t;

_Bool EF_Algorithm_IMU_Calibration_Init(IMU_Calibration_t *self);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_CALIBRATION_H__ */
