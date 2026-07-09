#ifndef __QEI_ENCODER_H__
#define __QEI_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_time.h"
#include <stdint.h>

typedef struct EF_QEI_Encoder_t {
  EF_BSP_TimerQEI_t *qei;
  int32_t delta;          // 两次差值
  uint32_t pulse_per_rev; // 每转脉冲数(PPR)
  uint8_t multiplier;     // 倍频数(通常为4)
  float dt;               // 间隔时间
  float omega_rotor;      // 转子角速度
  float omega_output;     // 输出轴角速度
  float radio;            // 减速比
                          //
  // 倒数 部分MCU不支持硬件除法
  float inv_pulse_per_rev; // 每转脉冲数(PPR)
  float inv_multiplier;    // 倍频数(通常为4)
  float inv_radio;         // 减速比

  _Bool is_inited; // 是否初始化

  _Bool (*getSpeed)(struct EF_QEI_Encoder_t *self, float dt, float *omega_rotor,
                    float *omega_output);
} EF_QEI_Encoder_t;

_Bool EF_Device_QEI_Encoder_Init(EF_QEI_Encoder_t *self, EF_BSP_TimerQEI_t *qei,
                                 uint32_t ppr, uint8_t multiplier, float radio);

#ifdef __cplusplus
}
#endif

#endif /* __QEI_ENCODER_H__ */
