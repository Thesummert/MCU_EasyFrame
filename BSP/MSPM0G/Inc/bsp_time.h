#ifndef __BSP_TIME_H__
#define __BSP_TIME_H__

#include <math.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*计数器计数模式*/
typedef enum {
  TIMER_COUNT_MODE_ONCE_UP,
  TIMER_COUNT_MODE_ONCE_DOWN,
  TIMER_COUNT_MODE_ONCE_UP_DOWN,
  TIMER_COUNT_MODE_PERIODIC_UP,
  TIMER_COUNT_MODE_PERIODIC_DOWN,
  TIMER_COUNT_MODE_PERIODIC_UP_DOWN,
} EF_TimerCounterMode_e;

typedef enum {
  TIMER_PWM_MODE_UP,
  TIMER_PWM_MODE_DOWN,
  TIMER_PWM_MODE_CENTER,
} EF_TimerPWMMode_e;

/*Easyframe 定时器*/
typedef struct EF_BSP_TimerBase_t {
  _Bool is_inited;                    // 是否完成初始化
  uint32_t prescaler;                 // 预分频系数
  uint32_t auto_reload_num;           // 自动重装载值
  uint32_t max_prescaler;             // 最大分频系数
  uint32_t max_auto_reload_num;       // 最大重装载值
  uint32_t clock_source_freq;         // 时钟源频率
  EF_TimerCounterMode_e counter_mode; // 计时模式
  float freq;                         // 当前频率
  float max_freq;                     // 当前设定下的最高频率

  // 函数指针
  _Bool (*StartCounter)(struct EF_BSP_TimerBase_t *self);
  _Bool (*StopCounter)(struct EF_BSP_TimerBase_t *self);
  _Bool (*setCounterMode)(struct EF_BSP_TimerBase_t *self,
                          EF_TimerCounterMode_e mode);

  _Bool (*setCounterPrescaler)(struct EF_BSP_TimerBase_t *self,
                               uint32_t prescaler);
  _Bool (*getFreq)(struct EF_BSP_TimerBase_t *self, float *freq);

      struct {
    GPTIMER_Regs *tim;
    DL_TIMER_TIMER_MODE mode;

  } mspm0g;

} EF_BSP_TimerBase_t;

/*Easyframe PWM控制结构体*/
typedef struct EF_BSP_TimerPWM_t {
  _Bool is_inited;
  EF_BSP_TimerBase_t *etim; // EF定时器句柄
  uint8_t max_channel;      // 最大通道数
  float radio[6];           // 占空比
  uint32_t channels[6];     // CCP装载值
  EF_TimerPWMMode_e mode;   // PWM模式

  // 函数指针
  _Bool (*SetPWM_Float)(struct EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                        float radio);
  _Bool (*SetPWM)(struct EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                  uint32_t compare);
  _Bool (*StartPWM)(struct EF_BSP_TimerPWM_t *self);
  _Bool (*StopPWM)(struct EF_BSP_TimerPWM_t *self);
  _Bool (*SetPWM_Mode)(struct EF_BSP_TimerPWM_t *self, EF_TimerPWMMode_e mode);

  struct {
    DL_TIMER_PWM_MODE pwm_mode; // PWM模式
    _Bool is_support4channel;   // 是否支持4通道
  } mspm0g;
} EF_BSP_TimerPWM_t;

typedef struct EF_BSP_TimerQEI_t {
  _Bool is_inited;
  EF_BSP_TimerBase_t *etim; // EF定时器句柄
  uint32_t base_load;       // 编码器起始值
  int32_t delta;            // 与编码器起始值的差

  // 函数指针
  _Bool (*setBaseLoad)(struct EF_BSP_TimerQEI_t *self, uint32_t load);
  _Bool (*getDelta)(struct EF_BSP_TimerQEI_t *self, int32_t *delta);

} EF_BSP_TimerQEI_t;

_Bool EF_BSP_TimerBase_Init(EF_BSP_TimerBase_t *self, GPTIMER_Regs *tim,
                            uint32_t clock_source_freq, uint32_t max_prescaler,
                            uint32_t max_auto_reload_num);

_Bool EF_BSP_TimerPWM_Init(EF_BSP_TimerPWM_t *self, EF_BSP_TimerBase_t *etim,
                           uint8_t max_channel);
_Bool EF_BSP_TimerQEI_Init(EF_BSP_TimerQEI_t *self, EF_BSP_TimerBase_t *etim);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TIME_H__ */
