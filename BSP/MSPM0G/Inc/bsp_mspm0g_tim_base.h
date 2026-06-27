#ifndef __BSP_MSPM0G_TIM_BASE_H__
#define __BSP_MSPM0G_TIM_BASE_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "ti_msp_dl_config.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} Time_t;

void EasyFrameSysTime_Init(uint32_t systemFreqMHz);
float EasyFrameSysTime_GetDeltaT(uint32_t *cnt_last);
double EasyFrameSysTime_GetDeltaT64(uint32_t *cnt_last);
float EasyFrameSysTime_GetTimeline_s(void);
float EasyFrameSysTime_GetTimeline_ms(void);
uint64_t EasyFrameSysTime_GetTimeline_us(void);
void EasyFrameSysTime_Delay(float Delay);
void EasyFrameSysTime_EasyFrameSysTimeUpdate(void);
void EasyFrameSysTime_Delay_us(uint64_t delay);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MSPM0G_TIM_BASE_H__ */
