/**
 * bsp_mspm0g_tim_base.c
 *
 * 这是系统定时器 可以在初始化中设定所使用的定时器
 *
 *
 */
#include "bsp_mspm0g_tim_base.h"
#include "string.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_timer.h"

static Time_t EasyFrameSysTime;
static uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
static uint32_t CYCCNT_RountCount;
static uint32_t CYCCNT_LAST;
static uint64_t CYCCNT64;
static void EasyFrameSysTime_CNT_Update(void);
static volatile uint32_t *cnt_now_ptr; // 定时器寄存器指针 方便修改代码

/**
 * @brief		EasyFrameSysTime定时器初始化
 * @param[in]	systemFreqMHz: 定时器频率 单位MHz
 * @retval		none
 */
void EasyFrameSysTime_Init(uint32_t systemFreqMHz)
{
    cnt_now_ptr = &(TIMG12->COUNTERREGS.CTR);

    memset(&EasyFrameSysTime, 0, sizeof(Time_t));

    CPU_FREQ_Hz = systemFreqMHz * 1000000;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;
    CYCCNT_RountCount = 0;
    DL_Timer_setLoadValue(TIMG12, 0xFFFFFFFF);
    DL_Timer_startCounter(TIMG12);
}

float EasyFrameSysTime_GetDeltaT(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = *cnt_now_ptr;
    float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    EasyFrameSysTime_CNT_Update();

    return dt;
}

double EasyFrameSysTime_GetDeltaT64(uint32_t *cnt_last)
{
    volatile uint32_t cnt_now = *cnt_now_ptr;
    double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_FREQ_Hz));
    *cnt_last = cnt_now;

    EasyFrameSysTime_CNT_Update();

    return dt;
}

void EasyFrameSysTime_EasyFrameSysTimeUpdate(void)
{
    volatile uint32_t cnt_now = *cnt_now_ptr;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    EasyFrameSysTime_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
    EasyFrameSysTime.s = CNT_TEMP1;
    EasyFrameSysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 - EasyFrameSysTime.ms * CPU_FREQ_Hz_ms;
    EasyFrameSysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float EasyFrameSysTime_GetTimeline_s(void)
{
    EasyFrameSysTime_EasyFrameSysTimeUpdate();

    float EasyFrameSysTime_Timelinef32 = EasyFrameSysTime.s + EasyFrameSysTime.ms * 0.001f + EasyFrameSysTime.us * 0.000001f;

    return EasyFrameSysTime_Timelinef32;
}

float EasyFrameSysTime_GetTimeline_ms(void)
{
    EasyFrameSysTime_EasyFrameSysTimeUpdate();

    float EasyFrameSysTime_Timelinef32 = EasyFrameSysTime.s * 1000 + EasyFrameSysTime.ms + EasyFrameSysTime.us * 0.001f;

    return EasyFrameSysTime_Timelinef32;
}

uint64_t EasyFrameSysTime_GetTimeline_us(void)
{
    EasyFrameSysTime_EasyFrameSysTimeUpdate();

    uint64_t EasyFrameSysTime_Timelinef32 = EasyFrameSysTime.s * 1000000 + EasyFrameSysTime.ms * 1000 + EasyFrameSysTime.us;

    return EasyFrameSysTime_Timelinef32;
}

/**
 * @brief	  私有函数,用于检查EasyFrameSysTime 寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
 */
static void EasyFrameSysTime_CNT_Update(void)
{
    volatile uint32_t cnt_now = *cnt_now_ptr;

    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++; // 溢出

    CYCCNT_LAST = cnt_now;
}

void EasyFrameSysTime_Delay(float Delay)
{
    uint32_t tickstart = *cnt_now_ptr;
    float wait = Delay;

    while ((*cnt_now_ptr - tickstart) < wait * (float)CPU_FREQ_Hz)
    {
    }
}

void EasyFrameSysTime_Delay_us(uint64_t delay)
{
    uint32_t tickstart = *cnt_now_ptr;
    while ((*cnt_now_ptr - tickstart) < delay * CPU_FREQ_Hz_us)
    {

    }
}
