#ifndef __COMM_LED_H__
#define __COMM_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_mspm0g_gpio.h"  // 选择所需的gpio头文件 今后会将所有BSP命名相同

// LED状态开关
typedef enum {
    EF_COMM_LED_OFF,        // 关闭
    EF_COMM_LED_ALWAYS_ON,  // 常亮
    EF_COMM_LED_TWINKLE,    // 闪烁
} EF_COMM_LED_State_e;

typedef struct EF_Device_Comm_LED_t {
    EasyFrame_GPIO_Typedef_t gpio;  // LED对应GPIO
    EF_COMM_LED_State_e state;      // 状态
    _Bool off_voltage;              // 关灯时后的电平 0代表低电平 1代表高电平
    struct {
        float freq;             // 闪烁状态频率 无法大于系统轮询任务最大频率
        float interval_time;    // 闪烁间隔时间
        float time_line_start;  // 状态1开始时间
        float time_line_end;    // 状态1结束时间
    } twinkle_data;             // 闪烁所需数据
    _Bool is_inited;            // 初始化标志位
    _Bool is_set;               // 是否设置标志位
                                //
    // 函数指针

    _Bool (*Set)(struct EF_Device_Comm_LED_t *self, EF_COMM_LED_State_e state, float freq,
                 float time_line);
    _Bool (*Show)(struct EF_Device_Comm_LED_t *self, float time_line);
} EF_Device_Comm_LED_t;

_Bool EF_Device_Comm_LED_Init(EF_Device_Comm_LED_t *self, GPIO_Regs *gpio, uint32_t pin,
                              _Bool off_voltage);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_LED_H__ */
