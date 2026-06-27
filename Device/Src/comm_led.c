/**
 *
 * @brief 这是通用普通LED驱动
 *
 *
 */
#include "comm_led.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_mspm0g_gpio.h"
#include "mcu_config.h"

static _Bool Set(EF_Device_Comm_LED_t *self, EF_COMM_LED_State_e state, float freq, float time_line);
static _Bool Show(EF_Device_Comm_LED_t *self, float time_line);

_Bool EF_Device_Comm_LED_Init(EF_Device_Comm_LED_t *self, GPIO_Regs *gpio, uint32_t pin,
                              _Bool off_voltage) {
    if (self == NULL || gpio == NULL) {
        RTT_Print(0, "Null pointer error happened in comm_led init \r\n");
        return false;
    }
    memset(self, 0, sizeof(EF_Device_Comm_LED_t));
    EasyFrame_GPIO_Init(&self->gpio, gpio, pin);  // 初始化对应的GPIO
    // 设定熄灭电平
    self->off_voltage = off_voltage;
    if (off_voltage == 0) {
        self->gpio.SetLow(&self->gpio);
    } else {
        self->gpio.SetHigh(&self->gpio);
    }
    // 设定函数指针
    self->Set = Set;
    self->Show = Show;
    self->is_inited = true;
    return true;
}

static _Bool Set(EF_Device_Comm_LED_t *self, EF_COMM_LED_State_e state, float freq, float time_line) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happened in comm_led set\r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "Comm led not inited \r\n");
        return false;
    }
    // 如果设定值相同 则不重置
    if (self->state == state && self->twinkle_data.freq == freq) {
        self->is_set = false;
        return false;
    }
    self->state = EF_COMM_LED_TWINKLE;
    if (state == EF_COMM_LED_TWINKLE) {
        self->twinkle_data.freq = freq;
        self->twinkle_data.interval_time = 1.0 / freq;  // 计算间隔时间
        self->twinkle_data.time_line_start = time_line;
        self->twinkle_data.time_line_end = time_line + self->twinkle_data.interval_time;
    }
    self->is_set = true;
    return true;
}

static _Bool Show(EF_Device_Comm_LED_t *self, float time_line) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happened in comm_led show\r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "Comm led not inited \r\n");
        return false;
    }
    // 只有在要求更新的状态下才更新
    if (self->is_set == true && self->state != EF_COMM_LED_TWINKLE) {
        if (self->off_voltage == 0) {
            self->gpio.SetLow(&self->gpio);
        } else {
            self->gpio.SetHigh(&self->gpio);
        }
        self->is_set = false;
        return true;
    }
    // 闪烁部分单独处理
    // 进入第一次闪烁先亮灯
    if (self->is_set == true) {
        if (self->off_voltage == 0) {
            self->gpio.SetLow(&self->gpio);
        } else {
            self->gpio.SetHigh(&self->gpio);
        }
        self->is_set = false;
    }
    if (time_line > self->twinkle_data.time_line_end) {
        // 到时间翻转led
        self->gpio.Toggle(&self->gpio);
        self->twinkle_data.time_line_start = time_line;
        self->twinkle_data.time_line_end = time_line + self->twinkle_data.interval_time;
    }
    return true;
}
