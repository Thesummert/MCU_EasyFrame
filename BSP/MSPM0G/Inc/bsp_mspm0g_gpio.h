#ifndef __BSP_MSPM0G_GPIO_H__
#define __BSP_MSPM0G_GPIO_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "ti_msp_dl_config.h"

    // GPIO抽象封装
typedef struct EasyFrame_GPIO_Typedef_t {

    _Bool (*Toggle)(struct EasyFrame_GPIO_Typedef_t *self);
    _Bool (*SetHigh)(struct EasyFrame_GPIO_Typedef_t *self);
    _Bool (*SetLow)(struct EasyFrame_GPIO_Typedef_t *self);
    _Bool isInited;
    // MSPM0G GPIO结构体
    struct {
        GPIO_Regs *gpio;
        uint32_t pin;
    }mspm0g ;
} EasyFrame_GPIO_Typedef_t;

_Bool EasyFrame_GPIO_Init(EasyFrame_GPIO_Typedef_t *self, GPIO_Regs *gpio, uint32_t pin);
_Bool EasyFrame_GPIO_Toggle(EasyFrame_GPIO_Typedef_t *self);
_Bool EasyFrame_GPIO_SetHigh(EasyFrame_GPIO_Typedef_t *self);
_Bool EasyFrame_GPIO_SetLow(EasyFrame_GPIO_Typedef_t *self);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MSPM0G_GPIO_H__ */
