#include "bsp_mspm0g_gpio.h"
#include <stdbool.h>
#include "mcu_config.h"
#include "ti/driverlib/dl_gpio.h"

/**
 * @brief 初始化GPIO引脚
 * @param self EasyFrame_GPIO_Typedef_t结构体指针
 * @param gpio GPIO寄存器结构体指针
 * @param pin  引脚号
 * @retval true 初始化成功
 * @retval false 初始化失败（如指针为NULL）
 */
_Bool EasyFrame_GPIO_Init(EasyFrame_GPIO_Typedef_t *self, GPIO_Regs *gpio, uint32_t pin)
{
    if (self == NULL || gpio == NULL) {
        RTT_Print(0, "Null pointer error happened in GPIO INIT \r\n");
        return false;
    }
    self->mspm0g.gpio = gpio;
    self->mspm0g.pin = pin;
    self->isInited = true;

    self->Toggle = EasyFrame_GPIO_Toggle;
    self->SetHigh = EasyFrame_GPIO_SetHigh;
    self->SetLow = EasyFrame_GPIO_SetLow;

    return true;
}

_Bool EasyFrame_GPIO_InitIOMux(EasyFrame_GPIO_Typedef_t *self, IOMUX_PINCM iomux)
{

    if (self == NULL ) {
        RTT_Print(0, "Null pointer error happened in GPIO INIT \r\n");
        return false;
    }
    self->mspm0g.iomux = iomux;
    return true;
}
/**
 * @brief 翻转GPIO引脚电平
 * @param self EasyFrame_GPIO_Typedef_t结构体指针
 * @retval true 操作成功
 * @retval false GPIO未初始化
 */
_Bool EasyFrame_GPIO_Toggle(EasyFrame_GPIO_Typedef_t *self)
{
    if (self->isInited == false) {
        RTT_Print(0, "GPIO Not Inited! /r/n");
        return false;
    }
    DL_GPIO_togglePins(self->mspm0g.gpio, self->mspm0g.pin);
    
    return true;
}

/**
 * @brief 设置GPIO引脚为高电平
 * @param self EasyFrame_GPIO_Typedef_t结构体指针
 * @retval true 操作成功
 * @retval false GPIO未初始化
 */
_Bool EasyFrame_GPIO_SetHigh(EasyFrame_GPIO_Typedef_t *self)
{
    if (self->isInited == false) {
        RTT_Print(0, "GPIO Not Inited! /r/n");
        return false;
    }
    DL_GPIO_setPins(self->mspm0g.gpio, self->mspm0g.pin);
    
    return true;
}

/**
 * @brief 设置GPIO引脚为低电平
 * @param self EasyFrame_GPIO_Typedef_t结构体指针
 * @retval true 操作成功
 * @retval false GPIO未初始化
 */
_Bool EasyFrame_GPIO_SetLow(EasyFrame_GPIO_Typedef_t *self)
{
    if (self->isInited == false) {
        RTT_Print(0, "GPIO Not Inited! /r/n");
        return false;
    }
    DL_GPIO_clearPins(self->mspm0g.gpio, self->mspm0g.pin);
    
    return true;
}
