#ifndef __BSP_MSPM0G_I2C_H__
#define __BSP_MSPM0G_I2C_H__

#include <stdbool.h>
#include <stdint.h>
#include "bsp_mspm0g_gpio.h"
#include "ti/devices/msp/peripherals/hw_i2c.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "ti_msp_dl_config.h"

typedef struct EasyFrame_I2C_Typedef_t {
    _Bool is_inited;
    _Bool (*transmit)(struct EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                      uint16_t size, uint32_t timeout);
    _Bool (*receive)(struct EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                     uint16_t size, uint32_t timeout);
    struct {
        I2C_Regs *i2c;
        uint32_t i2c_delay;  // 等待周期 防止MSPM硬件BUG
    } mspm0g;
} EasyFrame_I2C_Typedef_t;

_Bool EasyFrame_I2C_Init(EasyFrame_I2C_Typedef_t *self, I2C_Regs *i2c, uint32_t bus_clk);
_Bool EasyFrame_I2C_Transmit(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                             uint16_t size, uint32_t timeout);
_Bool EasyFrame_I2C_Receive(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                            uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MSPM0G_I2C_H__ */
