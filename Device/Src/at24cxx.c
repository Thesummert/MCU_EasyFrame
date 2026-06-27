#include "at24cxx.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "bsp_mspm0g_i2c.h"
#include "mcu_config.h"

#define AT24_TYPE_NUM   10     // at24种类数
#define AT24_DELAY_TIME 20000  // 20ms延时

const uint32_t AT_MAX_SIZE[AT24_TYPE_NUM] = {128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};

static _Bool ReadByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t *data);
static _Bool WriteByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t data);

_Bool EF_Device_AT24CXX_Init(EF_Device_AT24CXX_t *self, EF_Device_AT24Type_e type, uint8_t addr,
                             I2C_Regs *i2c, uint32_t bus_clk) {
    if (self == NULL || i2c == NULL) {
        RTT_Print(0, "Null pointer error happen in at24 init \r\n");
        return false;
    }
    if (!EasyFrame_I2C_Init(&self->i2c, i2c, bus_clk)) {
        return false;
    }
    self->device_type = type;
    self->device_addr = addr;
    self->max_size = AT_MAX_SIZE[type];

    // 函数指针设置
    self->WriteByte = WriteByte;
    self->ReadByte = ReadByte;

    self->is_inited = true;

    return true;
}

static _Bool ReadByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t *data) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in at24 read \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "AT24 not inited /r/n");
        return false;
    }
    if (self->max_size < addr ) {
        RTT_Print(0, "AT24 overflow \r\n");
        return false;
    }
    EasyFrame_I2C_Typedef_t *i2c = &self->i2c;
    uint8_t cmd[2];
    /*不同型号有不同的地址设置*/
    if (self->device_type > EF_AT24_TYPE_C16) {
        cmd[0] = (addr >> 8) & 0xFF;
        cmd[1] = addr & 0xFF;
        if (!i2c->transmit(i2c, self->device_addr, cmd, 2, AT24_DELAY_TIME)) {
            return false;
        }
    } else {
        uint8_t device_addr = (self->device_addr + ((addr / 256) << 1));  // 设定接收地址
        cmd[0] = addr & 0xFF;
        if (!i2c->transmit(i2c, device_addr, cmd, 1, AT24_DELAY_TIME)) {
            return false;
        }
    }
    if (!i2c->receive(i2c, self->device_addr , data, 1, AT24_DELAY_TIME)) {
        return false;
    }
    return true;
}

static _Bool WriteByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t data)
{
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in at24 write \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "AT24 not inited /r/n");
        return false;
    }
    if (self->max_size < addr ) {
        RTT_Print(0, "AT24 overflow \r\n");
        return false;
    }

    EasyFrame_I2C_Typedef_t *i2c = &self->i2c;
    uint8_t cmd[4];
    /*不同型号有不同的地址设置*/
    if (self->device_type > EF_AT24_TYPE_C16) {
        cmd[0] = (addr >> 8) & 0xFF;
        cmd[1] = addr & 0xFF;
        cmd[2] = data;
        if (!i2c->transmit(i2c, self->device_addr, cmd, 3, AT24_DELAY_TIME)) {
            return false;
        }
    } else {
        uint8_t device_addr = (self->device_addr + ((addr / 256) << 1));  // 设定接收地址
        cmd[0] = addr & 0xFF;
        cmd[1] = data;
        if (!i2c->transmit(i2c, device_addr, cmd, 2, AT24_DELAY_TIME)) {
            return false;
        }
    }

    return true;
}
