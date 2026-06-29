/**
 * @file at24cxx.c
 * @brief AT24Cxx 系列 EEPROM 驱动实现
 *
 * 支持 AT24C01/02/04/08/16/32/64/128/256/512 系列 EEPROM
 * 通过 I2C 接口实现字节读写和连续读写操作。
 */

#include "at24cxx.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "bsp_mspm0g_i2c.h"
#include "mcu_config.h"
#include "bsp_mspm0g_tim_base.h"

/** 毫秒级延时宏 */
#define Delay(X) EasyFrameSysTime_Delay(X)

/** AT24 支持的种类数量 */
#define AT24_TYPE_NUM 10  // at24种类数
/** 写操作后的内部写入延时（微秒） */
#define AT24_DELAY_TIME 200000  // 200ms延时

/** 各型号 EEPROM 的最大容量（字节），索引与 EF_Device_AT24Type_e 对应 */
const uint32_t AT_MAX_SIZE[AT24_TYPE_NUM] = {128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536};

static _Bool ReadByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t *data);
static _Bool WriteByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t data);
static _Bool WriteData(EF_Device_AT24CXX_t *self, uint32_t start_addr, uint8_t *data, uint32_t len);
static _Bool ReadData(EF_Device_AT24CXX_t *self, uint32_t start_addr, uint8_t *data, uint32_t len);

/**
 * @brief 初始化 AT24Cxx 设备
 * @param self    AT24Cxx 对象指针
 * @param type    器件型号（EF_Device_AT24Type_e 枚举）
 * @param addr    I2C 设备地址
 * @param i2c     I2C 外设寄存器基址
 * @param bus_clk I2C 总线时钟频率（Hz）
 * @return 成功返回 true，失败返回 false
 *
 * 初始化底层 I2C 控制器，设置设备类型、地址和容量，
 * 并绑定内部读写函数指针。
 */
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
    self->WriteData = WriteData;
    self->ReadData = ReadData;

    self->is_inited = true;

    return true;
}

/**
 * @brief 从指定地址读取一个字节
 * @param self AT24Cxx 对象指针
 * @param addr 读取地址
 * @param data 输出读取到的数据
 * @return 成功返回 true，失败返回 false
 *
 * 对于 C16 以上型号（AT24C32/64 等），地址为 16 位（双字节）；
 * 对于 C16 及以下型号（AT24C01/02/04/08/16），地址为 8 位（单字节），
 * 且高位地址通过修改 I2C 从机地址实现。
 */
static _Bool ReadByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t *data) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in at24 read \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "AT24 not inited /r/n");
        return false;
    }
    if (self->max_size < addr) {
        RTT_Print(0, "AT24 overflow \r\n");
        return false;
    }
    EasyFrame_I2C_Typedef_t *i2c = &self->i2c;
    uint8_t cmd[2];
    uint8_t cmd_len = 0;
    uint8_t device_addr = self->device_addr;
    /*不同型号有不同的地址设置*/
    if (self->device_type > EF_AT24_TYPE_C16) {
        cmd[0] = (addr >> 8) & 0xFF;
        cmd[1] = addr & 0xFF;
        cmd_len = 2;
    } else {
        device_addr = (self->device_addr + ((addr / 256) << 1));  // 设定接收地址
        cmd[0] = addr & 0xFF;
        cmd_len = 1;
    }
    if (!i2c->ReadAfterTransmit(i2c, device_addr, cmd, cmd_len, data, 1, AT24_DELAY_TIME * 10)) {
        return false;
    }
    return true;
}

/**
 * @brief 向指定地址写入一个字节
 * @param self AT24Cxx 对象指针
 * @param addr 写入地址
 * @param data 待写入的字节数据
 * @return 成功返回 true，失败返回 false
 *
 * 不同型号的地址编码方式与 ReadByte 相同。
 * 写操作完成后需要等待器件内部写周期（AT24_DELAY_TIME）。
 */
static _Bool WriteByte(EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t data) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in at24 write \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "AT24 not inited /r/n");
        return false;
    }
    if (self->max_size < addr) {
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

/**
 * @brief 从起始地址开始连续写入多个字节
 * @param self       AT24Cxx 对象指针
 * @param start_addr 起始地址
 * @param data       待写入数据缓冲区
 * @param len        写入字节数
 * @return 成功返回 true，失败返回 false
 *
 * 逐个字节调用 WriteByte 写入，不启用 EEPROM 页写入模式。
 * 每字节写入后插入 100ms 延时等待写周期完成。
 */
static _Bool WriteData(EF_Device_AT24CXX_t *self, uint32_t start_addr, uint8_t *data, uint32_t len) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in at24 write \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "AT24 not inited /r/n");
        return false;
    }
    if (self->max_size < start_addr || self->max_size < start_addr + len) {
        RTT_Print(0, "AT24 overflow \r\n");
        return false;
    }
    for (uint32_t i = 0; i < len; i++) {
        if (!self->WriteByte(self, start_addr + i, data[i])) {
            return false;
        }
        Delay(0.10f);
    }
    return true;
}

/**
 * @brief 从起始地址开始连续读取多个字节
 * @param self       AT24Cxx 对象指针
 * @param start_addr 起始地址
 * @param data       接收数据缓冲区
 * @param len        读取字节数
 * @return 成功返回 true，失败返回 false
 *
 * 逐个字节调用 ReadByte 读取，每字节后插入 50ms 延时。
 */
static _Bool ReadData(EF_Device_AT24CXX_t *self, uint32_t start_addr, uint8_t *data, uint32_t len) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happen in at24 write \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "AT24 not inited /r/n");
        return false;
    }
    if (self->max_size < start_addr || self->max_size < start_addr + len) {
        RTT_Print(0, "AT24 overflow \r\n");
        return false;
    }
    for (uint32_t i = 0; i < len; i++) {
        if (!self->ReadByte(self, start_addr + i, &data[i])) {
            return false;
        }
        Delay(0.05f);
    }
    return true;
}
