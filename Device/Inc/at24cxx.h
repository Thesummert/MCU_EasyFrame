#ifndef __AT24CXX_H__
#define __AT24CXX_H__

#include <stdint.h>
#include "bsp_mspm0g_i2c.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 设备型号枚举 */
typedef enum {
    EF_AT24_TYPE_C1 = 0,
    EF_AT24_TYPE_C2,
    EF_AT24_TYPE_C4,
    EF_AT24_TYPE_C8,
    EF_AT24_TYPE_C16,
    EF_AT24_TYPE_C32,
    EF_AT24_TYPE_C64,
    EF_AT24_TYPE_C128,
    EF_AT24_TYPE_C256,
    EF_AT24_TYPE_C512,
} EF_Device_AT24Type_e;

/* 24cxx驱动 */
typedef struct EF_Device_AT24CXX_t {
    EF_Device_AT24Type_e device_type;  // 设备种类
    uint8_t device_addr;               // 设备地址
    uint32_t max_size;                 // 最大存储大小
    EasyFrame_I2C_Typedef_t i2c;       // I2C通信结构体
    _Bool is_inited;                   // 初始化标志位

    _Bool (*ReadByte)(struct EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t *data);
    _Bool (*WriteByte)(struct EF_Device_AT24CXX_t *self, uint32_t addr, uint8_t data);
} EF_Device_AT24CXX_t;

_Bool EF_Device_AT24CXX_Init(EF_Device_AT24CXX_t *self, EF_Device_AT24Type_e type, uint8_t addr,
                             I2C_Regs *i2c, uint32_t bus_clk);

#ifdef __cplusplus
}
#endif

#endif /* __AT24CXX_H__ */
