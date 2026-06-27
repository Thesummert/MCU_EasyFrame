#include "bsp_mspm0g_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include "mcu_config.h"
#include "string.h"
#include "bsp_mspm0g_tim_base.h"

#define Delay_us(X) EasyFrameSysTime_Delay_us(X)
#define GetTime()   EasyFrameSysTime_GetTimeline_us()

_Bool EasyFrame_I2C_Init(EasyFrame_I2C_Typedef_t *self, I2C_Regs *i2c, uint32_t bus_clk) {
    if (self == NULL || i2c == NULL) {
        RTT_Print(0, "Null pointer error happened in i2c init \r\n");
        return false;
    }
    memset(self, 0, sizeof(EasyFrame_I2C_Typedef_t));
    self->mspm0g.i2c = i2c;

    DL_I2C_ClockConfig i2cClockConfig;
    DL_I2C_getClockConfig(i2c, &i2cClockConfig);
    /*
     * Calculate number of clock cycles to delay after controller transfer initiated
     * gDelayCycles = 3 I2C functional clock cycles
     * gDelayCycles = 3 * I2C clock divider * (CPU clock freq / I2C clock freq)
     */
    self->mspm0g.i2c_delay = (3 * (i2cClockConfig.divideRatio + 1)) * (CPUCLK_FREQ / bus_clk);

    self->transmit = EasyFrame_I2C_Transmit;
    self->receive = EasyFrame_I2C_Receive;

    self->is_inited = true;
    return true;
}

_Bool EasyFrame_I2C_Transmit(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                             uint16_t size, uint32_t timeout) {
    if (self->is_inited == false) {
        RTT_Print(0, "I2C not inited \r\n");
        return false;
    }
    uint64_t start = GetTime();
    uint8_t sendPackage = (size + I2C_FIFO_MAX_SIZE - 1) / I2C_FIFO_MAX_SIZE;  // 计算发包数
    for (uint8_t i = 0; i < sendPackage; ++i) {
        uint8_t currentSize =
            (i == sendPackage - 1) ? (size - i * I2C_FIFO_MAX_SIZE) : I2C_FIFO_MAX_SIZE;  // 当前包大小
        DL_I2C_fillControllerTXFIFO(self->mspm0g.i2c, data + i * I2C_FIFO_MAX_SIZE,
                                    currentSize);  // 填充FIFO
        /* Wait for I2C to be Idle */
        while (!(DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(5000);
        }
        DL_I2C_startControllerTransfer(self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX,
                                       currentSize);
        delay_cycles(self->mspm0g.i2c_delay);  // FIFO启动延时 防止TI的硬件BUG
        while (DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_BUSY) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(5000);
        }
        // 追踪问题
        if (DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        Delay_us(5000);  // 等待5ms 进入下一轮发送
    }
    return true;
}

_Bool EasyFrame_I2C_Receive(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                            uint16_t size, uint32_t timeout) {
    if (self->is_inited == false) {
        RTT_Print(0, "I2C not inited \r\n");
        return false;
    }
    uint64_t start = GetTime();

    DL_I2C_startControllerTransfer(self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, size);
    for (uint8_t i = 0; i < size; ++i) {
        while (DL_I2C_isControllerRXFIFOEmpty(self->mspm0g.i2c)) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(5000);
        }
        data[i] = DL_I2C_receiveControllerData(self->mspm0g.i2c);
    }
    return true;
}
