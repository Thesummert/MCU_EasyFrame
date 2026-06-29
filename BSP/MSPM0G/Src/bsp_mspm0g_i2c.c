/**
 * @file bsp_mspm0g_i2c.c
 * @brief MSPM0G I2C 总线驱动实现
 *
 * 基于 TI MSPM0G 系列 MCU 的 I2C 控制器驱动，提供 I2C 主模式下的
 * 收发、高级传输、先写后读、总线恢复等功能。
 * 内部使用 FIFO 分包传输机制，并包含 TI 硬件 BUG 的延时补偿处理。
 */

#include "bsp_mspm0g_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/reent.h>
#include "mcu_config.h"
#include "string.h"
#include "bsp_mspm0g_tim_base.h"
#include "ti/devices/msp/peripherals/hw_i2c.h"
#include "ti/driverlib/dl_i2c.h"

/** 微秒级延时宏 */
#define Delay_us(X) EasyFrameSysTime_Delay_us(X)
/** 获取当前时间戳（微秒）宏 */
#define GetTime()   EasyFrameSysTime_GetTimeline_us()

static _Bool EasyFrame_I2C_Transmit(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                                    uint16_t size, uint32_t timeout);
static _Bool EasyFrame_I2C_Receive(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                                   uint16_t size, uint32_t timeout);
static _Bool TransmitAdvanced(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                              uint16_t size, uint32_t timeout, _Bool start, _Bool ack, _Bool stop);
static _Bool ReadAfterTransmit(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *cmd,
                               uint16_t cmd_len, uint8_t *data, uint16_t data_len, uint32_t timeout);
static _Bool Reset(EasyFrame_I2C_Typedef_t *self, _Bool need_change);

/**
 * @brief 初始化 I2C 控制器
 * @param self    I2C 对象指针
 * @param i2c     I2C 外设寄存器基址
 * @param bus_clk I2C 总线时钟频率（Hz）
 * @return 成功返回 true，失败返回 false
 *
 * 初始化 I2C 对象，计算 FIFO 启动延时所需的时钟周期数，
 * 绑定内部函数指针。
 */
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
    self->TransmitAdvanced = TransmitAdvanced;
    self->ReadAfterTransmit = ReadAfterTransmit;
    self->Reset = Reset;

    self->is_inited = true;
    return true;
}

/**
 * @brief 初始化 I2C 引脚（SDA/SCL）
 * @param self I2C 对象指针
 * @param sda  SDA 引脚 GPIO 配置
 * @param scl  SCL 引脚 GPIO 配置
 * @return 成功返回 true，失败返回 false
 *
 * 保存 SDA 和 SCL 的 GPIO 信息，用于后续总线恢复时的 GPIO 位冲操作。
 */
_Bool EasyFrame_I2C_InitGPIO(EasyFrame_I2C_Typedef_t *self, EasyFrame_GPIO_Typedef_t sda,
                             EasyFrame_GPIO_Typedef_t scl) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error happened in i2c init \r\n");
        return false;
    }
    self->mspm0g.scl = scl;
    self->mspm0g.sda = sda;
    return true;
}

/**
 * @brief I2C 数据发送
 * @param self       I2C 对象指针
 * @param slave_addr 从机设备地址（7位）
 * @param data       待发送数据缓冲区
 * @param size       发送数据字节数
 * @param timeout    超时时间（微秒）
 * @return 成功返回 true，失败返回 false
 *
 * 将数据按 FIFO 最大长度分包发送。单包发送带起始/停止条件，
 * 多包时首包发起始、末包发停止、中间包不产生起停条件。
 */
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

        /* Wait for I2C to be Idle */
        while (!(DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(20);
        }
        DL_I2C_flushControllerTXFIFO(self->mspm0g.i2c);
        delay_cycles(self->mspm0g.i2c_delay);      // FIFO启动延时 防止TI的硬件BUG

        DL_I2C_fillControllerTXFIFO(self->mspm0g.i2c, data + i * I2C_FIFO_MAX_SIZE,
                                    currentSize);  // 填充FIFO
        delay_cycles(self->mspm0g.i2c_delay);      // FIFO启动延时 防止TI的硬件BUG
        /*根据发包数量选择发送模式*/
        if (sendPackage == 1) {
            // 如果只有一次发包 则发送启动 结束
            DL_I2C_startControllerTransferAdvanced(
                self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, currentSize,
                DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
                DL_I2C_CONTROLLER_ACK_ENABLE);
        }
        else if (i == 0) {
            // 如果有多次发包 则第一次发送启动
            DL_I2C_startControllerTransferAdvanced(
                self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, currentSize,
                DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
                DL_I2C_CONTROLLER_ACK_ENABLE);
        }
        else if (i == sendPackage - 1) {
            // 如果有多次发包 则最后一次发送结束
            DL_I2C_startControllerTransferAdvanced(
                self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, currentSize,
                DL_I2C_CONTROLLER_START_DISABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
                DL_I2C_CONTROLLER_ACK_ENABLE);
        }
        else {
            // 如果有多次发包 则其余包不发送启动与结束
            DL_I2C_startControllerTransferAdvanced(
                self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, currentSize,
                DL_I2C_CONTROLLER_START_DISABLE, DL_I2C_CONTROLLER_STOP_DISABLE,
                DL_I2C_CONTROLLER_ACK_ENABLE);
        }
        // DL_I2C_startControllerTransfer(self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX,
        //                                currentSize);
        delay_cycles(self->mspm0g.i2c_delay);  // FIFO启动延时 防止TI的硬件BUG
        while (DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_BUSY) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(20);
        }
        // 追踪问题
        if (DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        Delay_us(20);  // 等待5ms 进入下一轮发送
    }
    return true;
}

/**
 * @brief I2C 数据接收
 * @param self       I2C 对象指针
 * @param slave_addr 从机设备地址（7位）
 * @param data       接收数据缓冲区
 * @param size       预期接收的字节数
 * @param timeout    超时时间（微秒）
 * @return 成功返回 true，失败返回 false
 *
 * 启动 I2C 接收传输后，从 RX FIFO 逐字节读取数据。
 */
_Bool EasyFrame_I2C_Receive(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                            uint16_t size, uint32_t timeout) {
    if (self->is_inited == false) {
        RTT_Print(0, "I2C not inited \r\n");
        return false;
    }
    uint64_t start = GetTime();

    while (!(DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (GetTime() - start > timeout) {
            return false;
        }
        Delay_us(20);
    }
    delay_cycles(self->mspm0g.i2c_delay);  // FIFO启动延时 防止TI的硬件BUG
    DL_I2C_startControllerTransfer(self->mspm0g.i2c, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, size);
    delay_cycles(self->mspm0g.i2c_delay);  // FIFO启动延时 防止TI的硬件BUG
    for (uint8_t i = 0; i < size; ++i) {
        while (DL_I2C_isControllerRXFIFOEmpty(self->mspm0g.i2c)) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(20);
        }
        data[i] = DL_I2C_receiveControllerData(self->mspm0g.i2c);
    }
    return true;
}

/**
 * @brief I2C 高级发送（可自定义起始、应答、停止条件）
 * @param self       I2C 对象指针
 * @param slave_addr 从机设备地址
 * @param data       待发送数据缓冲区
 * @param size       数据长度
 * @param timeout    超时时间（微秒）
 * @param need_start 是否产生起始条件
 * @param ack        是否使能应答
 * @param stop       是否产生停止条件
 * @return 成功返回 true，失败返回 false
 *
 * 相比普通发送，允许调用方灵活控制起始/应答/停止信号的产生时机。
 */
static _Bool TransmitAdvanced(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *data,
                              uint16_t size, uint32_t timeout, _Bool need_start, _Bool ack, _Bool stop) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error in i2c \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "I2C not inited \r\n");
        return false;
    }
    DL_I2C_CONTROLLER_ACK ack_reg = DL_I2C_CONTROLLER_ACK_DISABLE;
    DL_I2C_CONTROLLER_STOP stop_reg = DL_I2C_CONTROLLER_STOP_DISABLE;
    DL_I2C_CONTROLLER_START start_reg = DL_I2C_CONTROLLER_START_DISABLE;
    if (ack) {
        ack_reg = DL_I2C_CONTROLLER_ACK_ENABLE;
    }
    if (stop) {
        stop_reg = DL_I2C_CONTROLLER_STOP_ENABLE;
    }
    if (need_start) {
        start_reg = DL_I2C_CONTROLLER_START_ENABLE;
    }

    uint64_t start = GetTime();
    uint8_t sendPackage = (size + I2C_FIFO_MAX_SIZE - 1) / I2C_FIFO_MAX_SIZE;  // 计算发包数
    for (uint8_t i = 0; i < sendPackage; ++i) {
        uint8_t currentSize =
            (i == sendPackage - 1) ? (size - i * I2C_FIFO_MAX_SIZE) : I2C_FIFO_MAX_SIZE;  // 当前包大小

        /* Wait for I2C to be Idle */
        while (!(DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(20);
        }

        DL_I2C_fillControllerTXFIFO(self->mspm0g.i2c, data + i * I2C_FIFO_MAX_SIZE,
                                    currentSize);  // 填充FIFO
        delay_cycles(self->mspm0g.i2c_delay);      // FIFO启动延时 防止TI的硬件BUG
        if (i == 0) {
            // 第一次发包必须发送start
            DL_I2C_startControllerTransferAdvanced(self->mspm0g.i2c, slave_addr,
                                                   DL_I2C_CONTROLLER_DIRECTION_TX, currentSize,
                                                   DL_I2C_CONTROLLER_START_ENABLE, stop_reg, ack_reg);
        } else {
            DL_I2C_startControllerTransferAdvanced(self->mspm0g.i2c, slave_addr,
                                                   DL_I2C_CONTROLLER_DIRECTION_TX, currentSize,
                                                   start_reg, stop_reg, ack_reg);
        }
        delay_cycles(self->mspm0g.i2c_delay);  // FIFO启动延时 防止TI的硬件BUG
        while (DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_BUSY) {
            if (GetTime() - start > timeout) {
                return false;
            }
            Delay_us(20);
        }
        // 追踪问题
        if (DL_I2C_getControllerStatus(self->mspm0g.i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) {
            return false;
        }
        Delay_us(20);  // 等待5ms 进入下一轮发送
    }
    return true;
}

/**
 * @brief 先写命令后读数据（复合传输）
 * @param self       I2C 对象指针
 * @param slave_addr 从机设备地址
 * @param cmd        待发送的命令数据
 * @param cmd_len    命令长度（不得超过 FIFO 最大深度）
 * @param data       接收数据缓冲区
 * @param data_len   预期接收数据长度
 * @param timeout    超时时间（微秒）
 * @return 成功返回 true，失败返回 false
 *
 * 利用 I2C 的 RD_ON_TXEMPTY 特性：先发送 TX FIFO 中的命令字节，
 * 待 TX FIFO 空后自动发起重复起始条件切换为接收模式读取数据。
 * 适用于 AT24Cxx 等需要先发地址再读数据的存储器类器件。
 */
static _Bool ReadAfterTransmit(EasyFrame_I2C_Typedef_t *self, uint8_t slave_addr, uint8_t *cmd,
                               uint16_t cmd_len, uint8_t *data, uint16_t data_len, uint32_t timeout) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error in i2c \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "I2C not inited \r\n");
        return false;
    }
    if (cmd_len > I2C_FIFO_MAX_SIZE) {
        RTT_Print(0, "I2C FIFO overflow \r\n");
        return false;
    }
    uint64_t start = GetTime();
    I2C_Regs *i2c = self->mspm0g.i2c;

    /* Wait for I2C to be Idle */
    while (!(DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (GetTime() - start > timeout) {
            return false;
        }
        Delay_us(20);
    }

    DL_I2C_flushControllerRXFIFO(i2c); // 清空RXFIFO
    DL_I2C_flushControllerTXFIFO(i2c); // 清空TXFIFO
    delay_cycles(self->mspm0g.i2c_delay);

    /* Enable hardware RD_ON_TXEMPTY before starting transfer:
     * TX FIFO bytes sent first (as write), then auto REPEATED START + read */
    // DL_I2C_enableControllerReadOnTXEmpty(i2c);
    DL_I2C_fillControllerTXFIFO(i2c, cmd, cmd_len);
    delay_cycles(self->mspm0g.i2c_delay);
    // 需要手动操作寄存器启动TX为空时开启接收
    i2c->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;


    DL_I2C_startControllerTransferAdvanced(i2c, slave_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, data_len,
        DL_I2C_CONTROLLER_START_ENABLE,
        DL_I2C_CONTROLLER_STOP_ENABLE,
        DL_I2C_CONTROLLER_ACK_DISABLE);
    delay_cycles(self->mspm0g.i2c_delay);

    /* Wait for transfer to complete */
    while (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (GetTime() - start > timeout) {
            // DL_I2C_disableControllerReadOnTXEmpty(i2c);
            i2c->MASTER.MCTR = 0;
            return false;
        }
        Delay_us(20);
    }

    /* Check for transfer error */
    if (DL_I2C_getControllerStatus(i2c) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        // DL_I2C_disableControllerReadOnTXEmpty(i2c);
            i2c->MASTER.MCTR = 0;
        return false;
    }

    /* Read received data from RX FIFO */
    for (uint16_t i = 0; i < data_len; ++i) {
        while (DL_I2C_isControllerRXFIFOEmpty(i2c)) {
            if (GetTime() - start > timeout) {
                // DL_I2C_disableControllerReadOnTXEmpty(i2c);
            i2c->MASTER.MCTR = 0;
                return false;
            }
            Delay_us(20);
        }
        data[i] = DL_I2C_receiveControllerData(i2c);
    }

    // DL_I2C_disableControllerReadOnTXEmpty(i2c);
            i2c->MASTER.MCTR = 0;
    return true;
}

/**
 * @brief I2C 总线恢复（软件位冲方式）
 * @param self        I2C 对象指针
 * @param need_change 是否执行 GPIO 位冲恢复流程
 * @return 成功返回 true，失败返回 false
 *
 * 当从机拉低 SCL/SDA 导致总线锁死时，将对应引脚切换为 GPIO 输出，
 * 通过发送 9 个 SCL 脉冲以及 START/STOP 信号来释放总线，
 * 最后恢复 IOMUX 配置并重新使能 I2C 控制器。
 */
static _Bool Reset(EasyFrame_I2C_Typedef_t *self, _Bool need_change) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error in i2c \r\n");
        return false;
    }
    if (self->is_inited == false) {
        RTT_Print(0, "I2C not inited \r\n");
        return false;
    }
    if (need_change) {
        EasyFrame_GPIO_Typedef_t *sda = &self->mspm0g.sda;
        EasyFrame_GPIO_Typedef_t *scl = &self->mspm0g.scl;
        I2C_Regs *i2c = self->mspm0g.i2c;

        /* 保存当前PINCM配置，用于恢复 */
        uint32_t sda_pincm_save = IOMUX->SECCFG.PINCM[sda->mspm0g.iomux];
        uint32_t scl_pincm_save = IOMUX->SECCFG.PINCM[scl->mspm0g.iomux];

        /* 停I2C控制器，清除BUSBSY */
        DL_I2C_disableController(i2c);

        /* SDA/SCL 切 GPIO 输出 */
        DL_GPIO_initDigitalOutput(sda->mspm0g.iomux);
        DL_GPIO_initDigitalOutput(scl->mspm0g.iomux);
        DL_GPIO_enableOutput(sda->mspm0g.gpio, sda->mspm0g.pin);
        DL_GPIO_enableOutput(scl->mspm0g.gpio, scl->mspm0g.pin);

        /* Idle: both high */
        sda->SetHigh(sda);
        scl->SetHigh(scl);
        Delay_us(20);

        /* Step 1: START (SDA↓, SCL=H) */
        sda->SetLow(sda);
        Delay_us(20);

        /* Step 2: 9 SCL pulses，释放SDA为Hi-Z避免与从机冲突 */
        DL_GPIO_disableOutput(sda->mspm0g.gpio, sda->mspm0g.pin);
        for (int i = 0; i < 9; i++) {
            scl->SetLow(scl);
            Delay_us(20);
            scl->SetHigh(scl);
            Delay_us(20);
        }

        /* Step 3: START (SDA↓) + STOP (SDA↑) */
        DL_GPIO_enableOutput(sda->mspm0g.gpio, sda->mspm0g.pin);
        sda->SetLow(sda);
        Delay_us(20);
        sda->SetHigh(sda);
        Delay_us(20);

        /* 恢复PINCM配置，切回I2C外设模式 */
        IOMUX->SECCFG.PINCM[sda->mspm0g.iomux] = sda_pincm_save;
        IOMUX->SECCFG.PINCM[scl->mspm0g.iomux] = scl_pincm_save;

        /* 重新使能I2C控制器 */
        DL_I2C_enableController(i2c);
    }
    return true;
}
