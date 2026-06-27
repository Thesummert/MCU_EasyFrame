#include "bsp_mspm0g_spi.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "SEGGER_RTT.h"
#include "bsp_mspm0g_gpio.h"
#include "bsp_mspm0g_tim_base.h"
#include "mcu_config.h"
#include "ti/driverlib/dl_spi.h"

#define Delay_us(X) EasyFrameSysTime_Delay_us(X)
#define GetTime()   EasyFrameSysTime_GetTimeline_us()

static void SPI_Select(EasyFrame_SPI_Typedef_t *self);
static void SPI_UnSelect(EasyFrame_SPI_Typedef_t *self);

/**
 * @brief 初始化 EasyFrame SPI 结构体
 *
 * @param self     EasyFrame SPI 结构体指针
 * @param cs_state 片选信号有效电平（1: 高电平有效，0: 低电平有效）
 * @param spi      SPI 外设寄存器指针
 * @param gpio     片选 GPIO 外设寄存器指针
 * @param pin      片选引脚编号
 * @return 初始化是否成功，true 表示成功，false 表示失败
 */
_Bool EasyFrame_SPI_Init(EasyFrame_SPI_Typedef_t *self, uint8_t cs_state, SPI_Regs *spi, GPIO_Regs *gpio,
                         uint32_t pin) {
    if (self == NULL || spi == NULL || gpio == NULL) {
        RTT_Print(0, "Null pointer error happened in spi init \r\n");
        return false;
    }
    memset(self, 0, sizeof(EasyFrame_SPI_Typedef_t));
    self->cs_state = cs_state;
    self->mspm0g.spi = spi;
    if (!EasyFrame_GPIO_Init(&self->mspm0g.cs_gpio, gpio, pin)) {
        return false;
    }
    self->Transmit = EasyFrame_SPI_Transmit;
    self->Receive = EasyFrame_SPI_Receive;
    self->TransmitAndReceive = EasyFrame_SPI_TransmitAndReceive;
    self->is_inited = true;

    self->Select = SPI_Select;
    self->UnSelect = SPI_UnSelect;
    return true;
}

/**
 * @brief 使能 SPI 片选信号
 *
 * @param self EasyFrame SPI 结构体指针
 */
static void SPI_Select(EasyFrame_SPI_Typedef_t *self) {
    if (self->is_inited == false) {
        RTT_Print(0, "SPI not inited! \r\n");
        return;
    }
    if (self->cs_state) {
        self->mspm0g.cs_gpio.SetHigh(&self->mspm0g.cs_gpio);
    } else {
        self->mspm0g.cs_gpio.SetLow(&self->mspm0g.cs_gpio);
    }
}

/**
 * @brief 失能 SPI 片选信号
 *
 * @param self EasyFrame SPI 结构体指针
 */
static void SPI_UnSelect(EasyFrame_SPI_Typedef_t *self) {
    if (self->is_inited == false) {
        RTT_Print(0, "SPI not inited! \r\n");
        return;
    }
    if (!self->cs_state) {
        self->mspm0g.cs_gpio.SetHigh(&self->mspm0g.cs_gpio);
    } else {
        self->mspm0g.cs_gpio.SetLow(&self->mspm0g.cs_gpio);
    }
}

/**
 * @brief SPI 发送数据
 *
 * @param self    EasyFrame SPI 结构体指针
 * @param data    待发送数据指针
 * @param size    发送数据长度
 * @param timeout 超时时间（单位：us）
 * @return 发送是否成功，true 表示成功，false 表示失败
 */
_Bool EasyFrame_SPI_Transmit(EasyFrame_SPI_Typedef_t *self, uint8_t *data, uint16_t size,
                             uint32_t timeout) {
    uint64_t start = GetTime();
    for (uint16_t i = 0; i < size; i++) {
        while (DL_SPI_isBusy(self->mspm0g.spi)) {
            if (GetTime() - start > timeout) {
                SPI_UnSelect(self);
                RTT_Print(0, "SPI Transmit Error \r\n");
                return false;
            }
            Delay_us(10000);
        }

        // uint8_t temp;
        DL_SPI_transmitData8(self->mspm0g.spi, data[i]);
        while (DL_SPI_isBusy(self->mspm0g.spi)); 
        // while (!DL_SPI_isRXFIFOEmpty(self->mspm0g.spi)) {
        //     temp = DL_SPI_receiveData8(self->mspm0g.spi);
        // }
    }
    return true;
}

/**
 * @brief SPI 接收数据
 *
 * @param self    EasyFrame SPI 结构体指针
 * @param data    接收数据缓冲区指针
 * @param size    接收数据长度
 * @param timeout 超时时间（单位：us）
 * @return 接收是否成功，true 表示成功，false 表示失败
 */
_Bool EasyFrame_SPI_Receive(EasyFrame_SPI_Typedef_t *self, uint8_t *data, uint16_t size,
                            uint32_t timeout) {
    uint64_t start = GetTime();
    for (uint16_t i = 0; i < size; ++i) {
        // 等待发送完成（TX FIFO 空 / SPI 不忙）
        while (DL_SPI_isBusy(self->mspm0g.spi)) {
            if (GetTime() - start > timeout) {
                SPI_UnSelect(self);
                return false;
            }
            Delay_us(10000);
        }
        while (!DL_SPI_isRXFIFOEmpty(self->mspm0g.spi)) {
            DL_SPI_receiveData8(self->mspm0g.spi);
        }
        // 发送 dummy 数据，触发时钟
        DL_SPI_transmitData8(self->mspm0g.spi, 0xFF);

        // while (!DL_SPI_isRXFIFOEmpty(self->mspm0g.spi)) {
        //     if (GetTime() - start > timeout) {
        //         SPI_UnSelect(self);
        //         return false;
        //     }
        //     Delay_us(10000);
        // }

        // 读取数据
        data[i] = DL_SPI_receiveDataBlocking8(self->mspm0g.spi);
    }

    return true;
}

/**
 * @brief SPI 发送并接收数据（待实现）
 *
 * @param self      EasyFrame SPI 结构体指针
 * @param rx_data   接收数据缓冲区指针
 * @param rx_size   接收数据长度
 * @param tx_buffer 发送数据缓冲区指针
 * @param tx_length 发送数据长度
 * @param timeout   超时时间（单位：us）
 * @return 操作是否成功，true 表示成功，false 表示失败
 */
_Bool EasyFrame_SPI_TransmitAndReceive(EasyFrame_SPI_Typedef_t *self, uint8_t *rx_data, uint16_t rx_size,
                                       uint8_t *tx_buffer, uint16_t tx_length, uint32_t timeout) {
    uint64_t start = GetTime();
    for (uint16_t i = 0; i < rx_size; ++i) {
        while (DL_SPI_isBusy(self->mspm0g.spi)) {
            if (GetTime() - start > timeout) {
                SPI_UnSelect(self);
                return false;
            }
            Delay_us(10000);
        }

        // 当接收数量大于发送数量时 主动发送dummy
        if (i < tx_length) {
            DL_SPI_transmitData8(self->mspm0g.spi, tx_buffer[i]);
        } else {
            DL_SPI_transmitData8(self->mspm0g.spi, 0xFF);
        }

        // while (DL_SPI_isRXFIFOEmpty(self->mspm0g.spi)) {
        //     if (GetTime() - start > timeout) {
        //         SPI_UnSelect(self);
        //         return false;
        //     }
        //     Delay_us(10000);
        // }

        rx_data[i] = DL_SPI_receiveDataBlocking8(self->mspm0g.spi);
    }

    return true;
}
