#ifndef __BSP_MSPM0G_SPI_H__
#define __BSP_MSPM0G_SPI_H__

#include <stdint.h>
#include "ti/devices/msp/peripherals/hw_spi.h"
#ifdef __cplusplus
extern "C" {
#endif

#include "ti_msp_dl_config.h"
#include "bsp_mspm0g_gpio.h"
#include "mcu_config.h"

// EasyFrame SPI 结构体
typedef struct EasyFrame_SPI_Typedef_t {
    _Bool is_inited;
    uint8_t cs_state;  // 片选时有效电平设置
    _Bool (*Transmit)(struct EasyFrame_SPI_Typedef_t *self, uint8_t *data, uint16_t size, uint32_t timeout);
    _Bool (*Receive)(struct EasyFrame_SPI_Typedef_t *self, uint8_t *data, uint16_t size, uint32_t timeout);
    _Bool (*TransmitAndReceive)(struct EasyFrame_SPI_Typedef_t *self, uint8_t *rx_data, uint16_t rx_size, uint8_t *tx_buffer, uint16_t tx_length, uint32_t timeout);

    void (*Select)(struct EasyFrame_SPI_Typedef_t *self);
    void (*UnSelect)(struct EasyFrame_SPI_Typedef_t *self);
    struct {
        SPI_Regs *spi;
        EasyFrame_GPIO_Typedef_t cs_gpio;
        // 缓冲区
        uint8_t rxBuffer[SPI_BUFFER_SIZE];
        uint8_t txBuffer[SPI_BUFFER_SIZE];

    } mspm0g;
} EasyFrame_SPI_Typedef_t;

_Bool EasyFrame_SPI_Init(EasyFrame_SPI_Typedef_t *self, uint8_t cs_state, SPI_Regs *spi, GPIO_Regs *gpio,
                         uint32_t pin);
_Bool EasyFrame_SPI_Transmit(EasyFrame_SPI_Typedef_t *self, uint8_t *data, uint16_t size,
                             uint32_t timeout);
_Bool EasyFrame_SPI_Receive(EasyFrame_SPI_Typedef_t *self, uint8_t *data, uint16_t size,
                            uint32_t timeout);
_Bool EasyFrame_SPI_TransmitAndReceive(EasyFrame_SPI_Typedef_t *self, uint8_t *rx_data, uint16_t rx_size,
                                       uint8_t *tx_buffer, uint16_t tx_length, uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MSPM0G_SPI_H__ */
