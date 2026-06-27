#ifndef __BSP_MSPM0G_USART_H__
#define __BSP_MSPM0G_USART_H__

#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/devices/msp/peripherals/hw_dma.h"
#include "ti/devices/msp/peripherals/hw_uart.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_config.h"
#include "ti_msp_dl_config.h"

#if UART_ENABLE_DMA
#include "bsp_mspm0g_dma.h"
#endif
#if UART_ENABLE_IT
#include "bsp_mspm0g_it.h"
#endif

typedef struct EF_Usart_Typedef {
  _Bool is_inited;

  void (*TransmitByte)(struct EF_Usart_Typedef *self, uint8_t data);
  void (*ReceiveByte)(struct EF_Usart_Typedef *self, uint8_t *data);
  _Bool (*TransmitPoll)(struct EF_Usart_Typedef *self, uint8_t *data,
                        uint16_t len, uint64_t timeout);
  _Bool (*ReceivePoll)(struct EF_Usart_Typedef *self, uint8_t *data,
                       uint16_t len, uint64_t timeout);
  _Bool (*TransmitFIFOPoll)(struct EF_Usart_Typedef *self, uint8_t *data,
                            uint16_t len, uint64_t timeout);
  _Bool (*ReceiveFIFOPoll)(struct EF_Usart_Typedef *self, uint8_t *data,
                           uint16_t len, uint64_t timeout);
  _Bool (*TransmitDMA)(struct EF_Usart_Typedef *self, uint8_t *buff,
                       uint16_t len);
  _Bool (*ReceiveDMA)(struct EF_Usart_Typedef *self, uint8_t *buff,
                      uint16_t len);
_Bool (*ReceiveDMA_IDLE)(struct EF_Usart_Typedef *self, uint8_t *buff,
                             uint16_t len);
  _Bool (*Receive_IT)(struct EF_Usart_Typedef *self, uint8_t *data,
                      uint16_t len);
  _Bool (*Transmit_IT)(struct EF_Usart_Typedef *self, uint8_t *data,
                       uint16_t len);

  _Bool (*GetFlag_IT_RX)(struct EF_Usart_Typedef *self);
  _Bool (*GetFlag_IT_TX)(struct EF_Usart_Typedef *self);

  void (*DMA_Callback)();
  struct {
    UART_Regs *uart; // 串口示例
    struct {
#if UART_ENABLE_DMA == 1
      // DMA缓冲区 由于MSPM0G无需处理内存区域 直接在此处定义
      uint8_t rx_buffer[UART_RX_DMA_BUFFER_SIZE];
      uint8_t tx_buffer[UART_TX_DMA_BUFFER_SIZE];
      uint8_t rx_size_set; // DMA RX设定大小
      uint8_t tx_size_set; // DMA TX设定大小
      // 保存指针地址
      uint8_t *rx_buffer_ptr;
      uint8_t *tx_buffer_ptr;

      EF_DMA_Typedef rx_dma;
      EF_DMA_Typedef tx_dma;
      IRQn_Type dma_irq;
      _Bool is_inited; // 是否完成初始化
#endif
    } dma;
    struct {
#if UART_ENABLE_IT
      IRQn_Type irq; // 中断实例

      EF_IT_Typedef rx_it;      // 接收中断
      EF_IT_Typedef tx_it;      // 发送中断
      uint8_t *rx_buffer;       // 接收缓冲区
      uint8_t *tx_buffer;       // 发送缓冲区
      uint16_t rx_size_set;     // 中断RX设定大小
      uint16_t tx_size_set;     // 中断TX设定大小
      uint16_t rx_size_now;     // 中断RX实际大小
      uint16_t tx_size_now;     // 中断TX实际大小
      _Bool flag_rx;            // RX中断完成标志位
      _Bool flag_tx;            // TX中断完成标志位
      _Bool rx_is_inited;       // RX中断是否初始化标志位
      _Bool tx_is_inited;       // TX中断是否初始化标志位
                                //
      struct
      {
      EF_IT_Typedef rx_idle_it; // 接收空闲中断
      uint16_t rec_size; // 接收的数据大小
      }rx_idle;
#endif
    } it;
  } mspm0g;
} EF_Usart_Typedef;

_Bool EF_BSP_Uart_Init(EF_Usart_Typedef *self, UART_Regs *uart);
_Bool EF_BSP_Uart_InitIT(EF_Usart_Typedef *self, EF_IT_e rx_type,
                         EF_IT_e tx_type, void (*RxCallback)(void *param),
                         void (*TxCallback)(void *param), IRQn_Type irq,
                         uint32_t priority);
_Bool EF_BSP_Uart_InitNormalDMA(EF_Usart_Typedef *self, DMA_Regs *dma,
                                uint32_t rx_dma_channel,
                                uint32_t tx_dma_channel, EF_IT_e rx_type,
                                EF_IT_e tx_type,
                                void (*RxCallback)(void *param),
                                void (*TxCallback)(void *param), IRQn_Type irq,
                                uint32_t priority);
_Bool EF_BSP_Uart_Init_IDLE_IT(EF_Usart_Typedef *self, EF_IT_e type, void (*Callback)(void *param));
EF_Usart_Typedef *EF_BSP_Uart_GetPtr(uint8_t id);
/*此处定义现成的回调函数 可在外部修改成自定义*/
void EF_BSP_Uart0_RxCallback(void *param);
void EF_BSP_Uart0_TxCallback(void *param);
void EF_BSP_Uart0_DMA_RxCallback(void *param);
void EF_BSP_Uart0_DMA_TxCallback(void *param);
void EF_BSP_Uart0_IDLE_RxCallback(void *param);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MSPM0G_USART_H__ */
