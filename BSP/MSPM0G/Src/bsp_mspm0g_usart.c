#include "bsp_mspm0g_usart.h"
#include "SEGGER_RTT.h"
#include "bsp_mspm0g_dma.h"
#include "bsp_mspm0g_it.h"
#include "bsp_mspm0g_tim_base.h"
#include "mcu_config.h"
#include "string.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_dma.h"
#include "ti/driverlib/dl_uart.h"
#include "ti/driverlib/dl_uart_main.h"
#include <stdbool.h>
#include <stdint.h>

#define Delay_us(X) EasyFrameSysTime_Delay_us(X)
#define GetTime() EasyFrameSysTime_GetTimeline_us()

/*串口实例*/
static EF_Usart_Typedef uart0;

/*Function define*/
static void TransmitByte(EF_Usart_Typedef *self, uint8_t data);
static void ReceiveByte(EF_Usart_Typedef *self, uint8_t *data);
static _Bool TransmitPoll(EF_Usart_Typedef *self, uint8_t *data, uint16_t len,
                          uint64_t timeout);
static _Bool ReceivePoll(EF_Usart_Typedef *self, uint8_t *data, uint16_t len,
                         uint64_t timeout);
static _Bool TransmitFIFOPoll(EF_Usart_Typedef *self, uint8_t *data,
                              uint16_t len, uint64_t timeout);
static _Bool ReceiveFIFOPoll(EF_Usart_Typedef *self, uint8_t *data,
                             uint16_t len, uint64_t timeout);
static _Bool TransmitDMA(EF_Usart_Typedef *self, uint8_t *buff, uint16_t len);
static _Bool ReceiveDMA(EF_Usart_Typedef *self, uint8_t *buff, uint16_t len);
static _Bool ReceiveDMA_IDLE(EF_Usart_Typedef *self, uint8_t *buff,
                             uint16_t len);
static _Bool Transmit_IT(EF_Usart_Typedef *self, uint8_t *data, uint16_t len);
static _Bool Receive_IT(EF_Usart_Typedef *self, uint8_t *data, uint16_t len);
static _Bool GetFlag_IT_RX(EF_Usart_Typedef *self);
static _Bool GetFlag_IT_TX(EF_Usart_Typedef *self);

/**
 * @brief 根据ID获取对应的串口实例指针。
 * @param id 串口编号（0 = UART0）。
 * @return 成功返回串口实例指针，无效ID返回NULL。
 */
EF_Usart_Typedef *EF_BSP_Uart_GetPtr(uint8_t id) {
  switch (id) {
  case 0:
    return &uart0;
  default:
    return NULL;
  }
}

/**
 * @brief 初始化串口实例，绑定各收发函数指针并清零结构体。
 * @param self 指向串口实例结构体的指针。
 * @param uart 指向UART外设寄存器的指针。
 * @retval true  初始化成功。
 * @retval false 初始化失败（self或uart为NULL）。
 */
_Bool EF_BSP_Uart_Init(EF_Usart_Typedef *self, UART_Regs *uart) {
  if (self == NULL || uart == NULL) {
    RTT_Print(0, "Null pointer error happen in uart init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_Usart_Typedef));
  self->mspm0g.uart = uart;
  self->ReceiveByte = ReceiveByte;
  self->TransmitByte = TransmitByte;
  self->TransmitPoll = TransmitPoll;
  self->ReceivePoll = ReceivePoll;
  self->TransmitFIFOPoll = TransmitFIFOPoll;
  self->ReceiveFIFOPoll = ReceiveFIFOPoll;
  self->TransmitDMA = TransmitDMA;
  self->ReceiveDMA = ReceiveDMA;
  self->ReceiveDMA_IDLE = ReceiveDMA_IDLE;
  self->Receive_IT = Receive_IT;
  self->Transmit_IT = Transmit_IT;
  self->GetFlag_IT_RX = GetFlag_IT_RX;
  self->GetFlag_IT_TX = GetFlag_IT_TX;
  self->is_inited = true;

  return true;
}

/**
 * @brief 初始化串口的中断收发功能，将回调注册到中断组。
 * @param self     指向串口实例结构体的指针。
 * @param rx_type  接收中断类型（NONE_IT_CALLBACK 表示不启用接收）。
 * @param tx_type  发送中断类型（NONE_IT_CALLBACK 表示不启用发送）。
 * @param RxCallback 接收中断回调函数指针（可为NULL）。
 * @param TxCallback 发送中断回调函数指针（可为NULL）。
 * @param irq      中断向量号。
 * @param priority 中断优先级。
 * @retval true  初始化成功。
 * @retval false 初始化失败（self为NULL）。
 */
_Bool EF_BSP_Uart_InitIT(EF_Usart_Typedef *self, EF_IT_e rx_type,
                         EF_IT_e tx_type, void (*RxCallback)(void *param),
                         void (*TxCallback)(void *param), IRQn_Type irq,
                         uint32_t priority) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart init it \r\n");
    return false;
  }
  EF_IT_Group_Typedef *temp_group = EF_BSP_IT_Group_GetUartPtr();
  self->mspm0g.it.irq = irq;
  NVIC_SetPriority(irq, priority);
  NVIC_ClearPendingIRQ(irq);
  if (rx_type != NONE_IT_CALLBACK) {
    EF_BSP_IT_Init(&self->mspm0g.it.rx_it, rx_type, RxCallback);
    if (EF_BSP_IT_Group_Add(temp_group, &self->mspm0g.it.rx_it)) {
      self->mspm0g.it.rx_is_inited = true;
    }
  }
  if (tx_type != NONE_IT_CALLBACK) {
    EF_BSP_IT_Init(&self->mspm0g.it.tx_it, tx_type, TxCallback);
    if (EF_BSP_IT_Group_Add(temp_group, &self->mspm0g.it.tx_it)) {
      self->mspm0g.it.tx_is_inited = true;
    }
  }
  return true;
}

/**
 * @brief 初始化串口的DMA收发功能，将DMA回调注册到中断组。
 * @param self           指向串口实例结构体的指针。
 * @param dma            指向DMA外设寄存器的指针。
 * @param rx_dma_channel 接收DMA通道号。
 * @param tx_dma_channel 发送DMA通道号。
 * @param rx_type        接收DMA中断类型（NONE_IT_CALLBACK 表示不启用接收）。
 * @param tx_type        发送DMA中断类型（NONE_IT_CALLBACK 表示不启用发送）。
 * @param RxCallback     接收DMA中断回调函数指针（可为NULL）。
 * @param TxCallback     发送DMA中断回调函数指针（可为NULL）。
 * @param irq            DMA中断向量号。
 * @param priority       DMA中断优先级。
 * @retval true  初始化成功。
 * @retval false 初始化失败（self或dma为NULL）。
 */
_Bool EF_BSP_Uart_InitNormalDMA(EF_Usart_Typedef *self, DMA_Regs *dma,
                                uint32_t rx_dma_channel,
                                uint32_t tx_dma_channel, EF_IT_e rx_type,
                                EF_IT_e tx_type,
                                void (*RxCallback)(void *param),
                                void (*TxCallback)(void *param), IRQn_Type irq,
                                uint32_t priority) {
  if (self == NULL || dma == NULL) {
    RTT_Print(0, "Null pointer error happen in uart init it \r\n");
    return false;
  }
  self->mspm0g.dma.dma_irq = irq;
  EF_IT_Group_Typedef *temp_group = EF_BSP_IT_Group_GetDMAPtr();
  if (rx_type != NONE_IT_CALLBACK) {

    if (!EF_BSP_DMA_Init(&self->mspm0g.dma.rx_dma, dma, rx_dma_channel, rx_type,
                         RxCallback, temp_group)) {
      return false;
    }
  }
  if (tx_type != NONE_IT_CALLBACK) {
    if (!EF_BSP_DMA_Init(&self->mspm0g.dma.tx_dma, dma, tx_dma_channel, tx_type,
                         TxCallback, temp_group)) {
      return false;
    }
  }
  NVIC_SetPriority(irq, priority);
  self->mspm0g.dma.is_inited = true;
  return true;
}

/**
 * @brief 初始化串口接收空闲中断，将空闲中断回调注册到中断组。
 * @param self     指向串口实例结构体的指针。
 * @param type     空闲中断类型。
 * @param Callback 空闲中断回调函数指针。
 * @retval true  初始化成功。
 * @retval false 初始化失败（self为NULL或中断注册失败）。
 */
_Bool EF_BSP_Uart_Init_IDLE_IT(EF_Usart_Typedef *self, EF_IT_e type,
                               void (*Callback)(void *param)) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart init idle it \r\n");
    return false;
  }
  EF_IT_Group_Typedef *temp_group = EF_BSP_IT_Group_GetUartPtr();
  if (!EF_BSP_IT_Init(&self->mspm0g.it.rx_idle.rx_idle_it,
                      UART0_IDLE_RX_CALLBACK, Callback)) {
    return false;
  }
  if (EF_BSP_IT_Group_Add(temp_group, &self->mspm0g.it.rx_idle.rx_idle_it)) {
    return false;
  }
  return true;
}
/**
 * @brief 发送单个字节（调用者需保证实例有效且已初始化）。
 * @param self 指向串口实例结构体的指针。
 * @param data 待发送的字节。
 */
static void TransmitByte(EF_Usart_Typedef *self, uint8_t data) {
  DL_UART_transmitData(self->mspm0g.uart, data);
}

/**
 * @brief 接收单个字节（调用者需保证实例有效且已初始化）。
 * @param self 指向串口实例结构体的指针。
 * @param data 用于存放接收字节的指针。
 */
static void ReceiveByte(EF_Usart_Typedef *self, uint8_t *data) {
  *data = DL_UART_receiveData(self->mspm0g.uart);
}

/**
 * @brief 逐字节轮询发送数据，带超时机制。
 * @param self    指向串口实例结构体的指针。
 * @param data    待发送数据的缓冲区指针。
 * @param len     待发送数据的长度。
 * @param timeout 超时时间（微秒）。
 * @retval true  发送完成。
 * @retval false 发送失败（空指针、未初始化或超时）。
 */
static _Bool TransmitPoll(EF_Usart_Typedef *self, uint8_t *data, uint16_t len,
                          uint64_t timeout) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart transmit \r\n");
    return false;
  }
  if (self->is_inited == false) {
    RTT_Print(0, "Uart not inited \r\n");
    return false;
  }
  uint64_t start = GetTime();
  for (uint16_t i = 0; i < len; i++) {
    while (DL_UART_isBusy(self->mspm0g.uart)) {
      if (GetTime() - start > timeout) {
        return false;
      }
    }
    self->TransmitByte(self, data[i]);
  }
  return true;
}

/**
 * @brief 逐字节轮询接收数据，带超时机制。
 * @param self    指向串口实例结构体的指针。
 * @param data    接收数据的缓冲区指针。
 * @param len     期望接收的数据长度。
 * @param timeout 超时时间（微秒）。
 * @retval true  接收完成。
 * @retval false 接收失败（空指针、未初始化或超时）。
 */
static _Bool ReceivePoll(EF_Usart_Typedef *self, uint8_t *data, uint16_t len,
                         uint64_t timeout) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart receive\r\n");
    return false;
  }
  if (self->is_inited == false) {
    RTT_Print(0, "Uart not inited \r\n");
    return false;
  }
  uint64_t start = GetTime();
  for (uint16_t i = 0; i < len; i++) {
    while (DL_UART_isRXFIFOEmpty(self->mspm0g.uart)) {
      if (GetTime() - start > timeout) {
        return false;
      }
    }
    self->ReceiveByte(self, &data[i]);
  }
  return true;
}

/**
 * @brief 利用TX FIFO批量发送数据，自动分包处理，带超时机制。
 *
 * 先等待FIFO清空，然后每次填入 UART_TX_FIFO_MAX_SIZE 字节，
 * 等待本次FIFO发送完成后再填下一批，最后发送剩余尾数。
 *
 * @param self    指向串口实例结构体的指针。
 * @param data    待发送数据的缓冲区指针。
 * @param len     待发送数据的长度。
 * @param timeout 超时时间（微秒）。
 * @retval true  发送完成。
 * @retval false 发送失败（空指针、未初始化或超时）。
 */
static _Bool TransmitFIFOPoll(EF_Usart_Typedef *self, uint8_t *data,
                              uint16_t len, uint64_t timeout) {
  // 只有小于等于FIFO MAX SIZE 才是直接发送
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart receive\r\n");
    return false;
  }
  if (self->is_inited == false) {
    RTT_Print(0, "Uart not inited \r\n");
    return false;
  }
  uint64_t start = GetTime();
  while (!DL_UART_isTXFIFOEmpty(self->mspm0g.uart)) {
    if (GetTime() - start > timeout) {
      return false;
    }
  }

  uint16_t send_time = len / UART_TX_FIFO_MAX_SIZE;
  uint8_t remain_size = len % UART_TX_FIFO_MAX_SIZE;
  for (uint16_t i = 0; i < send_time; i++) {
    DL_UART_fillTXFIFO(self->mspm0g.uart, &data[i * 4], 4);
    while (!DL_UART_isTXFIFOEmpty(self->mspm0g.uart)) {
      if (GetTime() - start > timeout) {
        return false;
      }
    }
  }
  DL_UART_fillTXFIFO(self->mspm0g.uart, &data[len - remain_size], remain_size);
  while (!DL_UART_isTXFIFOEmpty(self->mspm0g.uart)) {
    if (GetTime() - start > timeout) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 利用RX FIFO轮询接收数据，带超时机制。
 *
 * @warning 不建议使用此轮询版本，可能存在数据掺杂问题。
 *
 * @param self    指向串口实例结构体的指针。
 * @param data    接收数据的缓冲区指针。
 * @param len     期望接收的数据长度。
 * @param timeout 超时时间（微秒）。
 * @retval true  接收完成。
 * @retval false 接收失败（空指针、未初始化或超时）。
 */
static _Bool ReceiveFIFOPoll(EF_Usart_Typedef *self, uint8_t *data,
                             uint16_t len, uint64_t timeout) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart receive\r\n");
    return false;
  }
  if (self->is_inited == false) {
    RTT_Print(0, "Uart not inited \r\n");
    return false;
  }
  uint64_t start = GetTime();
  uint16_t read = 0;
  while (read < len) {
    if (GetTime() - start > timeout) {
      return false;
    }
    if (!DL_UART_isRXFIFOEmpty(self->mspm0g.uart)) {
      data[read] = DL_UART_receiveData(self->mspm0g.uart);
      read++;
    }
  }
  return true;
}

/**
 * @brief 通过DMA发送数据（骨架，待实现）。
 * @param self 指向串口实例结构体的指针。
 * @param buff 待发送数据缓冲区指针。
 * @param len  待发送数据的长度。
 * @retval true  发送完成。
 * @retval false 发送失败（空指针或未初始化）。
 */
static _Bool TransmitDMA(EF_Usart_Typedef *self, uint8_t *buff, uint16_t len) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart transmit\r\n");
    return false;
  }
  if (self->is_inited == false || self->mspm0g.dma.is_inited == false) {
    RTT_Print(0, "Uart not inited \r\n");
    return false;
  }

  uint8_t *addr = (buff == NULL)
                      ? (self->mspm0g.dma.tx_buffer)
                      : buff; // 设定地址 当输入地址为空指针时 设定为内置地址
  EF_DMA_Typedef *dma = &self->mspm0g.dma.tx_dma;
  if (!dma->Set(dma, addr, (uint8_t *)&self->mspm0g.uart->TXDATA, len)) {
    return false;
  }

  self->mspm0g.dma.tx_buffer_ptr = addr;
  self->mspm0g.dma.tx_size_set = len;
  NVIC_EnableIRQ(self->mspm0g.dma.dma_irq); // 启动DMA中断
  NVIC_EnableIRQ(self->mspm0g.it.irq);      // 启动串口中断
  if (!DL_DMA_isChannelEnabled(dma->mspm0g.dma, dma->mspm0g.channel)) {
    return false;
  }
  // DL_UART_enableInterrupt(self->mspm0g.uart,
  //                         DL_UART_MAIN_IIDX_DMA_DONE_tx); // 启用传输完成中断
  self->mspm0g.dma.tx_dma.EnableFullIRQ(
      &self->mspm0g.dma.tx_dma); // 启用DMA中断
  // DL_UART_enableInterrupt(uart0.mspm0g.uart,
  //                         DL_UART_INTERRUPT_TX); // 使用中断触发DMA
  return true;
}

/**
 * @brief 通过DMA接收数据 接收满中断。
 * @param self 指向串口实例结构体的指针。
 * @param buff 接收数据缓冲区指针。
 * @param len  期望接收的数据长度。
 * @retval true  接收完成。
 * @retval false 接收失败（空指针或未初始化）。
 */
static _Bool ReceiveDMA(EF_Usart_Typedef *self, uint8_t *buff, uint16_t len) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart transmit\r\n");
    return false;
  }
  if (self->is_inited == false || self->mspm0g.dma.is_inited == false) {
    RTT_Print(0, "Uart not inited \r\n");
    return false;
  }
  uint8_t *addr = (buff == NULL)
                      ? (self->mspm0g.dma.rx_buffer)
                      : buff; // 设定地址 当输入地址为空指针时 设定为内置地址
  if (!self->mspm0g.dma.rx_dma.Set(&self->mspm0g.dma.rx_dma,
                                   (uint8_t *)&self->mspm0g.uart->RXDATA, addr,
                                   len)) {
    return false;
  }
  self->mspm0g.dma.rx_buffer_ptr = addr;
  self->mspm0g.dma.rx_size_set = len;

  NVIC_EnableIRQ(self->mspm0g.dma.dma_irq); // 启动DMA中断
  NVIC_EnableIRQ(self->mspm0g.it.irq);      // 启动串口中断
  if (!DL_DMA_isChannelEnabled(self->mspm0g.dma.rx_dma.mspm0g.dma,
                               self->mspm0g.dma.rx_dma.mspm0g.channel)) {
    return false;
  }
  // DL_UART_enableInterrupt(self->mspm0g.uart,
  //                         DL_UART_MAIN_IIDX_DMA_DONE_RX); // 启用传输完成中断
  self->mspm0g.dma.rx_dma.EnableFullIRQ(
      &self->mspm0g.dma.rx_dma); // 启用DMA中断
  // DL_UART_enableInterrupt(uart0.mspm0g.uart,
  //                         DL_UART_INTERRUPT_RX); // 使用中断触发DMA
  return true;
}

/**
 * @brief 通过DMA配合空闲中断接收不定长数据。
 *
 * DMA 持续搬运数据到缓冲区，当串口总线空闲时触发空闲中断，
 * 由空闲中断回调计算实际接收长度并重启DMA通道。
 *
 * @param self 指向串口实例结构体的指针。
 * @param buff 接收数据缓冲区指针。
 * @param len  期望接收的最大数据长度。
 * @retval true  启动成功。
 * @retval false 启动失败（空指针或未初始化）。
 */
static _Bool ReceiveDMA_IDLE(EF_Usart_Typedef *self, uint8_t *buff,
                             uint16_t len) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart transmit\r\n");
    return false;
  }
  if (self->is_inited == false || self->mspm0g.dma.is_inited == false) {
    RTT_Print(0, "Uart not inited \r\n");
    return false;
  }
  uint8_t *addr = (buff == NULL)
                      ? (self->mspm0g.dma.rx_buffer)
                      : buff; // 设定地址 当输入地址为空指针时 设定为内置地址
  if (!self->mspm0g.dma.rx_dma.Set(&self->mspm0g.dma.rx_dma,
                                   (uint8_t *)&self->mspm0g.uart->RXDATA, addr,
                                   len)) {
    return false;
  }
  self->mspm0g.dma.rx_buffer_ptr = addr;
  self->mspm0g.dma.rx_size_set = len;

  // NVIC_EnableIRQ(self->mspm0g.dma.dma_irq); // 启动DMA中断
  NVIC_EnableIRQ(self->mspm0g.it.irq); // 启动串口中断
  if (!DL_DMA_isChannelEnabled(self->mspm0g.dma.rx_dma.mspm0g.dma,
                               self->mspm0g.dma.rx_dma.mspm0g.channel)) {
    return false;
  }
  DL_UART_enableInterrupt(self->mspm0g.uart,
                          DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR); // 启用空闲中断
  // self->mspm0g.dma.rx_dma.EnableFullIRQ(
  //     &self->mspm0g.dma.rx_dma); // 启用DMA中断
  // DL_UART_enableInterrupt(uart0.mspm0g.uart,
  //                         DL_UART_INTERRUPT_RX); // 使用中断触发DMA
  return true;
}

/**
 * @brief 启动中断方式发送，使能TX中断并重置发送状态。
 *
 * 调用后由 TX 中断回调逐批填充 FIFO 完成发送，
 * 可通过 GetFlag_IT_TX() 查询是否发送完毕。
 *
 * @param self 指向串口实例结构体的指针。
 * @param data 待发送数据的缓冲区指针。
 * @param len  待发送数据的长度。
 * @retval true  启动成功。
 * @retval false 启动失败（空指针或TX中断未初始化）。
 */
static _Bool Transmit_IT(EF_Usart_Typedef *self, uint8_t *data, uint16_t len) {
  if (self == NULL || data == NULL) {
    RTT_Print(0, "Null pointer error happen in uart receive\r\n");
    return false;
  }
  if (self->mspm0g.it.tx_is_inited == false) {
    RTT_Print(0, "Uart tx it not inited \r\n");
    return false;
  }
  self->mspm0g.it.tx_buffer = data;
  self->mspm0g.it.tx_size_set = len;
  self->mspm0g.it.tx_size_now = 0;
  self->mspm0g.it.flag_tx = 0;
  NVIC_EnableIRQ(self->mspm0g.it.irq);
  DL_UART_enableInterrupt(uart0.mspm0g.uart, DL_UART_INTERRUPT_TX);
  return true;
}
/**
 * @brief 启动中断方式接收，使能RX中断并重置接收状态。
 *
 * 调用后由 RX 中断回调从 FIFO 读取数据，
 * 可通过 GetFlag_IT_RX() 查询是否接收完毕。
 *
 * @param self 指向串口实例结构体的指针。
 * @param data 接收数据缓冲区指针。
 * @param len  期望接收的数据长度。
 * @retval true  启动成功。
 * @retval false 启动失败（空指针或RX中断未初始化）。
 */
static _Bool Receive_IT(EF_Usart_Typedef *self, uint8_t *data, uint16_t len) {
  if (self == NULL || data == NULL) {
    RTT_Print(0, "Null pointer error happen in uart receive\r\n");
    return false;
  }
  if (self->mspm0g.it.rx_is_inited == false) {
    RTT_Print(0, "Uart rx it not inited \r\n");
    return false;
  }
  self->mspm0g.it.rx_buffer = data;
  self->mspm0g.it.rx_size_set = len;
  self->mspm0g.it.rx_size_now = 0;
  self->mspm0g.it.flag_rx = 0;
  NVIC_EnableIRQ(self->mspm0g.it.irq);
  DL_UART_enableInterrupt(uart0.mspm0g.uart, DL_UART_INTERRUPT_RX);
  return true;
}

/**
 * @brief 查询接收中断是否已完成。
 * @param self 指向串口实例结构体的指针。
 * @retval true  接收已完成。
 * @retval false 接收未完成、空指针或未初始化。
 */
static _Bool GetFlag_IT_RX(EF_Usart_Typedef *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart receive\r\n");
    return false;
  }
  if (self->mspm0g.it.rx_is_inited == false) {
    RTT_Print(0, "Uart rx it not inited \r\n");
    return false;
  }
  return self->mspm0g.it.flag_rx;
}

/**
 * @brief 查询发送中断是否已完成。
 * @param self 指向串口实例结构体的指针。
 * @retval true  发送已完成。
 * @retval false 发送未完成、空指针或未初始化。
 */
static _Bool GetFlag_IT_TX(EF_Usart_Typedef *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happen in uart receive\r\n");
    return false;
  }
  if (self->mspm0g.it.tx_is_inited == false) {
    RTT_Print(0, "Uart tx it not inited \r\n");
    return false;
  }
  return self->mspm0g.it.flag_tx;
}

/*
 * ========================
 * 回调函数区
 * ========================
 * */

/**
 * @brief UART0 接收中断回调，从RX FIFO中读取全部可用数据。
 *
 * 每收到一字节存入 rx_buffer，当接收数量达到 rx_size_set
 * 时关闭 RX 中断并置位 flag_rx。
 *
 * @param param 回调参数（未使用）。
 */
void EF_BSP_Uart0_RxCallback(void *param) {
  while (!DL_UART_isRXFIFOEmpty(uart0.mspm0g.uart)) {
    uart0.mspm0g.it.rx_buffer[uart0.mspm0g.it.rx_size_now] =
        DL_UART_receiveData(uart0.mspm0g.uart);
    uart0.mspm0g.it.rx_size_now++;
  }
  DL_UART_clearInterruptStatus(uart0.mspm0g.uart, DL_UART_INTERRUPT_RX);
  if (uart0.mspm0g.it.rx_size_now == uart0.mspm0g.it.rx_size_set) {
    uart0.mspm0g.it.flag_rx = true;
    DL_UART_disableInterrupt(uart0.mspm0g.uart, DL_UART_INTERRUPT_RX);
  }
}

/**
 * @brief UART0 发送中断回调，分包将数据填入TX FIFO。
 *
 * TX FIFO为空时触发此回调。每次填入 UART_TX_FIFO_MAX_SIZE 字节，
 * 剩余不足一包时填入余数并关闭 TX 中断、置位 flag_tx。
 *
 * @param param 回调参数（未使用）。
 */
void EF_BSP_Uart0_TxCallback(void *param) {
  uint16_t last_num;
  DL_UART_clearInterruptStatus(uart0.mspm0g.uart, DL_UART_INTERRUPT_TX);
  last_num = uart0.mspm0g.it.tx_size_set - uart0.mspm0g.it.tx_size_now;
  if (last_num > UART_TX_FIFO_MAX_SIZE) {
    DL_UART_fillTXFIFO(uart0.mspm0g.uart,
                       &uart0.mspm0g.it.tx_buffer[uart0.mspm0g.it.tx_size_now],
                       UART_TX_FIFO_MAX_SIZE);
    uart0.mspm0g.it.tx_size_now += UART_TX_FIFO_MAX_SIZE;
  } else if (last_num > 0) {
    DL_UART_fillTXFIFO(uart0.mspm0g.uart,
                       &uart0.mspm0g.it.tx_buffer[uart0.mspm0g.it.tx_size_now],
                       last_num);
    uart0.mspm0g.it.tx_size_now += last_num;
    uart0.mspm0g.it.flag_tx = true;
    DL_UART_disableInterrupt(uart0.mspm0g.uart, DL_UART_INTERRUPT_TX);
  }
}

/**
 * @brief UART0 DMA接收完成回调，清除DMA中断标志后重新启动DMA接收。
 * @param param 回调参数（未使用）。
 */
void EF_BSP_Uart0_DMA_RxCallback(void *param) {
  // // 测试发送
  // uart0.TransmitPoll(&uart0, uart0.mspm0g.dma.rx_buffer_ptr,
  // uart0.mspm0g.dma.rx_size_set, 100000); 默认再次开启DMA
  uart0.mspm0g.dma.rx_dma.ClearIRQ(&uart0.mspm0g.dma.rx_dma); // 清除标志位
  uart0.ReceiveDMA(&uart0, uart0.mspm0g.dma.rx_buffer_ptr,
                   uart0.mspm0g.dma.rx_size_set);
}

/**
 * @brief UART0 DMA发送完成回调，清除DMA中断标志位。
 * @param param 回调参数（未使用）。
 */
void EF_BSP_Uart0_DMA_TxCallback(void *param) {
  // // 测试
  // uart0.TransmitPoll(&uart0, "abc", 4, 10000);
  uart0.mspm0g.dma.tx_dma.ClearIRQ(&uart0.mspm0g.dma.tx_dma); // 清除标志位
}

/**
 * @brief UART0 空闲中断回调，计算实际接收数据长度并重启DMA空闲接收。
 *
 * @warning 此函数只有在FIFO阈值设置为大于1就触发处理的状态下才能正常工作。
 *          如果阈值设置成 >= 1/2，还需要手动读取数据并存入接收缓冲区。
 *
 * @param param 回调参数（未使用）。
 */
void EF_BSP_Uart0_IDLE_RxCallback(void *param) {
  EF_DMA_Typedef *dma = &uart0.mspm0g.dma.rx_dma;
  uart0.mspm0g.it.rx_idle.rec_size =
      uart0.mspm0g.dma.rx_size_set -
      dma->mspm0g.dma->DMACHAN[dma->mspm0g.channel].DMASZ;
  DL_UART_clearInterruptStatus(uart0.mspm0g.uart, DL_UART_IIDX_RX_TIMEOUT_ERROR); // 清除标志位
  uart0.ReceiveDMA_IDLE(&uart0, uart0.mspm0g.dma.rx_buffer_ptr, uart0.mspm0g.dma.rx_size_set); // 默认再次开启DMA
}
