#include "bsp_mspm0g_it.h"
#include "SEGGER_RTT.h"
#include "mcu_config.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_dma.h"
#include "ti/driverlib/dl_mcan.h"
#include "ti/driverlib/dl_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_can.h"

static void CallbackRunner(EF_IT_Group_Typedef *self, EF_IT_e type,
                           void *param);
static _Bool IT_Group_Init(EF_IT_Group_Typedef *self);

static EF_IT_Group_Typedef uart_it_group; // 串口中断组合
static EF_IT_Group_Typedef dma_it_group;  // DMA 中断组合
static EF_IT_Group_Typedef can_it_group;  // CAN 中断组合

/**
 * @brief 初始化中断对象。
 * @param self 指向EF_IT_Typedef结构体的指针。
 * @param Callback 中断回调函数指针。
 * @retval true 初始化成功。
 * @retval false 初始化失败（如self为NULL）。
 */
_Bool EF_BSP_IT_Init(EF_IT_Typedef *self, EF_IT_e type,
                     void (*Callback)(void *param)) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happened in it init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_IT_Typedef));
  self->type = type;
  self->Callback = Callback;

  self->is_inited = true;
  return true;
}

/**
 * @brief 初始化中断组。
 *
 * 此函数用于初始化中断组。
 */
void EF_BSP_IT_Group_Init() {
  IT_Group_Init(&uart_it_group);
  IT_Group_Init(&dma_it_group);
  IT_Group_Init(&can_it_group);
}

/**
 * @brief 初始化中断组对象。
 * @param self 指向EF_IT_Group_Typedef结构体的指针。
 * @retval true 初始化成功。
 * @retval false 初始化失败（如self为NULL）。
 */
static _Bool IT_Group_Init(EF_IT_Group_Typedef *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happened in it group init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_IT_Group_Typedef));
  return true;
}

/**
 * @brief 向中断组添加一个中断对象。
 * @param self 指向EF_IT_Group_Typedef结构体的指针。
 * @param it 指向EF_IT_Typedef结构体的指针。
 * @retval true 添加成功。
 * @retval false 添加失败（如self或it为NULL，或超出最大数量）。
 */
_Bool EF_BSP_IT_Group_Add(EF_IT_Group_Typedef *self, EF_IT_Typedef *it) {
  if (self == NULL || it == NULL) {
    RTT_Print(0, "Null pointer error happened in it group add \r\n");
    return false;
  }
  // 判断是否超出大小
  if (self->num >= IT_GROUP_ITEM_MAX_SIZE) {
    RTT_Print(0, "Exceeds the max it num \r\n");
    return false;
  }
  self->it[self->num] = it;
  self->num++;
  return true;
}

/**
 * @brief  执行与指定类型匹配的中断回调函数。
 *
 * @param  self  指向中断组结构体的指针。
 * @param  type  需要匹配的中断类型。
 * @param  param 回调函数的参数指针。
 *
 * @note   如果 self 为 NULL，则函数直接返回。
 *         遍历中断组中的所有中断项，若类型匹配，则调用对应的回调函数。
 */
static void CallbackRunner(EF_IT_Group_Typedef *self, EF_IT_e type,
                           void *param) {
  if (self == NULL) {
    return;
  }
  for (uint8_t i = 0; i < self->num; i++) {
    if (self->it[i]->type == type) {
      if (self->it[i]->Callback != NULL) {
        self->it[i]->Callback(param);
      }
      return;
    }
  }
}

EF_IT_Group_Typedef *EF_BSP_IT_Group_GetUartPtr() { return &uart_it_group; }

EF_IT_Group_Typedef *EF_BSP_IT_Group_GetDMAPtr() { return &dma_it_group; }

EF_IT_Group_Typedef *EF_BSP_IT_Group_GetCANPtr() { return &can_it_group; }

/*
 * ============================
 * 此处为具体的IQR handler实现
 * 先不做更抽象的封装 需手动实现
 * ============================
 **/

void UART_0_INST_IRQHandler(void) {
  switch (DL_UART_getPendingInterrupt(UART0)) {
  case DL_UART_IIDX_RX: // 串口RX回调
    CallbackRunner(&uart_it_group, UART0_RX_CALLBACK, NULL);
    break;
  case DL_UART_IIDX_TX: // 串口TX回调
    CallbackRunner(&uart_it_group, UART0_TX_CALLBACK, NULL);
    break;
  case DL_UART_IIDX_RX_TIMEOUT_ERROR:
    CallbackRunner(&uart_it_group, UART0_IDLE_RX_CALLBACK, NULL);
  default:
    break;
  }
}

void DMA_IRQHandler(void) {
  uint32_t id = DL_DMA_getPendingInterrupt(DMA);
  switch (id) {
  case DL_DMA_EVENT_IIDX_DMACH1:
    CallbackRunner(&dma_it_group, UART0_DMA_RX_CALLBACK, NULL);
    break;
  case DL_DMA_EVENT_IIDX_DMACH0:
    CallbackRunner(&dma_it_group, UART0_DMA_TX_CALLBACK, NULL);
    break;
  default:
    break;
  }
}

void CANFD0_IRQHandler(void) {
  EF_CAN_IT_CallbackPass_t pass;
  uint32_t can_it;
  pass.can = MCAN0_INST;
  switch (DL_MCAN_getPendingInterrupt(MCAN0_INST)) {
  case DL_MCAN_IIDX_LINE0:
  case DL_MCAN_IIDX_LINE1:
    can_it = DL_MCAN_getIntrStatus(MCAN0_INST);
    if ((can_it & DL_MCAN_INTERRUPT_RF0N)) {
      pass.canid = 0;
      CallbackRunner(&can_it_group, MCAN0_RXFIFO_0_Callback, &pass);
    }
    if ((can_it & DL_MCAN_INTERRUPT_RF1N)) {
      pass.canid = 1;
      CallbackRunner(&can_it_group, MCAN0_RXFIFO_1_Callback, &pass);
    }

    break;
  default:
    break;
  }
}
