#include "bsp_mspm0g_dma.h"
#include "SEGGER_RTT.h"
#include "bsp_mspm0g_it.h"
#include "mcu_config.h"
#include "ti/driverlib/dl_dma.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/reent.h>
#include <wchar.h>

static _Bool Set(EF_DMA_Typedef *self, uint8_t *src_ptr, uint8_t *target_ptr,
                 uint32_t size);
static _Bool EnableFullIRQ(EF_DMA_Typedef *self);

static void ClearIRQ(EF_DMA_Typedef *self);

/**
 * @brief 初始化DMA通道实例，绑定操作函数指针并将中断回调注册到中断组。
 * @param self     指向DMA实例结构体的指针。
 * @param dma      指向DMA外设寄存器的指针。
 * @param channel  DMA通道号。
 * @param type     中断类型。
 * @param Callback 中断回调函数指针。
 * @param it_group 指向中断组结构体的指针，用于注册中断。
 * @retval true  初始化成功。
 * @retval false 初始化失败（self或it_group为NULL）。
 */
_Bool EF_BSP_DMA_Init(EF_DMA_Typedef *self, DMA_Regs *dma, uint32_t channel,
                      EF_IT_e type, void (*Callback)(void *param),
                      EF_IT_Group_Typedef *it_group) {
  if (self == NULL || it_group == NULL) {
    RTT_Print(0, "Null pointer error happened in dma init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_DMA_Typedef));
  self->mspm0g.dma = dma;
  self->mspm0g.channel = channel;
  self->Set = Set;
  self->EnableFullIRQ = EnableFullIRQ;
  self->ClearIRQ = ClearIRQ;
  EF_BSP_IT_Init(&self->it, type, Callback);
  EF_BSP_IT_Group_Add(it_group, &self->it);
  self->is_inited = true;
  return true;
}

/**
 * @brief 配置DMA源地址、目标地址及传输大小，并使能DMA通道。
 * @param self       指向DMA实例结构体的指针。
 * @param src_ptr    源地址指针。
 * @param target_ptr 目标地址指针。
 * @param size       传输数据大小。
 * @retval true  配置成功。
 * @retval false 配置失败（空指针或未初始化）。
 */
static _Bool Set(EF_DMA_Typedef *self, uint8_t *src_ptr, uint8_t *target_ptr,
                 uint32_t size) {
  if (self == NULL || src_ptr == NULL || target_ptr == NULL) {
    RTT_Print(0, "Null pointer error happened in dma set \r\n");
    return false;
  }
  if (self->is_inited == false) {
    RTT_Print(0, "DMA not inited \r\n");
    return false;
  }
  DL_DMA_setSrcAddr(self->mspm0g.dma, self->mspm0g.channel, (uint32_t)src_ptr);
  DL_DMA_setDestAddr(self->mspm0g.dma, self->mspm0g.channel,
                     (uint32_t)target_ptr);
  DL_DMA_setTransferSize(self->mspm0g.dma, self->mspm0g.channel, size);
  DL_DMA_enableChannel(self->mspm0g.dma, self->mspm0g.channel);

  return true;
}

/**
 * @brief 使能DMA通道的传输完成中断。
 *
 * 根据通道号选择对应的中断掩码，然后使能DMA中断。
 *
 * @param self 指向DMA实例结构体的指针。
 * @retval true  使能成功。
 * @retval false 使能失败（空指针或未初始化）。
 */
static _Bool EnableFullIRQ(EF_DMA_Typedef *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error happened in dma enable irq \r\n");
    return false;
  }
  if (self->is_inited == false) {
    RTT_Print(0, "DMA not inited \r\n");
    return false;
  }
  uint32_t it_mask;
  switch (self->mspm0g.channel) {
  case UART0_RX_DMA_CHANNEL_CHAN_ID:
    it_mask = DMA_CPU_INT_IMASK_DMACH1_SET;
    break;
  case UART0_TX_DMA_CHANNEL_CHAN_ID:
    it_mask = DMA_CPU_INT_IMASK_DMACH0_SET;
    break;
  default:
    break;
  }
  DL_DMA_enableInterrupt(self->mspm0g.dma, it_mask);
  return true;
}

/**
 * @brief 清除DMA通道的中断标志位。
 *
 * 根据通道号选择对应的中断掩码，然后清除该通道的中断状态。
 *
 * @param self 指向DMA实例结构体的指针。
 */
static void ClearIRQ(EF_DMA_Typedef *self)
{
  uint32_t it_mask;
  switch (self->mspm0g.channel) {
  case UART0_TX_DMA_CHANNEL_CHAN_ID:
    it_mask = DMA_CPU_INT_IMASK_DMACH0_SET;
    break;
  case UART0_RX_DMA_CHANNEL_CHAN_ID:
    it_mask = DMA_CPU_INT_IMASK_DMACH1_SET;
    break;
  default:
    break;
  }
  DL_DMA_clearInterruptStatus(self->mspm0g.dma, it_mask);
}
