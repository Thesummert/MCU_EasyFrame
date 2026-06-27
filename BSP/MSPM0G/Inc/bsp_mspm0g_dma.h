#ifndef __BSP_MSPM0G_DMA_H__
#define __BSP_MSPM0G_DMA_H__

#include "bsp_mspm0g_it.h"
#include "ti/devices/msp/peripherals/hw_dma.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "ti_msp_dl_config.h"

typedef struct EF_DMA_Typedef {
  EF_IT_Typedef it; // 中断
  struct {
    DMA_Regs *dma;
    uint32_t channel;
  } mspm0g;
  _Bool is_inited;     // 是否完成初始化
  _Bool transfer_flag; // 传输完成标志位

  /*函数指针*/
  _Bool (*Set)(struct EF_DMA_Typedef *self, uint8_t *src_ptr,
               uint8_t *target_ptr, uint32_t size);

  _Bool (*EnableFullIRQ)(struct EF_DMA_Typedef *self);
  void (*ClearIRQ)(struct EF_DMA_Typedef *self);
} EF_DMA_Typedef;

_Bool EF_BSP_DMA_Init(EF_DMA_Typedef *self, DMA_Regs *dma, uint32_t channel,
                      EF_IT_e type, void (*Callback)(void *param),
                      EF_IT_Group_Typedef *it_group);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MSPM0G_DMA_H__ */
