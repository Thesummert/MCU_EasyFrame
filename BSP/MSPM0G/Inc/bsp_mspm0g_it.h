#ifndef __BSP_MSPM0G_IT_H__
#define __BSP_MSPM0G_IT_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "mcu_config.h"
#include "ti_msp_dl_config.h"

/*中断向量种类*/
typedef enum {
    // 从1开始计数 0 代表无中断
    NONE_IT_CALLBACK = 0,
  UART0_RX_CALLBACK = 1,
  UART0_TX_CALLBACK,
  UART0_DMA_RX_CALLBACK,
  UART0_DMA_TX_CALLBACK,
  UART0_IDLE_RX_CALLBACK,
  UART_0_DMA_TX_Callback,
  UART_0_DMA_RX_Callback,

} EF_IT_e;

/*中断结构体*/
typedef struct EF_IT_Typedef {
  _Bool it_flag;                 // 中断标志位
  _Bool is_inited;               // 是否初始化
  EF_IT_e type;                  // 中断向量种类
  void (*Callback)(void *param); // 中断回调函数
} EF_IT_Typedef;

/*中断组合结构体*/
typedef struct EF_IT_Group_Typedef {
  uint8_t num;                               // 已经注册的中断结构体数量
  EF_IT_Typedef *it[IT_GROUP_ITEM_MAX_SIZE]; // 中断数组
} EF_IT_Group_Typedef;

_Bool EF_BSP_IT_Init(EF_IT_Typedef *self, EF_IT_e type, void (*Callback)(void *param));

void EF_BSP_IT_Group_Init();
_Bool EF_BSP_IT_Group_Add(EF_IT_Group_Typedef *self, EF_IT_Typedef *it);

EF_IT_Group_Typedef *EF_BSP_IT_Group_GetUartPtr();
EF_IT_Group_Typedef *EF_BSP_IT_Group_GetDMAPtr();

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MSPM0G_IT_H__ */
