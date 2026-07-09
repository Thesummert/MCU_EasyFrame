#include "bsp_can.h"
#include "bsp_mspm0g_it.h"
#include "mcu_config.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/devices/msp/peripherals/hw_mcan.h"
#include "ti/driverlib/dl_mcan.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

const uint8_t CAN_DLC_TO_LENGTH[16] = {
    [0] = 0, // EF_CAN_DLC_NUM0
    [1] = 1,   [2] = 2, [3] = 3, [4] = 4, [5] = 5, [6] = 6, [7] = 7, [8] = 8,
    [9] = 12,  // EF_CAN_DLC_12
    [10] = 16, // EF_CAN_DLC_16
    [11] = 20, // EF_CAN_DLC_20
    [12] = 24, // EF_CAN_DLC_24
    [13] = 32, // EF_CAN_DLC_32
    [14] = 48, // EF_CAN_DLC_48
    [15] = 64, // EF_CAN_DLC_64
};

static void RX_Callback(void *can_ptr);
static _Bool DecodeData(uint32_t id, uint32_t dlc, const uint8_t *data,
                        uint32_t xtd, uint32_t rtr);

static _Bool TransmitFIFO(EF_BSP_CAN_t *self, uint32_t id, uint8_t *data,
                          uint16_t len, _Bool is_ext_id);

static _Bool StartRec(EF_BSP_CAN_t *self);
static _Bool DecodeData(uint32_t id, uint32_t dlc, const uint8_t *data,
                        uint32_t xtd, uint32_t rtr) {
  /*按照具体应用填写 暂时先不做抽象处理*/
  return true;
}

_Bool EF_BSP_CAN_Init(EF_BSP_CAN_t *self, MCAN_Regs *can, _Bool is_fdcan,
                      _Bool is_baud_switch) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in can init \r\n");
    return false;
  }

  self->is_fdcan = is_fdcan;
  self->is_baud_switch = is_baud_switch;
  self->mspm0g.can = can;

  self->TransmitFIFO = TransmitFIFO;
  self->StartRec = StartRec;

  self->is_inited = true;
  return true;
}

_Bool EF_BSP_CAN_InitIT(EF_BSP_CAN_t *self, IRQn_Type irq, uint32_t priority,
                        EF_IT_e rx0, EF_IT_e rx1) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in can init it \r\n");
    return false;
  }
  self->irq = irq;
  EF_BSP_IT_Init(&self->rx0it, rx0, RX_Callback);
  EF_BSP_IT_Init(&self->rx1it, rx1, RX_Callback);
  if (EF_BSP_IT_Group_Add(EF_BSP_IT_Group_GetCANPtr(), &self->rx0it) == 0 || EF_BSP_IT_Group_Add(EF_BSP_IT_Group_GetCANPtr(), &self->rx1it) == 0) {
      return false;
  }
  return true;
}

static _Bool TransmitFIFO(EF_BSP_CAN_t *self, uint32_t id, uint8_t *data,
                          uint16_t len, _Bool is_ext_id) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in can transmit \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  if (len > CAN_DLC_TO_LENGTH[self->max_dlc_len]) {
    // 超出fifo数据DLC大小
    return false;
  }
  DL_MCAN_TxBufElement tx_msg;
  DL_MCAN_TxFIFOStatus tx_status;

  uint8_t dlc = len;

  if (len > 8) {
    uint8_t index = 8;
    /*FDCAN如果超过8位 发送数据长度是固定的 取最小的刚好比它大的那位*/
    while (len > CAN_DLC_TO_LENGTH[index]) {
      index++;
    }
    dlc = index;
  }
  tx_msg.id = id;
  tx_msg.rtr = 0;   // 发送数据帧
  tx_msg.dlc = dlc; // 设定发送长度
  tx_msg.xtd = is_ext_id;
  tx_msg.fdf = self->is_fdcan;
  tx_msg.esi = 0; // 不发送错误帧
  tx_msg.efc = 0; // 不记录TX
  tx_msg.mm = 0;

  memcpy(tx_msg.data, data, len); // 搬运数据
  DL_MCAN_writeMsgRam(self->mspm0g.can, DL_MCAN_MEM_TYPE_FIFO, 0,
                      &tx_msg);                             // 将数据写到fifo中
  DL_MCAN_getTxFIFOQueStatus(self->mspm0g.can, &tx_status); // 获取发送index
  DL_MCAN_TXBufAddReq(self->mspm0g.can, tx_status.putIdx);  // 添加发送请求

  return true;
}

static _Bool StartRec(EF_BSP_CAN_t *self) {

  if (self == NULL) {
    RTT_Print(0, "Null pointer error in can transmit \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  NVIC_EnableIRQ(self->irq);
  DL_MCAN_enableInterrupt(self->mspm0g.can,  DL_MCAN_INTERRUPT_RF0N);
  DL_MCAN_enableInterrupt(self->mspm0g.can,  DL_MCAN_INTERRUPT_RF1N);
  return true;
}

static void RX_Callback(void *can_ptr) {
  EF_CAN_IT_CallbackPass_t *pass = (EF_CAN_IT_CallbackPass_t *)can_ptr;
  MCAN_Regs *mcan = (MCAN_Regs *)pass->can;
  uint8_t fifo_id = pass->canid;
  DL_MCAN_RxBufElement rx_msg;
  DL_MCAN_RxFIFOStatus rx_status;
  rx_status.num = fifo_id;
  DL_MCAN_getRxFIFOStatus(mcan, &rx_status);
  while (rx_status.fillLvl > 0) {
    DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO,
                       0, // bufNum (FIFO模式忽略)
                       fifo_id, &rx_msg);
    uint32_t xtd = rx_msg.xtd; // 0=标准帧, 1=扩展帧
    uint32_t id;               // CAN ID (11或29位)

    if (xtd == 0) {
      // 根据帧类型设定ID
      id = ((rx_msg.id & (uint32_t)0x1FFC0000) >> (uint32_t)18);
    } else {
      id = rx_msg.id;
    }
    uint32_t dlc = rx_msg.dlc;   // 数据长度码
    uint8_t *data = rx_msg.data; // 数据载荷
    uint32_t rtr = rx_msg.rtr;   // 0=数据帧, 1=远程帧

    DecodeData(id, dlc, data, xtd, rtr);
    DL_MCAN_writeRxFIFOAck(mcan, fifo_id, rx_status.getIdx); // 确认释放
  }
}
