#include "master_slave_codec.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "mcu_config.h"
#include "crc_ref.h"

static _Bool Encoder(MS_CommProtocolBase_t *self, uint8_t *data, uint16_t len, uint8_t *out);

/**
 * @brief 初始化主从通信协议基础对象
 *
 * @param[in] self  协议对象指针，不可为NULL
 * @param[in] id_len ID长度（字节数），决定ID字段占用字节数
 * @param[in] crc_len CRC长度（字节数），决定校验类型，0表示CRC8，其他值表示CRC16
 * @return _Bool 初始化成功返回true，失败（self为NULL）返回false
 *
 * @note 该函数会清空协议对象的所有字段，然后设置ID长度和CRC长度，并注册编码器函数。
 */
_Bool MS_CommProtocolBase_Init(MS_CommProtocolBase_t *self, uint8_t id_len, uint8_t crc_len) {
    if (self == NULL) {
        RTT_Print(0, "Null pointer error in MS_CommBase Init \r\n");
        return false;
    }
    memset(self, 0, sizeof(MS_CommProtocolBase_t));
    self->idlength = id_len;
    self->crc_len = crc_len;

    self->Encoder = Encoder;

    return true;
}

/**
 * @brief 编码数据帧：添加ID前缀和CRC校验
 *
 * @param[in] self 协议对象指针，不可为NULL
 * @param[in] data 待编码的原始数据缓冲区
 * @param[in] len  原始数据长度（字节数）
 * @param[out] out 编码后的输出缓冲区，长度需 >= len + id_len + crc_len
 * @return _Bool 编码成功返回true，参数为NULL时返回false
 *
 * @note 该函数不会检验输出缓冲区是否溢出，调用方需确保输出缓冲区足够大。
 *       编码格式：[ID字段(id_len字节)] + [原始数据] + [CRC校验(crc_len字节)]
 *       - id_len == 0 时，ID字段仅占用 self->id.data[1] 一个字节
 *       - id_len != 0 时，ID字段占用 self->id.data 的两个字节
 *       - crc_len == 0 时追加CRC8校验，否则追加CRC16校验
 */
static _Bool Encoder(MS_CommProtocolBase_t *self, uint8_t *data, uint16_t len, uint8_t *out) {
    /*此函数不会检验溢出*/
    if (self == NULL || data == NULL || out == NULL) {
        RTT_Print(0, "Null pointer error in MS_CommBase encode \r\n");
        return false;
    }
    memset(out, 0, len + self->idlength + self->crc_len);
    if (self->idlength == 0) {
        out[0] = self->id.data[1];
    } else {
        memcpy(out, self->id.data, 2);
    }
    memcpy(out + self->idlength, data, len);  // 将数据拷贝到发送区中
    // 添加CRC
    if (self->crc_len == 0) {
        Append_CRC8_Check_Sum(out, self->idlength + self->crc_len + len);
    } else {
        Append_CRC16_Check_Sum(out, self->idlength + self->crc_len + len);
    }

    return true;
}
