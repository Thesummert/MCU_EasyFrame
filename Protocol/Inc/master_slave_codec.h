#ifndef __MASTER_SLAVE_CODEC_H__
#define __MASTER_SLAVE_CODEC_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

/*主从机通信基础结构体*/
typedef struct  MS_CommProtocolBase_t{
    uint8_t idlength;  // ID长度 0为8位整数 1为16位整数
    uint8_t crc_len;   // CRC校验长度 0为8位整数 1为16位整数

    union {
        uint16_t id;
        uint8_t data[sizeof(uint16_t)];
    } id;  // ID
    union {
        uint16_t crc;
        uint8_t data[sizeof(uint16_t)];
    } crc;  // CRC

_Bool (*Encoder)(struct MS_CommProtocolBase_t *self, uint8_t *data, uint16_t len, uint8_t *out);
} MS_CommProtocolBase_t;

_Bool MS_CommProtocolBase_Init(MS_CommProtocolBase_t *self, uint8_t id_len, uint8_t crc_len);

#ifdef __cplusplus
}
#endif

#endif /* __MASTER_SLAVE_CODEC_H__ */
