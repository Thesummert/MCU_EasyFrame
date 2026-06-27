#ifndef __MCU_CONFIG_H__
#define __MCU_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define BUS_CLK 80 // 主线时钟

// 是否使用RTT
#define USING_SEGGER_RTT 1

// 配合宏开关可快速启用rtt打印 关闭时几乎不消耗资源 开启优化下应该会优化掉此函数
#if USING_SEGGER_RTT == 1
#include "SEGGER_RTT.h"
#define RTT_Print(X, ...) SEGGER_RTT_printf(X, ##__VA_ARGS__)
#else
#define RTT_Print(X, ...)
#endif

// 一个中断组合最大容量
#define IT_GROUP_ITEM_MAX_SIZE 16

// SPI BUFFER大小
#define SPI_BUFFER_SIZE 64

// I2C FIFO BUFFER大小
#define I2C_FIFO_MAX_SIZE 8

// 陀螺仪是否使用IQMATH数学库
#define USING_IQMATH_LSM 1

#if USING_IQMATH_LSM == 1
#define USING_IQMATH_IMU 1
#else
#define USING_IQMATH_IMU 0
#endif

#define UART_ENABLE_FIFO 1
#define UART_ENABLE_DMA  1
#define UART_ENABLE_IT   1

#if UART_ENABLE_FIFO == 1
#define UART_TX_FIFO_MAX_SIZE 4
#define UART_RX_FIFO_MAX_SIZE 4
#endif

#if UART_ENABLE_DMA == 1
#define UART_TX_DMA_BUFFER_SIZE 64
#define UART_RX_DMA_BUFFER_SIZE 64
#endif

#if UART_ENABLE_IT == 1
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MCU_CONFIG_H__ */
