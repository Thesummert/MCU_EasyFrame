#include "qei_encoder.h"
#include "bsp_time.h"
#include "dsp/fast_math_functions.h"
#include "mcu_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/reent.h>


static _Bool getSpeed(EF_QEI_Encoder_t *self, float dt, float *omega_rotor, float *omega_output);

/**
 * @brief       初始化QEI编码器设备
 * @param[in]   self        编码器对象指针
 * @param[in]   qei         QEI定时器基类指针(需先初始化)
 * @param[in]   ppr         编码器每转脉冲数(PPR)
 * @param[in]   multiplier  倍频数(通常为4)
 * @param[in]   radio       减速比(电机轴/输出轴)
 * @retval      true        初始化成功
 * @retval      false       初始化失败(空指针)
 */
_Bool EF_Device_QEI_Encoder_Init(EF_QEI_Encoder_t *self, EF_BSP_TimerQEI_t *qei,
                                 uint32_t ppr, uint8_t multiplier, float radio)
{
    if (self == NULL || qei == NULL) {
        RTT_Print(0, "Null pointer error in qei encoder init \r\n");
        return false;
    }

    memset(self, 0, sizeof(EF_QEI_Encoder_t));
    self->multiplier = multiplier;
    self->radio = radio;
    self->pulse_per_rev = ppr;
    self->qei = qei;
    self->inv_pulse_per_rev = 1.0 / ppr;
    self->inv_multiplier = 1.0f / multiplier;
    self->inv_radio = 1.0f / radio;

    self->getSpeed = getSpeed;

    self->is_inited = true;

    return true;
}


/**
 * @brief       计算编码器转速(电机轴角速度和输出轴角速度)
 * @param[in]   self          编码器对象指针
 * @param[in]   dt            采样时间间隔(秒)
 * @param[out]  omega_rotor   电机轴角速度(rad/s, 可选传NULL)
 * @param[out]  omega_output  输出轴角速度(rad/s, 可选传NULL)
 * @retval      true          计算成功
 * @retval      false         计算失败(空指针、未初始化或dt=0)
 */
static _Bool getSpeed(EF_QEI_Encoder_t *self, float dt, float *omega_rotor, float *omega_output)
{
    if (self == NULL) {
        RTT_Print(0, "Null pointer error in qei get speed \r\n");
        return false;
    }
    if (self->is_inited == false) {
        return false;
    }
    if (dt == 0.0f) {
        return false;
    }
    EF_BSP_TimerQEI_t *qei = self->qei;

    qei->getDelta(qei, &self->delta);
    self->omega_rotor = (float)self->delta * self->inv_multiplier * self->inv_pulse_per_rev * 2 * PI / dt;
    self->omega_output = self->omega_rotor * self->inv_radio;

    if (omega_output != NULL) {
        *omega_output = self->omega_output;
    }
    if (omega_rotor != NULL) {
        *omega_rotor = self->omega_rotor;
    }

    return true;
}
