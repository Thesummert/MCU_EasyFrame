#ifndef __BRUSHED_MOTOR_H__
#define __BRUSHED_MOTOR_H__

#include "bsp_mspm0g_gpio.h"
#include "bsp_time.h"
#include "qei_encoder.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  BRUSHED_MOTOR_TYPE1, // 类似DRV8870 这种 只需要两个PWM输入来控制方向和速度
  BRUSHED_MOTOR_TYPE2, // 类似tb6612 需要使用单独的GPIO来控制方向 速度
} EF_BrushedMotorType_e;

typedef struct EF_BrushedMotor_t {
  EF_BrushedMotorType_e motor_type;      // 电机种类
  EF_QEI_Encoder_t *qei;                 // QEI编码器
  EF_BSP_TimerPWM_t *pwm;                // PWM
  EasyFrame_GPIO_Typedef_t *enable;      // 使能端口
  EasyFrame_GPIO_Typedef_t *dir1, *dir2; // 方向控制端口
  _Bool need_enable;                     // 是否需要使能功能
  float set_value;                       // 设定值
  _Bool break_flag;                      // 刹车标志
  float omega_rotor;                     // 转子角速度
  float omega_output;                    // 输出轴角速度

  _Bool (*SetOutput)(struct EF_BrushedMotor_t *self, float value);
  _Bool (*Break)(struct EF_BrushedMotor_t *self);
  _Bool (*Start)(struct EF_BrushedMotor_t *self);
  _Bool (*Stop)(struct EF_BrushedMotor_t *self);
  _Bool (*GetSpeed)(struct EF_BrushedMotor_t *self, float dt,
                    float *omega_rotor, float *omega_output);

} EF_BrushedMotor_t;

_Bool EF_BrushedMotor_Init(EF_BrushedMotor_t *self, EF_BrushedMotorType_e type,
                           _Bool need_enable, EF_BSP_TimerPWM_t *pwm);
_Bool EF_BrushedMotor_InitQEI(EF_BrushedMotor_t *self, EF_QEI_Encoder_t *qei);
_Bool EF_BrushedMotor_InitEnable(EF_BrushedMotor_t *self,
                                 EasyFrame_GPIO_Typedef_t *gpio);
_Bool EF_BrushedMotor_InitDir(EF_BrushedMotor_t *self,
                              EasyFrame_GPIO_Typedef_t *dir1,
                              EasyFrame_GPIO_Typedef_t *dir2);

#ifdef __cplusplus
}
#endif

#endif /* __BRUSHED_MOTOR_H__ */
