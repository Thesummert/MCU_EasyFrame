#include "brushed_motor.h"
#include "SEGGER_RTT.h"
#include "mcu_config.h"
#include "ti/driverlib/dl_aes.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

static _Bool SetOutput(EF_BrushedMotor_t *self, float value);
static _Bool Break(EF_BrushedMotor_t *self);
static _Bool Start(EF_BrushedMotor_t *self);
static _Bool Stop(EF_BrushedMotor_t *self);
static _Bool GetSpeed(EF_BrushedMotor_t *self, float dt, float *omega_rotor,
                      float *omega_output);

_Bool EF_BrushedMotor_Init(EF_BrushedMotor_t *self, EF_BrushedMotorType_e type,
                           _Bool need_enable, EF_BSP_TimerPWM_t *pwm) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_BrushedMotor_t));
  self->motor_type = type;
  self->pwm = pwm;
  self->need_enable = need_enable;

  // 初始化函数指针
  self->SetOutput = SetOutput;
  self->Break = Break;
  self->Start = Start;
  self->Stop = Stop;
  self->GetSpeed = GetSpeed;
  return true;
}

_Bool EF_BrushedMotor_InitQEI(EF_BrushedMotor_t *self, EF_QEI_Encoder_t *qei) {
  if (self == NULL || qei == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init qei \r\n");
    return false;
  }
  self->qei = qei;
  return true;
}

_Bool EF_BrushedMotor_InitEnable(EF_BrushedMotor_t *self,
                                 EasyFrame_GPIO_Typedef_t *gpio) {
  if (self == NULL || gpio == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init enable \r\n");
    return false;
  }
  self->enable = gpio;
  return true;
}

_Bool EF_BrushedMotor_InitDir(EF_BrushedMotor_t *self,
                              EasyFrame_GPIO_Typedef_t *dir1,
                              EasyFrame_GPIO_Typedef_t *dir2) {
  if (self == NULL || dir1 == NULL || dir2 == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor init dir \r\n");
    return false;
  }
  self->dir1 = dir1;
  self->dir2 = dir2;
  return true;
}

static _Bool SetOutput(EF_BrushedMotor_t *self, float value) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor set value \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  switch (self->motor_type) {
  case BRUSHED_MOTOR_TYPE1:
    // 需要判断方向设定数值
    if (value >= 0.0f) {
      self->pwm->SetPWM(self->pwm, 0, value);
      self->pwm->SetPWM(self->pwm, 1, 0);
    } else {
      self->pwm->SetPWM(self->pwm, 0, 0);
      self->pwm->SetPWM(self->pwm, 1, -value);
    }
    break;
  case BRUSHED_MOTOR_TYPE2:
    break;
  default:
    break;
  }

  return true;
}

static _Bool Break(EF_BrushedMotor_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor set break \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  switch (self->motor_type) {
  case BRUSHED_MOTOR_TYPE1:
    // 都是高电平的时候是刹车
    self->pwm->SetPWM(self->pwm, 0, 0xFFFFFFFF);
    self->pwm->SetPWM(self->pwm, 1, 0xFFFFFFFF);
    break;
  case BRUSHED_MOTOR_TYPE2:
    break;
  default:
    break;
  }

  return true;
}

static _Bool Start(EF_BrushedMotor_t *self) {

  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor start \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  self->pwm->StartPWM(self->pwm);

  return true;
}

static _Bool Stop(EF_BrushedMotor_t *self) {

  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor stop \r\n");
    return false;
  }
  if (self->pwm == NULL) {
    return false;
  }
  self->pwm->StopPWM(self->pwm);

  return true;
}

static _Bool GetSpeed(EF_BrushedMotor_t *self, float dt, float *omega_rotor,
                      float *omega_output) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in brushed motor get speed \r\n");
    return false;
  }
  if (self->qei == NULL) {
    return false;
  }
  if (!self->qei->getSpeed(self->qei, dt, &self->omega_rotor,
                           &self->omega_output)) {
    return false;
  }

  if (omega_rotor != NULL) {
    *omega_rotor = self->omega_rotor;
  }
  if (omega_output != NULL) {
    *omega_output = self->omega_output;
  }

  return true;
}
