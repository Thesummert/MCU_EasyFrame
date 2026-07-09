#include "bsp_time.h"
#include "mcu_config.h"
#include "ti/driverlib/dl_timer.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/reent.h>

static _Bool StartCounter(EF_BSP_TimerBase_t *self);
static _Bool StopCounter(EF_BSP_TimerBase_t *self);
static _Bool setCounterMode(EF_BSP_TimerBase_t *self,
                            EF_TimerCounterMode_e mode);
static _Bool setCounterPrescaler(EF_BSP_TimerBase_t *self, uint32_t prescaler);
static _Bool getFreq(EF_BSP_TimerBase_t *self, float *freq);

static _Bool SetPWM_Float(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                          float radio);
static _Bool SetPWM(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                    uint32_t compare);
static _Bool SetPWM_Mode(EF_BSP_TimerPWM_t *self, EF_TimerPWMMode_e mode);
static _Bool StartPWM(EF_BSP_TimerPWM_t *self);
static _Bool StopPWM(EF_BSP_TimerPWM_t *self);

static _Bool setBaseLoad(EF_BSP_TimerQEI_t *self, uint32_t load);
static _Bool getDelta(EF_BSP_TimerQEI_t *self, int32_t *delta);

/**
 * @brief       初始化定时器基类
 * @param[in]   self              定时器对象指针
 * @param[in]   tim               定时器寄存器基地址
 * @param[in]   clock_source_freq 时钟源频率(Hz)
 * @param[in]   max_prescaler     最大预分频系数
 * @param[in]   max_auto_reload_num 最大自动重装载值
 * @retval      true              初始化成功
 * @retval      false             初始化失败(空指针)
 */
_Bool EF_BSP_TimerBase_Init(EF_BSP_TimerBase_t *self, GPTIMER_Regs *tim,
                            uint32_t clock_source_freq, uint32_t max_prescaler,
                            uint32_t max_auto_reload_num) {
  if (self == NULL || tim == NULL) {
    RTT_Print(0, "Null pointer error in tim init \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_BSP_TimerBase_t));
  self->clock_source_freq = clock_source_freq;
  self->mspm0g.tim = tim;
  self->max_auto_reload_num = max_auto_reload_num;
  self->max_prescaler = max_prescaler;
  self->max_freq = clock_source_freq;

  self->StartCounter = StartCounter;
  self->StopCounter = StopCounter;
  self->setCounterMode = setCounterMode;
  self->setCounterPrescaler = setCounterPrescaler;
  self->getFreq = getFreq;

  self->is_inited = true;

  return true;
}

/**
 * @brief       启动定时器计数
 * @param[in]   self  定时器对象指针
 * @retval      true  启动成功
 * @retval      false 启动失败(空指针或未初始化)
 */
static _Bool StartCounter(EF_BSP_TimerBase_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  DL_Timer_startCounter(self->mspm0g.tim);

  return true;
}

/**
 * @brief       停止定时器计数
 * @param[in]   self  定时器对象指针
 * @retval      true  停止成功
 * @retval      false 停止失败(空指针或未初始化)
 */
static _Bool StopCounter(EF_BSP_TimerBase_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim stop \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  DL_Timer_stopCounter(self->mspm0g.tim);

  return true;
}

/**
 * @brief       设定定时器计数模式
 * @param[in]   self  定时器对象指针
 * @param[in]   mode  计数模式(单次/周期/上下等)
 * @retval      true  设定成功
 * @retval      false 设定失败(空指针或未初始化)
 */
static _Bool setCounterMode(EF_BSP_TimerBase_t *self,
                            EF_TimerCounterMode_e mode) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->counter_mode = mode;
  // 根据架构设定参数设定实际参数
  switch (self->counter_mode) {
  case TIMER_COUNT_MODE_ONCE_UP:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_ONE_SHOT_UP;
    break;
  case TIMER_COUNT_MODE_ONCE_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_ONE_SHOT;
    break;
  case TIMER_COUNT_MODE_ONCE_UP_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_ONE_SHOT_UP_DOWN;
    break;
  case TIMER_COUNT_MODE_PERIODIC_UP:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC_UP;
    break;
  case TIMER_COUNT_MODE_PERIODIC_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC;
    break;
  case TIMER_COUNT_MODE_PERIODIC_UP_DOWN:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC_UP_DOWN;
    break;
  default:
    self->mspm0g.mode = DL_TIMER_TIMER_MODE_PERIODIC_UP;
    break;
  }

  DL_TimerG_TimerConfig config = {
      .period = 0,
      .timerMode = self->mspm0g.mode,
      .startTimer = DL_TIMER_STOP,
  };
  DL_Timer_initTimerMode(self->mspm0g.tim, &config);

  return true;
}

/**
 * @brief       设置定时器预分频系数
 * @param[in]   self       定时器对象指针
 * @param[in]   prescaler  预分频系数(超过最大值自动钳位)
 * @retval      true       设置成功
 * @retval      false      设置失败(空指针或未初始化)
 */
static _Bool setCounterPrescaler(EF_BSP_TimerBase_t *self, uint32_t prescaler) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  if (prescaler > self->max_prescaler) {
    prescaler = self->max_prescaler;
  }
  self->prescaler = prescaler;
  DL_Timer_ClockConfig config;
  // 先获取原有参数
  DL_Timer_getClockConfig(self->mspm0g.tim, &config);
  config.prescale = prescaler;
  DL_Timer_setClockConfig(self->mspm0g.tim, &config);

  return true;
}

/**
 * @brief 获取定时器的实际频率
 *
 * 该函数用于计算并返回定时器的实际工作频率。
 *
 * @param self 指向定时器基类结构体的指针，不能为空。
 * @param freq 输出参数，用于存放计算得到的频率值，可以为NULL。
 * @retval true  获取频率成功。
 * @retval false 获取频率失败（如self为NULL或未初始化）。
 */

static _Bool getFreq(EF_BSP_TimerBase_t *self, float *freq) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in tim start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->freq = self->max_freq / self->prescaler / self->auto_reload_num;

  if (freq != NULL) {
    *freq = self->freq;
  }

  return true;
}

/*===========PWM===========*/
/**
 * @brief       初始化PWM控制器
 * @param[in]   self         PWM对象指针
 * @param[in]   etim         定时器基类对象指针(需先初始化)
 * @param[in]   max_channel  最大通道数
 * @retval      true         初始化成功
 * @retval      false        初始化失败(空指针或定时器未初始化)
 */
_Bool EF_BSP_TimerPWM_Init(EF_BSP_TimerPWM_t *self, EF_BSP_TimerBase_t *etim,
                           uint8_t max_channel) {
  if (self == NULL || etim == NULL) {
    RTT_Print(0, "Null pointer error in timer pwm init \r\n");
    return false;
  }
  if (etim->is_inited == false) {
    RTT_Print(0, "Need to init tim first then init pwm \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_BSP_TimerPWM_t));
  self->max_channel = max_channel;
  self->etim = etim;

  // MSPM0需要判断是否支持4channel
  if (max_channel > 2) {
    self->mspm0g.is_support4channel = true;
  }

  // 初始化函数指针
  self->SetPWM_Float = SetPWM_Float;
  self->SetPWM = SetPWM;
  self->StartPWM = StartPWM;
  self->StopPWM = StopPWM;
  self->SetPWM_Mode = SetPWM_Mode;

  self->is_inited = true;

  return true;
}

/**
 * @brief       设置PWM通道比较值(整数方式)
 * @param[in]   self        PWM对象指针
 * @param[in]   channel_id  通道ID(从0开始)
 * @param[in]   compare     比较值(占空比 = compare / max_auto_reload_num)
 * @retval      true        设置成功
 * @retval      false       设置失败(空指针或未初始化)
 */
static _Bool SetPWM(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                    uint32_t compare) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm set \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  // 超出最大值则设定为最大值
  if (compare > self->etim->mspm0g.tim->COUNTERREGS.LOAD) {
    compare = self->etim->mspm0g.tim->COUNTERREGS.LOAD;
  }
  // 对于此设定方式 不设置浮点设定值 减少性能开销
  self->channels[channel_id] = compare;

  DL_Timer_setCaptureCompareValue(self->etim->mspm0g.tim, compare, channel_id);
  return true;
}

/**
 * @brief       设置PWM通道占空比(浮点方式)
 * @param[in]   self        PWM对象指针
 * @param[in]   channel_id  通道ID(从0开始)
 * @param[in]   radio       占空比(0.0 ~ 1.0, 超出范围自动钳位)
 * @retval      true        设置成功
 * @retval      false       设置失败(空指针或未初始化)
 */
static _Bool SetPWM_Float(EF_BSP_TimerPWM_t *self, uint8_t channel_id,
                          float radio) {

  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm set \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  if (radio > 1.0f) {
    radio = 1.0f;
  } else if (radio < 0.0f) {
    radio = 0.0f;
  }
  self->radio[channel_id] = radio;
  self->channels[channel_id] = self->etim->mspm0g.tim->COUNTERREGS.LOAD * radio;
  DL_Timer_setCaptureCompareValue(self->etim->mspm0g.tim,
                                  self->channels[channel_id], channel_id);

  return true;
}

/**
 * @brief       启动PWM输出
 * @param[in]   self  PWM对象指针
 * @retval      true  启动成功
 * @retval      false 启动失败(空指针或未初始化)
 */
static _Bool StartPWM(EF_BSP_TimerPWM_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm start \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->etim->StartCounter(self->etim);

  return true;
}

/**
 * @brief       停止PWM输出
 * @param[in]   self  PWM对象指针
 * @retval      true  停止成功
 * @retval      false 停止失败(空指针或未初始化)
 */
static _Bool StopPWM(EF_BSP_TimerPWM_t *self) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm stop \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  self->etim->StopCounter(self->etim);

  return true;
}

/**
 * @brief       设置PWM模式(边缘对齐/中心对齐)
 * @param[in]   self  PWM对象指针
 * @param[in]   mode  PWM模式(向上计数/向下计数/中心对齐)
 * @retval      true  设置成功
 * @retval      false 设置失败(空指针、未初始化或未知模式)
 */
static _Bool SetPWM_Mode(EF_BSP_TimerPWM_t *self, EF_TimerPWMMode_e mode) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in pwm set mode \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  // 按照ef参数设定实际单片机控制参数
  switch (mode) {
  case TIMER_PWM_MODE_UP:
    self->mode = TIMER_PWM_MODE_UP;
    self->mspm0g.pwm_mode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP;
    break;
  case TIMER_PWM_MODE_DOWN:
    self->mode = TIMER_PWM_MODE_DOWN;
    self->mspm0g.pwm_mode = DL_TIMER_PWM_MODE_EDGE_ALIGN;
    break;
  case TIMER_PWM_MODE_CENTER:
    self->mode = TIMER_PWM_MODE_CENTER;
    self->mspm0g.pwm_mode = DL_TIMER_PWM_MODE_CENTER_ALIGN;
    break;
  default:
    return false;
  }
  const DL_TimerA_PWMConfig config = {
      .period = 0,
      .startTimer = DL_TIMER_STOP,
      .pwmMode = self->mspm0g.pwm_mode,
      .isTimerWithFourCC = self->mspm0g.is_support4channel,
  };
  // 设定PWM
  DL_Timer_initPWMMode(self->etim->mspm0g.tim, &config);

  return true;
}

/*===========QEI===========*/
/**
 * @brief       初始化QEI编码器接口
 * @param[in]   self  QEI对象指针
 * @param[in]   etim  定时器基类对象指针(需先初始化并配置为QEI模式)
 * @retval      true  初始化成功
 * @retval      false 初始化失败(空指针或定时器未初始化)
 */
_Bool EF_BSP_TimerQEI_Init(EF_BSP_TimerQEI_t *self, EF_BSP_TimerBase_t *etim) {
  if (self == NULL || etim == NULL) {
    RTT_Print(0, "Null pointer error in timer qei init \r\n");
    return false;
  }
  if (etim->is_inited == false) {
    RTT_Print(0, "Need to init tim first then init qei \r\n");
    return false;
  }
  memset(self, 0, sizeof(EF_BSP_TimerQEI_t));
  self->etim = etim;
  // 获取默认装载值
  self->base_load = DL_Timer_getLoadValue(self->etim->mspm0g.tim);

  // 初始化函数指针
  self->setBaseLoad = setBaseLoad;
  self->getDelta = getDelta;

  self->is_inited = true;

  return true;
}

/**
 * @brief       设置QEI编码器LOAD基准值
 * @param[in]   self  QEI对象指针
 * @param[in]   load  LOAD值(超过最大值自动钳位)
 * @retval      true  设置成功
 * @retval      false 设置失败(空指针或未初始化)
 */
static _Bool setBaseLoad(EF_BSP_TimerQEI_t *self, uint32_t load) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in qei set load \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  if (load > self->etim->max_auto_reload_num) {
    load = self->etim->max_auto_reload_num;
  }
  DL_Timer_setLoadValue(self->etim->mspm0g.tim, load);

  return true;
  ;
}

/**
 * @brief       获取QEI编码器增量值(基准值与当前LOAD的差值)
 * @param[in]   self   QEI对象指针
 * @param[out]  delta  增量值输出指针
 * @retval      true   获取成功
 * @retval      false  获取失败(空指针或未初始化)
 */
static _Bool getDelta(EF_BSP_TimerQEI_t *self, int32_t *delta) {
  if (self == NULL) {
    RTT_Print(0, "Null pointer error in qei set load \r\n");
    return false;
  }
  if (self->is_inited == false) {
    return false;
  }
  *delta = self->base_load - DL_Timer_getLoadValue(self->etim->mspm0g.tim);
  DL_Timer_setLoadValue(self->etim->mspm0g.tim, self->base_load);
  return true;
}
