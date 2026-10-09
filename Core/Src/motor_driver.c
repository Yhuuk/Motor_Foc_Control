/**
 * @file motor_driver.c
 * @brief 板级 DRV8313 功率级控制与三相 PWM 管理。
 */

#include "motor_driver.h"

#include "tim.h"

/** DRV8313 退出 nSLEEP 后，使用输出前所需的等待时间。 */
#define MOTOR_DRIVER_WAKE_DELAY_MS       (2U)

/** 复位 DRV8313 锁存故障时使用的保守复位脉冲宽度。 */
#define MOTOR_DRIVER_RESET_PULSE_MS      (1U)

/** 当前版本 PCB 上，DRV8313 OUT1 经由 IN1 连接至 TIM3 通道 3。 */
#define MOTOR_DRIVER_OUT1_CHANNEL        TIM_CHANNEL_3

/** 当前版本 PCB 上，DRV8313 OUT2 经由 IN2 连接至 TIM3 通道 2。 */
#define MOTOR_DRIVER_OUT2_CHANNEL        TIM_CHANNEL_2

/** 当前版本 PCB 上，DRV8313 OUT3 经由 IN3 连接至 TIM3 通道 1。 */
#define MOTOR_DRIVER_OUT3_CHANNEL        TIM_CHANNEL_1

/** 电机功率级的当前状态；该状态可能会被故障中断服务程序修改。 */
static volatile MotorDriver_State_t s_motor_driver_state =
    MOTOR_DRIVER_STATE_UNINITIALIZED;

/** 软件故障锁存标志；只有显式执行硬件复位并成功后才会清除。 */
static volatile bool s_motor_driver_fault_latched = false;

/**
 * @brief 将归一化占空比指令转换为 TIM3 比较值。
 *
 * @param duty 归一化占空比指令；超出 0.0f 至 1.0f 的值会被限幅。
 * @return 限制在 TIM3 当前自动重装载范围内的比较值。
 */
static uint32_t MotorDriver_DutyToCompare(float duty)
{
  uint32_t timer_period;
  uint32_t compare_value;

  if (!(duty > 0.0f))
  {
    return 0U;
  }

  timer_period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;

  if (duty >= 1.0f)
  {
    return timer_period - 1U;
  }

  compare_value = (uint32_t)(duty * (float)timer_period);
  if (compare_value >= timer_period)
  {
    compare_value = timer_period - 1U;
  }

  return compare_value;
}

/**
 * @brief 强制拉低板级半桥使能信号，并清零所有 PWM 占空比。
 *
 * 本辅助函数可安全地在 nFAULT 中断回调中调用。它不会复位 DRV8313，因此
 * 原始硬件故障仍可被观察和诊断。
 */
static void MotorDriver_ForceSafeOutputs(void)
{
  HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_RESET);

  __HAL_TIM_SET_COMPARE(&htim3, MOTOR_DRIVER_OUT1_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, MOTOR_DRIVER_OUT2_CHANNEL, 0U);
  __HAL_TIM_SET_COMPARE(&htim3, MOTOR_DRIVER_OUT3_CHANNEL, 0U);
}

/**
 * @brief 初始化 PWM 输出，并使 DRV8313 保持在安全的休眠状态。
 *
 * @return 初始化成功时返回 MOTOR_DRIVER_STATUS_OK；任一 PWM 通道无法启动时
 *         返回 MOTOR_DRIVER_STATUS_HAL_ERROR。
 */
MotorDriver_Status_t MotorDriver_Init(void)
{
  if (s_motor_driver_state != MOTOR_DRIVER_STATE_UNINITIALIZED)
  {
    return MOTOR_DRIVER_STATUS_OK;
  }

  MotorDriver_ForceSafeOutputs();
  HAL_GPIO_WritePin(DRV8313_RESET_GPIO_Port, DRV8313_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV8313_EN_GPIO_Port, DRV8313_EN_Pin, GPIO_PIN_RESET);

  if (HAL_TIM_PWM_Start(&htim3, MOTOR_DRIVER_OUT1_CHANNEL) != HAL_OK)
  {
    return MOTOR_DRIVER_STATUS_HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim3, MOTOR_DRIVER_OUT2_CHANNEL) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&htim3, MOTOR_DRIVER_OUT1_CHANNEL);
    return MOTOR_DRIVER_STATUS_HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&htim3, MOTOR_DRIVER_OUT3_CHANNEL) != HAL_OK)
  {
    (void)HAL_TIM_PWM_Stop(&htim3, MOTOR_DRIVER_OUT2_CHANNEL);
    (void)HAL_TIM_PWM_Stop(&htim3, MOTOR_DRIVER_OUT1_CHANNEL);
    return MOTOR_DRIVER_STATUS_HAL_ERROR;
  }

  s_motor_driver_fault_latched = false;
  s_motor_driver_state = MOTOR_DRIVER_STATE_SLEEP;
  return MOTOR_DRIVER_STATUS_OK;
}

/**
 * @brief 唤醒 DRV8313，同时保持所有功率输出禁止。
 *
 * @return 成功进入就绪状态时返回 MOTOR_DRIVER_STATUS_OK，否则返回状态错误
 *         或故障错误。
 */
MotorDriver_Status_t MotorDriver_Wake(void)
{
  if (s_motor_driver_state == MOTOR_DRIVER_STATE_UNINITIALIZED)
  {
    return MOTOR_DRIVER_STATUS_NOT_INITIALIZED;
  }

  if (s_motor_driver_fault_latched)
  {
    return MOTOR_DRIVER_STATUS_FAULT_ACTIVE;
  }

  if ((s_motor_driver_state == MOTOR_DRIVER_STATE_READY) ||
      (s_motor_driver_state == MOTOR_DRIVER_STATE_ENABLED))
  {
    return MOTOR_DRIVER_STATUS_OK;
  }

  MotorDriver_ForceSafeOutputs();
  HAL_GPIO_WritePin(DRV8313_RESET_GPIO_Port, DRV8313_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV8313_EN_GPIO_Port, DRV8313_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(MOTOR_DRIVER_WAKE_DELAY_MS);
  HAL_GPIO_WritePin(DRV8313_RESET_GPIO_Port, DRV8313_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(MOTOR_DRIVER_RESET_PULSE_MS);

  if (MotorDriver_IsFaultActive())
  {
    s_motor_driver_fault_latched = true;
    s_motor_driver_state = MOTOR_DRIVER_STATE_FAULT;
    return MOTOR_DRIVER_STATUS_FAULT_ACTIVE;
  }

  s_motor_driver_state = MOTOR_DRIVER_STATE_READY;
  return MOTOR_DRIVER_STATUS_OK;
}

/**
 * @brief 使能 DRV8313 的全部三个半桥通道。
 *
 * @return 使能成功时返回 MOTOR_DRIVER_STATUS_OK，否则返回状态错误或故障错误。
 */
MotorDriver_Status_t MotorDriver_EnableOutputs(void)
{
  if (s_motor_driver_state == MOTOR_DRIVER_STATE_UNINITIALIZED)
  {
    return MOTOR_DRIVER_STATUS_NOT_INITIALIZED;
  }

  if (s_motor_driver_state == MOTOR_DRIVER_STATE_ENABLED)
  {
    return MOTOR_DRIVER_STATUS_OK;
  }

  if (s_motor_driver_state == MOTOR_DRIVER_STATE_FAULT)
  {
    return MOTOR_DRIVER_STATUS_FAULT_ACTIVE;
  }

  if (s_motor_driver_state != MOTOR_DRIVER_STATE_READY)
  {
    return MOTOR_DRIVER_STATUS_INVALID_STATE;
  }

  if (s_motor_driver_fault_latched || MotorDriver_IsFaultActive())
  {
    MotorDriver_ForceSafeOutputs();
    s_motor_driver_fault_latched = true;
    s_motor_driver_state = MOTOR_DRIVER_STATE_FAULT;
    return MOTOR_DRIVER_STATUS_FAULT_ACTIVE;
  }

  HAL_GPIO_WritePin(M_EN_GPIO_Port, M_EN_Pin, GPIO_PIN_SET);
  s_motor_driver_state = MOTOR_DRIVER_STATE_ENABLED;
  return MOTOR_DRIVER_STATUS_OK;
}

/**
 * @brief 禁止全部三个半桥，并清零当前 PWM 指令。
 */
void MotorDriver_DisableOutputs(void)
{
  if (s_motor_driver_state == MOTOR_DRIVER_STATE_UNINITIALIZED)
  {
    return;
  }

  MotorDriver_ForceSafeOutputs();

  if (s_motor_driver_state == MOTOR_DRIVER_STATE_ENABLED)
  {
    s_motor_driver_state = MOTOR_DRIVER_STATE_READY;
  }
}

/**
 * @brief 先禁止所有半桥，然后使 DRV8313 进入休眠状态。
 */
void MotorDriver_EnterSleep(void)
{
  if (s_motor_driver_state == MOTOR_DRIVER_STATE_UNINITIALIZED)
  {
    return;
  }

  MotorDriver_ForceSafeOutputs();
  HAL_GPIO_WritePin(DRV8313_RESET_GPIO_Port, DRV8313_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV8313_EN_GPIO_Port, DRV8313_EN_Pin, GPIO_PIN_RESET);

  if (!s_motor_driver_fault_latched)
  {
    s_motor_driver_state = MOTOR_DRIVER_STATE_SLEEP;
  }
}

/**
 * @brief 复位 DRV8313 故障，同时保持功率输出禁止。
 *
 * @return 复位成功时返回 MOTOR_DRIVER_STATUS_OK，否则返回对应错误状态。
 */
MotorDriver_Status_t MotorDriver_ClearFault(void)
{
  if (s_motor_driver_state == MOTOR_DRIVER_STATE_UNINITIALIZED)
  {
    return MOTOR_DRIVER_STATUS_NOT_INITIALIZED;
  }

  MotorDriver_ForceSafeOutputs();
  HAL_GPIO_WritePin(DRV8313_RESET_GPIO_Port, DRV8313_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV8313_EN_GPIO_Port, DRV8313_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(MOTOR_DRIVER_WAKE_DELAY_MS);
  HAL_GPIO_WritePin(DRV8313_RESET_GPIO_Port, DRV8313_RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(MOTOR_DRIVER_RESET_PULSE_MS);

  if (MotorDriver_IsFaultActive())
  {
    s_motor_driver_fault_latched = true;
    s_motor_driver_state = MOTOR_DRIVER_STATE_FAULT;
    return MOTOR_DRIVER_STATUS_FAULT_ACTIVE;
  }

  s_motor_driver_fault_latched = false;
  s_motor_driver_state = MOTOR_DRIVER_STATE_READY;
  return MOTOR_DRIVER_STATUS_OK;
}

/**
 * @brief 设置 DRV8313 三路输出的归一化 PWM 占空比。
 *
 * @param output1_duty OUT1 的归一化占空比。
 * @param output2_duty OUT2 的归一化占空比。
 * @param output3_duty OUT3 的归一化占空比。
 * @return 比较寄存器更新成功时返回 MOTOR_DRIVER_STATUS_OK。
 */
MotorDriver_Status_t MotorDriver_SetDutyCycles(float output1_duty,
                                               float output2_duty,
                                               float output3_duty)
{
  if (s_motor_driver_state == MOTOR_DRIVER_STATE_UNINITIALIZED)
  {
    return MOTOR_DRIVER_STATUS_NOT_INITIALIZED;
  }

  __HAL_TIM_SET_COMPARE(&htim3,
                        MOTOR_DRIVER_OUT1_CHANNEL,
                        MotorDriver_DutyToCompare(output1_duty));
  __HAL_TIM_SET_COMPARE(&htim3,
                        MOTOR_DRIVER_OUT2_CHANNEL,
                        MotorDriver_DutyToCompare(output2_duty));
  __HAL_TIM_SET_COMPARE(&htim3,
                        MOTOR_DRIVER_OUT3_CHANNEL,
                        MotorDriver_DutyToCompare(output3_duty));

  return MOTOR_DRIVER_STATUS_OK;
}

/**
 * @brief 锁存硬件故障，并立即禁止三个半桥。
 */
void MotorDriver_HandleFaultInterrupt(void)
{
  if (!MotorDriver_IsFaultActive())
  {
    return;
  }

  MotorDriver_ForceSafeOutputs();
  s_motor_driver_fault_latched = true;
  s_motor_driver_state = MOTOR_DRIVER_STATE_FAULT;
}

/**
 * @brief 读取低电平有效的 DRV8313 nFAULT 输入。
 *
 * @return nFAULT 为低电平时返回 true，否则返回 false。
 */
bool MotorDriver_IsFaultActive(void)
{
  return HAL_GPIO_ReadPin(DRV8313_FAULT_GPIO_Port,
                          DRV8313_FAULT_Pin) == GPIO_PIN_RESET;
}

/**
 * @brief 读取软件保留的电机驱动故障状态。
 *
 * @return 故障已锁存时返回 true，否则返回 false。
 */
bool MotorDriver_IsFaultLatched(void)
{
  return s_motor_driver_fault_latched;
}

/**
 * @brief 返回电机驱动模块的当前状态。
 *
 * @return 当前的 MotorDriver_State_t 状态值。
 */
MotorDriver_State_t MotorDriver_GetState(void)
{
  return s_motor_driver_state;
}
