#ifndef FOC_DRIVE_MOTOR_DRIVER_H
#define FOC_DRIVE_MOTOR_DRIVER_H

#include <stdbool.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机驱动模块的返回状态码。
 */
typedef enum
{
  MOTOR_DRIVER_STATUS_OK = 0,
  MOTOR_DRIVER_STATUS_NOT_INITIALIZED,
  MOTOR_DRIVER_STATUS_INVALID_STATE,
  MOTOR_DRIVER_STATUS_FAULT_ACTIVE,
  MOTOR_DRIVER_STATUS_HAL_ERROR
} MotorDriver_Status_t;

/**
 * @brief DRV8313 电机功率级的运行状态。
 */
typedef enum
{
  MOTOR_DRIVER_STATE_UNINITIALIZED = 0,
  MOTOR_DRIVER_STATE_SLEEP,
  MOTOR_DRIVER_STATE_READY,
  MOTOR_DRIVER_STATE_ENABLED,
  MOTOR_DRIVER_STATE_FAULT
} MotorDriver_State_t;

/**
 * @brief 初始化 PWM 输出，并使 DRV8313 保持在安全的休眠状态。
 *
 * @return 初始化成功时返回 MOTOR_DRIVER_STATUS_OK，否则返回对应的错误状态。
 */
MotorDriver_Status_t MotorDriver_Init(void);

/**
 * @brief 唤醒 DRV8313，同时保持三个半桥均处于禁止状态。
 *
 * @return 驱动器进入就绪状态时返回 MOTOR_DRIVER_STATUS_OK；如果 nFAULT
 *         持续有效，则返回 MOTOR_DRIVER_STATUS_FAULT_ACTIVE。
 */
MotorDriver_Status_t MotorDriver_Wake(void);

/**
 * @brief 使能 DRV8313 的三个半桥输出。
 *
 * 调用本函数前，驱动器必须已唤醒、无故障并处于就绪状态。
 *
 * @return 输出成功使能时返回 MOTOR_DRIVER_STATUS_OK，否则返回拒绝本次状态
 *         转换的原因。
 */
MotorDriver_Status_t MotorDriver_EnableOutputs(void);

/**
 * @brief 禁止全部三个半桥，并将当前 PWM 占空比指令清零。
 */
void MotorDriver_DisableOutputs(void);

/**
 * @brief 禁止输出，并使 DRV8313 进入低功耗休眠状态。
 */
void MotorDriver_EnterSleep(void);

/**
 * @brief 复位 DRV8313 的故障锁存，同时保持输出禁止。
 *
 * @return 复位后 nFAULT 释放时返回 MOTOR_DRIVER_STATUS_OK；如果硬件故障
 *         仍然存在，则返回 MOTOR_DRIVER_STATUS_FAULT_ACTIVE。
 */
MotorDriver_Status_t MotorDriver_ClearFault(void);

/**
 * @brief 设置 DRV8313 OUT1、OUT2 和 OUT3 的归一化 PWM 占空比。
 *
 * 每个占空比都会被限制在 0.0f 至 1.0f 范围内。本函数采用当前版本电路板
 * 的布线关系：OUT1 对应 TIM3_CH3，OUT2 对应 TIM3_CH2，OUT3 对应
 * TIM3_CH1。
 *
 * @param output1_duty DRV8313 OUT1 的归一化占空比指令。
 * @param output2_duty DRV8313 OUT2 的归一化占空比指令。
 * @param output3_duty DRV8313 OUT3 的归一化占空比指令。
 * @return 比较寄存器更新成功时返回 MOTOR_DRIVER_STATUS_OK；如果模块尚未
 *         初始化，则返回 MOTOR_DRIVER_STATUS_NOT_INITIALIZED。
 */
MotorDriver_Status_t MotorDriver_SetDutyCycles(float output1_duty,
                                               float output2_duty,
                                               float output3_duty);

/**
 * @brief 立即使电机驱动模块进入故障状态。
 *
 * 本函数用于在 nFAULT 下降沿 GPIO 中断回调中调用。它会禁止所有半桥并在
 * 软件中锁存故障，但不会复位 DRV8313 的硬件故障源。
 */
void MotorDriver_HandleFaultInterrupt(void);

/**
 * @brief 读取 DRV8313 的 nFAULT 引脚当前是否处于低电平有效状态。
 *
 * @return 硬件故障信号有效时返回 true，否则返回 false。
 */
bool MotorDriver_IsFaultActive(void);

/**
 * @brief 读取软件故障锁存状态。
 *
 * @return 检测到故障后返回 true，直到 MotorDriver_ClearFault 成功清除故障。
 */
bool MotorDriver_IsFaultLatched(void);

/**
 * @brief 读取电机驱动模块的当前状态。
 *
 * @return 当前的 MotorDriver_State_t 状态值。
 */
MotorDriver_State_t MotorDriver_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_DRIVE_MOTOR_DRIVER_H */
