/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hrtim_pwm.h
  * @brief   HRTIM1 PWM 驱动（寄存器级，不依赖 HAL_HRTIM）
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __HRTIM_PWM_H
#define __HRTIM_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 定时器索引：0=A 1=B 2=C 3=D 4=E 5=F */
#define HRTIM_IDX_A   0u
#define HRTIM_IDX_B   1u
#define HRTIM_IDX_C   2u
#define HRTIM_IDX_D   3u
#define HRTIM_IDX_E   4u
#define HRTIM_IDX_F   5u

/* 电机1 = A/B/C，电机2 = D/F/E（注意 B/C 与 F/E 的对应关系） */
#define HRTIM_M1_A    HRTIM_IDX_A
#define HRTIM_M1_B    HRTIM_IDX_B
#define HRTIM_M1_C    HRTIM_IDX_C
#define HRTIM_M2_A    HRTIM_IDX_D
#define HRTIM_M2_B    HRTIM_IDX_F
#define HRTIM_M2_C    HRTIM_IDX_E

/* 调试安全开关：只开单边输出，杜绝直通
 *   0 = 完全不开输出（静态测试预驱用）
 *   1 = 只开高边 CHx1
 *   2 = 只开低边 CHx2
 *   3 = 双边互补（确认单边都正常后再用） */
#define PWM_DEBUG_SINGLE_SIDE   3u

/* 被测定时器：0=A(电机1A) 1=B 2=C 3=D(电机2A) 4=E 5=F
 * 使能和自检都用这个宏，改这里即可，不要只改 main.c 里的一处 */
#define PWM_DEBUG_TIMER         2u

/* 调试占空比：用来判断大电流是否随占空比变化
 *   0.5  = 50%（正常）
 *   0.05 = 5%（如果电流跟着降 → 每个周期在直通；不变 → 静态漏电） */
#define PWM_TEST_DUTY           0.5f

/* 1 = 三相 A/B/C 一起输出（50% 时相间无压差，安全）
 * 0 = 只输出 PWM_DEBUG_TIMER 指定的那一相（单路调试） */
#define PWM_TEST_ALL_PHASES     1u

/* 返回值：0 = 初始化成功；1 = DLL 校准超时（HRTIM 时钟异常） */
uint32_t hrtim_pwm_init(void);
void hrtim_pwm_outputs_enable(uint32_t timers_mask);
void hrtim_pwm_outputs_disable(void);
/* which: 1 = 只开 OUT1(高边)，2 = 只开 OUT2(低边)，3 = 两个都开 */
void hrtim_pwm_output_enable_one(uint32_t timer_idx, uint32_t which);
void hrtim_set_duty(uint32_t timer_idx, float duty);
void hrtim_pwm_adc_trigger_init(uint32_t cmp4);
void hrtim_selfcheck_beep(void);

/* 上电三声自检开关（1=开，0=关；调试 FOC 期间建议关掉） */
#define PWM_SELFCHECK_BEEP      0u

#ifdef __cplusplus
}
#endif

#endif /* __HRTIM_PWM_H */
