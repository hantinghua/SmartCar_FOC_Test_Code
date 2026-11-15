/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "common.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* ============================ 本板引脚定义 ============================ */
/* 磁编码器 CS（外部 10k 上拉，空闲为高）                                    */
#define ENC1_CS_Pin GPIO_PIN_15
#define ENC1_CS_GPIO_Port GPIOA          /* PA15 : MENC15A #1 片选 */
#define ENC2_CS_Pin GPIO_PIN_11
#define ENC2_CS_GPIO_Port GPIOE          /* PE11 : MENC15A #2 片选 */

/* 指示灯                                                                     */
#define LED_FAULT_Pin GPIO_PIN_12
#define LED_FAULT_GPIO_Port GPIOA        /* PA12 : 故障灯 */
#define LED_RUN_Pin GPIO_PIN_2
#define LED_RUN_GPIO_Port GPIOD          /* PD2  : 运行灯 */

/* 按键（外部 10k 上拉，按下为低）                                            */
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOC              /* PC0  : 按键 */

/* ---- PWM：HRTIM1 12 路（AF13），两路电机各 3 个半桥 ---------------------- */
/*  电机1：TA = PA8/PA9   TB = PA10/PA11   TC = PB12/PB13                    */
/*  电机2：TD = PB14/PB15  TF = PC6/PC7     TE = PC8/PC9                      */
/*  电流采样：ADC1 注入 PA0(IN1)/PA1(IN2)；ADC2 注入 PA4(IN17)/PA5(IN13)      */
/*  母线电压：ADC1 规则组 PB11(IN14)                                          */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
