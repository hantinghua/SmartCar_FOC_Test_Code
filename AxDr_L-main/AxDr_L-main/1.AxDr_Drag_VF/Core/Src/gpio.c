/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC1_CS_GPIO_Port, ENC1_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC2_CS_GPIO_Port, ENC2_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);

  /*--------------------------------------------------------------------------*/
  /* HRTIM1 12 路 PWM（AF13）                                                  */
  /*   PA8/PA9   = CHA1/CHA2   (电机1 A 相)                                    */
  /*   PA10/PA11 = CHB1/CHB2   (电机1 B 相)                                    */
  /*   PB12/PB13 = CHC1/CHC2   (电机1 C 相)                                    */
  /*   PB14/PB15 = CHD1/CHD2   (电机2 A 相)                                    */
  /*   PC6/PC7   = CHF1/CHF2   (电机2 B 相)                                    */
  /*   PC8/PC9   = CHE1/CHE2   (电机2 C 相)                                    */
  /*--------------------------------------------------------------------------*/
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_HRTIM1;

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC1_CS_Pin (PA15) */
  GPIO_InitStruct.Pin = ENC1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ENC1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC2_CS_Pin (PE11) */
  GPIO_InitStruct.Pin = ENC2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ENC2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_FAULT_Pin (PA12) */
  GPIO_InitStruct.Pin = LED_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RUN_Pin (PD2) */
  GPIO_InitStruct.Pin = LED_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin (PC0) */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
