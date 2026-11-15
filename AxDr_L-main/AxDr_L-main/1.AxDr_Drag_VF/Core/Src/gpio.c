/* Step 1 board setup. Bridge pins are GPIO outputs, never timer outputs. */
#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef io = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* Set all twelve active-high bridge input latches LOW before output mode. */
  HAL_GPIO_WritePin(GPIOA, BRIDGE_A_PINS|LED_FAULT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BRIDGE_B_PINS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, BRIDGE_C_PINS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC2_CS_GPIO_Port, ENC2_CS_Pin, GPIO_PIN_SET);

  io.Mode = GPIO_MODE_OUTPUT_PP;
  io.Pull = GPIO_PULLDOWN;
  io.Speed = GPIO_SPEED_FREQ_LOW;
  io.Pin = BRIDGE_A_PINS;
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = BRIDGE_B_PINS;
  HAL_GPIO_Init(GPIOB, &io);
  io.Pin = BRIDGE_C_PINS;
  HAL_GPIO_Init(GPIOC, &io);

  io.Pull = GPIO_NOPULL;
  io.Pin = LED_RUN_Pin;
  HAL_GPIO_Init(LED_RUN_GPIO_Port, &io);
  io.Pin = LED_FAULT_Pin;
  HAL_GPIO_Init(LED_FAULT_GPIO_Port, &io);

  io.Pull = GPIO_PULLUP;
  io.Pin = SPI1_CSN_Pin;
  HAL_GPIO_Init(SPI1_CSN_GPIO_Port, &io);
  io.Pin = ENC2_CS_Pin;
  HAL_GPIO_Init(ENC2_CS_GPIO_Port, &io);
}
