#include "common.h"
#include "spi.h"
#include "encoder.h"

#define MENC15A_REG_ANGLE   0x8021u
#define MENC15A_REG_SPEED   0x8032u

/* Seekfree reference: low 15 bits, 32768 counts/revolution.
 * 14-bit mode is available only to diagnose the previous implementation. */
static uint16_t angle_mask = 0x7FFFu;
static float angle_counts = 32768.0f;
volatile uint8_t g_enc_last_ok = 0u;
void encoder_set_bits(uint8_t bits)
{
  angle_mask = bits == 14u ? 0x3FFFu : 0x7FFFu;
  angle_counts = bits == 14u ? 16384.0f : 32768.0f;
}
#define MENC15A_TWO_PI      6.28318531f

static uint16_t enc_raw[2] = { 0u, 0u };
volatile uint8_t  g_enc_spi_err = 0u;   /* 1 = HAL SPI 返回错误 */
volatile uint16_t g_enc1_raw = 0u;      /* 编码器1 最新角度（调试 watch 用） */
volatile uint16_t g_enc2_raw = 0u;      /* 编码器2 最新角度 */
volatile uint16_t g_enc1_raw16 = 0u;    /* 编码器1 完整 16 位原始值（调试用） */
volatile float    g_enc1_deg = 0.0f;    /* 编码器1 机械角度（0~360 度，调试用） */
volatile float    g_enc2_deg = 0.0f;    /* 编码器2 机械角度 */

/* 一次读寄存器：CS 低 -> 发 16 位地址（丢弃返回）-> 再发 0 读回数据 -> CS 高
 * 注意：MENC15A 的数据在第二段传输才返回（与逐飞库 spi_read_16bit_register 一致）
 */
static uint16_t menc15a_transfer(SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port,
                                 uint16_t cs_pin,
                                 uint16_t reg,
                                 uint8_t *ok)
{
  SPI_TypeDef *spi = hspi->Instance;
  uint32_t started = DWT->CYCCNT;
  uint32_t budget = SystemCoreClock / 62500u; /* 16 us for the WHOLE transaction */
  uint16_t rx = 0u;
  *ok = 0u;
  /* Clear stale receive data / OVR after a previous failed transaction. */
  while ((spi->SR & SPI_SR_RXNE) != 0u) {
    (void)*((volatile uint16_t *)&spi->DR);
    if (DWT->CYCCNT - started >= budget) return 0u;
  }
  (void)spi->SR;
  SET_BIT(spi->CR1, SPI_CR1_SPE);
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
  for (uint32_t word = 0u; word < 2u; word++) {
    while ((spi->SR & SPI_SR_TXE) == 0u) {
      if (DWT->CYCCNT - started >= budget) goto transfer_done;
    }
    *((volatile uint16_t *)&spi->DR) = word == 0u ? reg : 0u;
    while ((spi->SR & SPI_SR_RXNE) == 0u) {
      if (DWT->CYCCNT - started >= budget) goto transfer_done;
    }
    rx = *((volatile uint16_t *)&spi->DR);
  }
  while ((spi->SR & SPI_SR_BSY) != 0u) {
    if (DWT->CYCCNT - started >= budget) goto transfer_done;
  }
  *ok = 1u;
transfer_done:
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  if (*ok == 0u) CLEAR_BIT(spi->CR1, SPI_CR1_SPE);
  return rx;
}

uint16_t encoder_read_raw(uint8_t idx)
{
  uint16_t rx;
  uint8_t  ok;

  if (idx == ENC_IDX_1)
  {
    rx = menc15a_transfer(&hspi1, ENC1_CS_GPIO_Port, ENC1_CS_Pin, MENC15A_REG_ANGLE, &ok);
  }
  else
  {
    rx = menc15a_transfer(&hspi4, ENC2_CS_GPIO_Port, ENC2_CS_Pin, MENC15A_REG_ANGLE, &ok);
  }

  g_enc_last_ok = ok;
  if (ok == 0u)
  {
    /* 读取失败：保留上一次角度，避免角度突然跳到 0 导致电机失控 */
    g_enc_spi_err = 1u;
    return enc_raw[idx];
  }

  enc_raw[idx] = (uint16_t)(rx & angle_mask);

  if (idx == ENC_IDX_1)
  {
    g_enc1_raw16 = rx;
    g_enc1_raw = enc_raw[idx];
    g_enc1_deg = (float)enc_raw[idx] * 360.0f / angle_counts;
  }
  else
  {
    g_enc2_raw = enc_raw[idx];
    g_enc2_deg = (float)enc_raw[idx] * 360.0f / angle_counts;
  }

  return enc_raw[idx];
}

float encoder_read_angle_rad(uint8_t idx)
{
  return (float)encoder_read_raw(idx) * MENC15A_TWO_PI / angle_counts;
}

float encoder_read_angle_norm(uint8_t idx)
{
  return (float)encoder_read_raw(idx) / angle_counts;
}

int16_t encoder_read_speed_raw(uint8_t idx)
{
  uint16_t rx;
  int16_t  v;
  uint8_t  ok;

  if (idx == ENC_IDX_1)
  {
    rx = menc15a_transfer(&hspi1, ENC1_CS_GPIO_Port, ENC1_CS_Pin, MENC15A_REG_SPEED, &ok);
  }
  else
  {
    rx = menc15a_transfer(&hspi4, ENC2_CS_GPIO_Port, ENC2_CS_Pin, MENC15A_REG_SPEED, &ok);
  }

  if (ok == 0u)
  {
    g_enc_spi_err = 1u;
    return 0;
  }

  v = (int16_t)(rx & 0x7FFFu);
  if (v > 16384)
  {
    v = (int16_t)(v - 32768);
  }
  return v;
}

uint8_t encoder_check_alive(uint8_t idx)
{
  uint16_t a = encoder_read_raw(idx);
  uint16_t b = encoder_read_raw(idx);

  /* 通信失败时常见的是全 0 或全 1 */
  if ((a == 0u && b == 0u) || (a == angle_mask && b == angle_mask))
  {
    return 0u;
  }
  return 1u;
}
