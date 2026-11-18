/* Step 2B: encoder/UART diagnostics with explicitly commanded fixed-vector pulse. */
#include "sensor_diag.h"
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "board_check.h"
#include <stdio.h>
#include <string.h>

volatile uint16_t g_step1_raw16;
volatile uint16_t g_step1_raw;
#define ENCODER_COUNTS (1u << SENSOR_DIAG_ENCODER_BITS)
#define ENCODER_MASK (ENCODER_COUNTS - 1u)
#if SENSOR_DIAG_ENCODER_BITS != 14u && SENSOR_DIAG_ENCODER_BITS != 15u
#error Unsupported encoder angle width
#endif
volatile uint32_t g_step1_spi_errors;
volatile uint32_t g_step1_uart_errors;
volatile uint8_t g_step1_spi_ok;
static uint8_t stream_enabled = 1u;
static uint8_t have_sample;
static uint16_t previous_raw;
static int32_t sample_delta;
static uint32_t sample_tick, report_tick, led_tick;

static void serial_text(const char *text)
{
  if (HAL_UART_Transmit(&huart3, (uint8_t *)text,
                        (uint16_t)strlen(text), 50u) != HAL_OK) {
    g_step1_uart_errors++;
  }
}

static void print_help(void)
{
  serial_text("\r\nSTEP2B_ENCODER_FIXED_VECTOR; HW_SPI; ENC1 only\r\n"
              "USART3 PC10=TX PC11=RX 115200 8N1\r\n"
              "ENC1 PB3=SCK PB4=MISO PB5=MOSI PA15=CS\r\n"
              "ANGLE_BITS=14; raw16 retained; verify ONE wrap per shaft turn\r\n"
              "?: help; h: one current vector point; p: pause; r: resume; s: one sample\r\n");
}

/* Keep the hardware transfer sequence used before cleanup. Reference:
 * Seekfree zf_driver_spi.c::spi_read_16bit_register and zf_device_menc15a.c.
 * CS stays low: complete command, discard command RX, complete dummy, read RX.
 * Only this function accesses SPI1; no SPI IRQ/DMA is enabled. */
static HAL_StatusTypeDef transfer_encoder(uint16_t *raw)
{
  SPI_TypeDef *spi = hspi1.Instance;
  volatile uint16_t *dr = (volatile uint16_t *)&spi->DR;
  uint32_t start = HAL_GetTick();
  const uint16_t tx[2] = {0x8021u, 0x0000u};
  while ((spi->SR & SPI_SR_FRLVL) != 0u) {
    (void)*dr;
    if ((uint32_t)(HAL_GetTick() - start) >= 5u) return HAL_TIMEOUT;
  }
  __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
  SET_BIT(spi->CR1, SPI_CR1_SPE);
  HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_RESET);
  for (uint32_t word = 0u; word < 2u; word++) {
    while ((spi->SR & SPI_SR_TXE) == 0u) {
      if ((uint32_t)(HAL_GetTick() - start) >= 5u) return HAL_TIMEOUT;
    }
    *dr = tx[word];
    while ((spi->SR & SPI_SR_TXE) == 0u || (spi->SR & SPI_SR_BSY) != 0u) {
      if ((uint32_t)(HAL_GetTick() - start) >= 5u) return HAL_TIMEOUT;
    }
    while ((spi->SR & SPI_SR_RXNE) == 0u) {
      if ((uint32_t)(HAL_GetTick() - start) >= 5u) return HAL_TIMEOUT;
    }
    uint16_t received = *dr;
    if (word == 0u) {
      while ((spi->SR & SPI_SR_FRLVL) != 0u) {
        (void)*dr;
        if ((uint32_t)(HAL_GetTick() - start) >= 5u) return HAL_TIMEOUT;
      }
      __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    } else {
      *raw = received;
    }
  }
  while ((spi->SR & SPI_SR_BSY) != 0u) {
    if ((uint32_t)(HAL_GetTick() - start) >= 5u) return HAL_TIMEOUT;
  }
  return (spi->SR & (SPI_SR_OVR|SPI_SR_MODF|SPI_SR_FRE)) != 0u ? HAL_ERROR : HAL_OK;
}

static void sample_encoder(void)
{
  uint16_t raw = 0u;
  HAL_StatusTypeDef status = transfer_encoder(&raw);
  /* Always release CS, including after a timeout. */
  HAL_GPIO_WritePin(SPI1_CSN_GPIO_Port, SPI1_CSN_Pin, GPIO_PIN_SET);
  g_step1_spi_ok = (status == HAL_OK);
  if (status != HAL_OK) {
    board_check_encoder(0u, 0u);
    g_step1_spi_errors++;
    have_sample = 0u;
    sample_delta = 0;
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    MX_SPI1_Init();
    return;
  }
  g_step1_raw16 = raw;
  g_step1_raw = raw & ENCODER_MASK;
  board_check_encoder((uint32_t)g_step1_raw * 36000u / ENCODER_COUNTS, raw != 0xFFFFu);
  sample_delta = 0;
  if (have_sample != 0u) {
    sample_delta = (int32_t)g_step1_raw - (int32_t)previous_raw;
    if (sample_delta > (int32_t)(ENCODER_COUNTS / 2u)) sample_delta -= (int32_t)ENCODER_COUNTS;
    if (sample_delta < -(int32_t)(ENCODER_COUNTS / 2u)) sample_delta += (int32_t)ENCODER_COUNTS;
  }
  previous_raw = g_step1_raw;
  have_sample = (raw != 0xFFFFu);
  HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin,
                   raw == 0xFFFFu || board_check_fault() != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void report_encoder(void)
{
  char line[192];
  if (g_step1_spi_ok == 0u) {
    (void)snprintf(line, sizeof(line),
        "ms=%lu angle=INVALID SPI_ERROR spi_err=%lu uart_err=%lu mode=FIXED_VECTOR_TEST\r\n",
        (unsigned long)HAL_GetTick(), (unsigned long)g_step1_spi_errors,
        (unsigned long)g_step1_uart_errors);
  } else {
    uint32_t centideg = (uint32_t)g_step1_raw * 36000u / ENCODER_COUNTS;
    (void)snprintf(line, sizeof(line),
        "ms=%lu raw16=0x%04X bits=%u raw=%u deg=%lu.%02lu delta=%ld spi_err=%lu uart_err=%lu%s mode=FIXED_VECTOR_TEST\r\n",
        (unsigned long)HAL_GetTick(), (unsigned int)g_step1_raw16,
        (unsigned int)SENSOR_DIAG_ENCODER_BITS, (unsigned int)g_step1_raw,
        (unsigned long)(centideg / 100u),
        (unsigned long)(centideg % 100u), (long)sample_delta,
        (unsigned long)g_step1_spi_errors, (unsigned long)g_step1_uart_errors,
        g_step1_raw16 == 0xFFFFu ? " CHECK_WIRING" : "");
  }
  serial_text(line);
}

static void poll_serial(void)
{
  uint8_t ch;
  if (HAL_UART_Receive(&huart3, &ch, 1u, 0u) == HAL_OK) {
    switch (ch) {
      case '?': print_help(); board_check_command('?'); break;
      case 'p': stream_enabled = 0u; serial_text("STREAM=PAUSED mode=FIXED_VECTOR_TEST\r\n"); break;
      case 'r': stream_enabled = 1u; serial_text("STREAM=RUNNING mode=FIXED_VECTOR_TEST\r\n"); break;
      case 's':
        sample_encoder();
        if (board_check_chart() == 0u) report_encoder();
        board_check_report((uint32_t)g_step1_raw * 36000u / ENCODER_COUNTS,
                           g_step1_spi_ok && g_step1_raw16 != 0xFFFFu);
        break;
      default: board_check_command(ch); break;
    }
  }
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) != RESET) {
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    g_step1_uart_errors++;
  }
}

void sensor_diag_init(void)
{
  HAL_Delay(100u);
  print_help();
  board_check_init();
  sample_tick = report_tick = led_tick = HAL_GetTick();
}

void sensor_diag_poll(void)
{
  uint32_t now = HAL_GetTick();
  poll_serial();
  board_check_poll();
  if ((uint32_t)(now - sample_tick) >= 20u) {
    sample_tick = now;
    sample_encoder();
  }
  if ((uint32_t)(now - report_tick) >= (board_check_chart() ? 20u : 200u)) {
    report_tick = now;
    if (stream_enabled != 0u) {
      if (board_check_chart() == 0u) report_encoder();
      board_check_report((uint32_t)g_step1_raw * 36000u / ENCODER_COUNTS,
                        g_step1_spi_ok && g_step1_raw16 != 0xFFFFu);
    }
  }
  if ((uint32_t)(now - led_tick) >= 500u) {
    led_tick = now;
    HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
  }
}
