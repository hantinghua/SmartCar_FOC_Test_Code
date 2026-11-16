/* Step 2A: zero-current calibration and DISCONNECTED-MOTOR waveform checks.
 * No rotating field, no current loop, no motor-drive command.
 * Currents are polled while PWM is OFF; not PWM-synchronised phase currents. */
#include "board_check.h"
#include "adc.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define ZERO_SAMPLES 256u
#define PWM_DURATION_MS 30000u
static uint16_t raw[2], offset[2], minimum[2], maximum[2];
static uint32_t total[2], zero_count, adc_errors, sample_tick;
static uint8_t adc_ready, sample_valid, zero_valid, collecting, chart_mode;
static uint8_t check_fault, timer_ready;
static volatile uint8_t active_pair, expired;
static volatile uint32_t deadline;

static void send_text(const char *s)
{
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 50u);
}

static void bridge_gpio_low(void)
{
  GPIO_InitTypeDef io = {0};
  HAL_GPIO_WritePin(GPIOA, BRIDGE_A_PINS, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, BRIDGE_B_PINS, GPIO_PIN_RESET);
  io.Mode = GPIO_MODE_OUTPUT_PP;
  io.Pull = GPIO_PULLDOWN;
  io.Speed = GPIO_SPEED_FREQ_LOW;
  io.Pin = BRIDGE_A_PINS;
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = GPIO_PIN_12|GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &io);
}

static void stop_pwm(void)
{
  if (timer_ready != 0u) {
    HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
    HRTIM1->sMasterRegs.MCR &= ~(HRTIM_MCR_TACEN|HRTIM_MCR_TBCEN|HRTIM_MCR_TCCEN);
  }
  active_pair = 0u;
  expired = 0u;
  bridge_gpio_low();
}

/* SysTick cuts HRTIM output even if main is waiting for UART or ADC.
 * Not a hardware overcurrent protection mechanism. */
void board_check_tick(void)
{
  if (active_pair != 0u && (int32_t)(HAL_GetTick() - deadline) >= 0) {
    HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
    active_pair = 0u;
    expired = 1u;
  }
}

static uint8_t configure_timer(void)
{
  __HAL_RCC_HRTIM1_CLK_ENABLE();
  HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
  timer_ready = 1u;
  HRTIM1->sCommonRegs.DLLCR = HRTIM_DLLCR_CALEN |
      HRTIM_DLLCR_CALRTE_0|HRTIM_DLLCR_CALRTE_1|HRTIM_DLLCR_CAL;
  uint32_t start = HAL_GetTick();
  while ((HRTIM1->sCommonRegs.ISR & HRTIM_ISR_DLLRDY) == 0u) {
    if (HAL_GetTick() - start >= 20u) return 0u;
  }
  HRTIM1->sMasterRegs.MCR = (5u << HRTIM_MCR_CK_PSC_Pos);
  HRTIM1->sCommonRegs.ADC1R = 0u;
  HRTIM1->sCommonRegs.ADC2R = 0u;
  for (uint32_t i = 0u; i < 3u; i++) {
    HRTIM_Timerx_TypeDef *t = &HRTIM1->sTimerxRegs[i];
    t->TIMxCR = HRTIM_TIMCR_CONT | (5u << HRTIM_TIMCR_CK_PSC_Pos);
    t->PERxR = 8000u; /* 160 MHz / 8000 = 20 kHz; verify on scope. */
    t->REPxR = 0u;
    t->CMP1xR = 4000u;
    t->RSTxR = 0u;
    /* Positive deadtime, prescaler DIV1: 160 ticks / 160 MHz = 1 us. */
    t->DTxR = 160u | (160u << 16);
    t->SETx1R = HRTIM_SET1R_CMP1;
    t->RSTx1R = HRTIM_RST1R_PER;
    t->SETx2R = HRTIM_SET2R_CMP1;
    t->RSTx2R = HRTIM_RST2R_PER;
    t->OUTxR = HRTIM_OUTR_DTEN;
  }
  HRTIM1->sCommonRegs.CR2 = HRTIM_CR2_TASWU|HRTIM_CR2_TBSWU|HRTIM_CR2_TCSWU;
  return 1u;
}

static void start_scope_pair(uint8_t pair)
{
  if (zero_valid == 0u || check_fault != 0u || sample_valid == 0u) {
    send_text("# REJECTED: need zero_ok, valid ADC and no fault; motor must be disconnected\r\n");
    return;
  }
  stop_pwm();
  if (configure_timer() == 0u) {
    check_fault = 1u;
    stop_pwm();
    send_text("# FAULT HRTIM_DLL_TIMEOUT\r\n");
    return;
  }
  GPIO_InitTypeDef io = {0};
  io.Mode = GPIO_MODE_AF_PP;
  io.Pull = GPIO_PULLDOWN;
  io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  io.Alternate = GPIO_AF13_HRTIM1;
  io.Pin = pair == 0u ? GPIO_PIN_8|GPIO_PIN_9 :
           (pair == 1u ? GPIO_PIN_10|GPIO_PIN_11 : GPIO_PIN_12|GPIO_PIN_13);
  HAL_GPIO_Init(pair == 2u ? GPIOB : GPIOA, &io);
  HRTIM1->sCommonRegs.CR2 = HRTIM_CR2_TARST|HRTIM_CR2_TBRST|HRTIM_CR2_TCRST;
  deadline = HAL_GetTick() + PWM_DURATION_MS;
  active_pair = pair + 1u;
  HRTIM1->sMasterRegs.MCR |= (HRTIM_MCR_TACEN << pair);
  HRTIM1->sCommonRegs.OENR = (3u << (pair * 2u));
  char line[160];
  (void)snprintf(line, sizeof(line),
      "# SCOPE_ONLY phase=%c: 20kHz 50percent nominal deadtime 1us; 30s cutoff; x=stop; motor DISCONNECTED\r\n",
      'A' + pair);
  send_text(line);
}

static void start_zero(void)
{
  stop_pwm();
  collecting = 1u;
  zero_valid = 0u;
  zero_count = 0u;
  for (uint32_t i = 0u; i < 2u; i++) {
    total[i] = 0u;
    minimum[i] = 4095u;
    maximum[i] = 0u;
  }
  send_text("# ZERO_BEGIN: PWM off; keep motor stationary, no external current\r\n");
}

static uint8_t read_adc(void)
{
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) return 0u;
  HAL_StatusTypeDef result = HAL_ADCEx_InjectedPollForConversion(&hadc1, 2u);
  if (result == HAL_OK) {
    raw[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    raw[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
  }
  (void)HAL_ADCEx_InjectedStop(&hadc1);
  return result == HAL_OK;
}

void board_check_init(void)
{
  MX_ADC1_Init();
  adc_ready = HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) == HAL_OK;
  if (adc_ready == 0u) check_fault = 1u;
  send_text("# STEP2A ZERO_AND_SCOPE; motor motion NOT implemented; PWM OFF\r\n"
            "# PA0/PA1 current ADC; 5mOhm x20; nominal VDDA=3.3V\r\n"
            "# z=zero; f=FireWater; t=text; x=stop; c=clear check_fault\r\n"
            "# DISCONNECT motor phases for scope: u/v/w=A/B/C pair for 30s; direct switch allowed\r\n");
  start_zero();
}

void board_check_poll(void)
{
  if (expired != 0u) {
    expired = 0u;
    stop_pwm();
    send_text("# PWM_STOP timeout; all bridge inputs low\r\n");
  }
  if (active_pair != 0u) return; /* Never label asynchronous PWM samples as phase current. */
  uint32_t now = HAL_GetTick();
  if (now - sample_tick < 2u) return;
  sample_tick = now;
  sample_valid = adc_ready != 0u ? read_adc() : 0u;
  if (sample_valid == 0u) {
    adc_errors++;
    check_fault = 1u;
    zero_valid = collecting = 0u;
    return;
  }
  if (collecting != 0u) {
    for (uint32_t i = 0u; i < 2u; i++) {
      total[i] += raw[i];
      if (raw[i] < minimum[i]) minimum[i] = raw[i];
      if (raw[i] > maximum[i]) maximum[i] = raw[i];
    }
    if (++zero_count == ZERO_SAMPLES) {
      collecting = 0u;
      zero_valid = 1u;
      for (uint32_t i = 0u; i < 2u; i++) {
        offset[i] = (uint16_t)((total[i] + ZERO_SAMPLES / 2u) / ZERO_SAMPLES);
        if (offset[i] < 1800u || offset[i] > 2300u || maximum[i] - minimum[i] > 60u)
          zero_valid = 0u;
      }
      if (zero_valid == 0u) check_fault = 1u;
      char line[144];
      (void)snprintf(line, sizeof(line), "# ZERO_%s offset=%u,%u p2p=%u,%u counts; PWM OFF\r\n",
          zero_valid != 0u ? "OK" : "FAIL", offset[0], offset[1],
          maximum[0] - minimum[0], maximum[1] - minimum[1]);
      send_text(line);
    }
  }
}

void board_check_command(uint8_t command)
{
  switch (command) {
    case '?':
      send_text("# z=zero; f=FireWater; t=text; x=stop; c=clear-and-zero\r\n"
                "# Scope ONLY with motor phases disconnected: u/v/w=A/B/C for 30s; resend to restart\r\n");
      break;
    case 'x': stop_pwm(); send_text("# STOP all bridge inputs low\r\n"); break;
    case 'z': start_zero(); break;
    case 'f': chart_mode = 1u;
      send_text("# CH: angle_deg,UA_mV,VA_mV,IU_mA,IV_mA,valid,zero_ok,pwm_pair,check_fault\r\n"); break;
    case 't': chart_mode = 0u; break;
    case 'u': case 'v': case 'w': start_scope_pair(command - 'u'); break;
    case 'c':
      stop_pwm();
      if (adc_ready != 0u && sample_valid != 0u) { check_fault = 0u; start_zero(); }
      else send_text("# FAULT persists: ADC unavailable\r\n");
      break;
    default: break;
  }
}

uint8_t board_check_fault(void) { return check_fault; }
uint8_t board_check_chart(void) { return chart_mode; }

void board_check_report(uint32_t angle_centideg, uint8_t encoder_ok)
{
  char line[256];
  uint32_t mv0 = (uint32_t)raw[0] * 3300u / 4095u;
  uint32_t mv1 = (uint32_t)raw[1] * 3300u / 4095u;
  uint8_t valid = sample_valid && zero_valid && !collecting && !active_pair && !check_fault;
  int32_t ma0 = valid ? ((int32_t)raw[0] - offset[0]) * 33000 / 4095 : 0;
  int32_t ma1 = valid ? ((int32_t)raw[1] - offset[1]) * 33000 / 4095 : 0;
  if (chart_mode != 0u) {
    (void)snprintf(line, sizeof(line), "diag:%lu.%02lu,%lu,%lu,%ld,%ld,%u,%u,%u,%u\r\n",
        (unsigned long)(angle_centideg / 100u), (unsigned long)(angle_centideg % 100u),
        (unsigned long)mv0, (unsigned long)mv1, (long)ma0, (long)ma1,
        (unsigned int)(valid && encoder_ok), zero_valid, active_pair, check_fault);
  } else {
    (void)snprintf(line, sizeof(line),
        "# ADC raw=%u,%u mV=%lu,%lu zero=%u,%u I_mA=%ld,%ld valid=%u zero_ok=%u pwm_pair=%u check_fault=%u adc_err=%lu\r\n",
        raw[0], raw[1], (unsigned long)mv0, (unsigned long)mv1, offset[0], offset[1],
        (long)ma0, (long)ma1, valid, zero_valid, active_pair, check_fault, (unsigned long)adc_errors);
  }
  send_text(line);
}
