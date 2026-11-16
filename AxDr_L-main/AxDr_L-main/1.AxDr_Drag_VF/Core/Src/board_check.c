/* Step 2B: single 100 ms fixed-vector pulse; no closed-loop rotation.
 * PWM-synchronised injected ADC, software current trip and timeout. */
#include "board_check.h"
#include "adc.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define ZERO_SAMPLES 256u
#define VECTOR_MS 100u
#define CURRENT_LIMIT_COUNTS 99 /* ~0.8 A: 5 mOhm, gain 20, VDDA 3.3 V */
static volatile uint16_t raw[2];
static uint16_t offset[2], minimum[2], maximum[2];
static uint32_t total[2], zero_count, adc_errors, sample_tick;
static uint8_t adc_ready, sample_valid, zero_valid, collecting, chart_mode;
static volatile uint8_t check_fault;
static uint8_t timer_ready;
static volatile uint32_t adc_tick, vector_samples, peak_counts;
static volatile uint32_t diag_irq_entries, diag_irq_jeos;
static uint32_t vector_start, next_allowed, encoder_tick, encoder_angle, angle_start;
static uint8_t encoder_valid;
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
  ADC1->IER &= ~ADC_IER_JEOSIE;
  (void)HAL_ADCEx_InjectedStop(&hadc1);
  MODIFY_REG(ADC1->JSQR, ADC_JSQR_JEXTSEL|ADC_JSQR_JEXTEN, 0u);
  active_pair = 0u;
  expired = 0u;
  bridge_gpio_low();
}

/* SysTick cuts HRTIM output even if main is waiting for UART or ADC.
 * Not a hardware overcurrent protection mechanism. */
static void cut_output(uint8_t reason)
{
  HRTIM1->sCommonRegs.ODISR = 0x0FFFu;
  ADC1->IER &= ~ADC_IER_JEOSIE;
  active_pair = 0u;
  expired = reason;
  if (reason != 1u) {
    check_fault = reason;
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
  }
}

void board_check_tick(void)
{
  if (active_pair == 4u || active_pair == 5u) {
    uint32_t now = HAL_GetTick();
    if (active_pair == 4u && (int32_t)(now - deadline) >= 0) cut_output(1u);
    else if (now - adc_tick >= 2u) cut_output(3u);
  }
}

void board_check_encoder(uint32_t angle, uint8_t ok)
{
  encoder_angle = angle;
  encoder_tick = HAL_GetTick();
  encoder_valid = ok;
  if (!ok && active_pair == 4u) cut_output(4u);
}

/* Own this IRQ: never call the legacy FOC HAL callback. No UART in ISR. */
void board_check_adc_irq(void)
{
  diag_irq_entries++;
  if ((ADC1->ISR & ADC_ISR_JEOS) == 0u) return;
  diag_irq_jeos++;
  raw[0] = (uint16_t)ADC1->JDR1;
  raw[1] = (uint16_t)ADC1->JDR2;
  ADC1->ISR = ADC_ISR_JEOS|ADC_ISR_JEOC;
  if (active_pair != 4u && active_pair != 5u) return;
  adc_tick = HAL_GetTick();
  vector_samples++;
  int32_t iu = (int32_t)raw[0] - offset[0];
  int32_t iv = (int32_t)raw[1] - offset[1];
  int32_t currents[3] = {iu, iv, -iu-iv};
  for (uint32_t i = 0; i < 3; i++) {
    uint32_t magnitude = currents[i] < 0 ? (uint32_t)-currents[i] : (uint32_t)currents[i];
    if (magnitude > peak_counts) peak_counts = magnitude;
    if (magnitude >= CURRENT_LIMIT_COUNTS || raw[0] < 100u || raw[0] > 3995u ||
        raw[1] < 100u || raw[1] > 3995u) { cut_output(2u); return; }
  }
  if (active_pair == 5u) return; /* Trigger self-test: all outputs disabled. */
  /* Ramp 0 -> 1.5% line-line duty over 50 ms, then hold to 100 ms.
   * CMP1 is the rising edge: all low-side FETs are ON at CMP2=1600.
   * Updates happen after ADC completion, well before CMP1 near 4000. */
  uint32_t elapsed = adc_tick - vector_start;
  uint32_t delta = elapsed < 50u ? elapsed * 80u / 50u : 80u;
  HRTIM1->sTimerxRegs[0].CMP1xR = 4000u - delta;
  HRTIM1->sTimerxRegs[1].CMP1xR = 4000u + delta / 2u;
  HRTIM1->sTimerxRegs[2].CMP1xR = 4000u + delta / 2u;
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

/* Shared by pulse preflight and the board-validated d diagnostic. */
static void configure_adc_trigger(void)
{
  /* ADC1 injected input supports TRG2; TRG1 is limited to ADC3/4/5.
   * HRTIM trigger 2 accepts Timer A CMP2 (not CMP3). */
  HRTIM1->sTimerxRegs[0].CMP2xR = 1600u;
  MODIFY_REG(HRTIM1->sCommonRegs.CR1, HRTIM_CR1_ADC2USRC, HRTIM_CR1_ADC2USRC_0);
  MODIFY_REG(HRTIM1->sCommonRegs.ADCPS1, HRTIM_ADCPS1_AD2PSC, 0u);
  HRTIM1->sCommonRegs.ADC2R = HRTIM_ADC2R_AD2TAC2;
  HRTIM1->sCommonRegs.CR2 = HRTIM_CR2_TASWU|HRTIM_CR2_MSWU;
  MODIFY_REG(ADC1->JSQR, ADC_JSQR_JEXTSEL|ADC_JSQR_JEXTEN,
             ADC_EXTERNALTRIGINJEC_HRTIM_TRG2|ADC_EXTERNALTRIGINJECCONV_EDGE_RISING);
}

static void start_vector(void)
{
  uint32_t now = HAL_GetTick();
  if (active_pair || expired || collecting || !zero_valid || !sample_valid ||
      check_fault || !encoder_valid || now-encoder_tick > 100u ||
      (next_allowed && (int32_t)(now-next_allowed) < 0)) {
    send_text("# VECTOR_REJECTED: need zero_ok, ADC, encoder, no fault; idle + 2s cooldown\r\n");
    return;
  }
  stop_pwm();
  if (!configure_timer()) {
    stop_pwm(); check_fault = 5u;
    send_text("# FAULT HRTIM_DLL_TIMEOUT PWM_OFF\r\n"); return;
  }
  configure_adc_trigger();
  ADC1->ISR = ADC_ISR_JEOS|ADC_ISR_JEOC;
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 1u, 0u);
  HAL_NVIC_ClearPendingIRQ(ADC1_2_IRQn);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  vector_samples = peak_counts = 0u;
  ADC1->IER |= ADC_IER_JEOSIE;
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
    stop_pwm(); check_fault = 6u;
    send_text("# FAULT ADC_START PWM_OFF\r\n"); return;
  }
  adc_tick = HAL_GetTick();
  uint32_t sync_start = adc_tick;
  active_pair = 5u; /* Preflight: GPIO low and outputs disabled. */
  HRTIM1->sCommonRegs.CR2 = HRTIM_CR2_TARST|HRTIM_CR2_TBRST|HRTIM_CR2_TCRST;
  HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN|HRTIM_MCR_TBCEN|HRTIM_MCR_TCCEN;
  while (active_pair == 5u && HAL_GetTick()-sync_start < 5u) { }
  if (active_pair != 5u || vector_samples < 64u || vector_samples > 140u) {
    uint32_t samples = vector_samples;
    uint8_t reason = check_fault ? check_fault : 7u;
    stop_pwm(); check_fault = reason;
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
    char line[112];
    (void)snprintf(line, sizeof(line), "# SYNC_FAIL PWM_OFF fault=%u samples=%lu; vector NOT started\r\n",
                   reason, (unsigned long)samples);
    send_text(line); return;
  }
  GPIO_InitTypeDef io = {0};
  io.Mode = GPIO_MODE_AF_PP;
  io.Pull = GPIO_PULLDOWN;
  io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  io.Alternate = GPIO_AF13_HRTIM1;
  io.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  HAL_GPIO_Init(GPIOA, &io);
  io.Pin = GPIO_PIN_12|GPIO_PIN_13;
  HAL_GPIO_Init(GPIOB, &io);
  /* Timers continue running in phase; ADC IRQ excluded only for handover. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (active_pair != 5u || check_fault) {
    __set_PRIMASK(primask); stop_pwm(); return;
  }
  vector_samples = peak_counts = 0u;
  angle_start = encoder_angle;
  vector_start = adc_tick = HAL_GetTick();
  deadline = vector_start + VECTOR_MS;
  next_allowed = vector_start + VECTOR_MS + 2000u;
  active_pair = 4u;
  HRTIM1->sCommonRegs.OENR = 0x3Fu;
  __set_PRIMASK(primask);
  send_text("# VECTOR_BEGIN R5 TRG2; 100ms; ramp=50ms; max_line_duty=1.5percent (~0.18V at 12V); x=stop\r\n");
}

/* Read-only event diagnosis: no AF pins or output enables, independent of
 * the vector watchdog. Poll first with ADC IRQ disabled, then test IRQ delivery. */
static uint8_t read_adc(void);

static void diagnose_sync(void)
{
  if (active_pair || expired || collecting || !adc_ready) {
    send_text("# DIAG_REJECTED: wait for zero calibration and PWM idle\r\n");
    return;
  }
  stop_pwm();
  send_text("# DIAG_BEGIN R5 PWM_OFF; route=TA_CMP2->HRTIM_TRG2->ADC1; stages=POLL,IRQ,SW\r\n");
  if (!configure_timer()) {
    stop_pwm(); send_text("# DIAG_FAIL DLL_TIMEOUT PWM_OFF\r\n"); return;
  }
  configure_adc_trigger();
  ADC1->IER &= ~ADC_IER_JEOSIE;
  ADC1->ISR = ADC_ISR_JEOS|ADC_ISR_JEOC;
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
    stop_pwm(); send_text("# DIAG_FAIL ADC_START PWM_OFF\r\n"); return;
  }
  HRTIM1->sTimerxRegs[0].TIMxICR = HRTIM_TIMICR_CMP2C;
  HRTIM1->sCommonRegs.CR2 = HRTIM_CR2_TARST;
  HRTIM1->sMasterRegs.MCR |= HRTIM_MCR_TACEN;
  uint32_t start = HAL_GetTick(), cmp_events = 0u, adc_eos = 0u, adc_seen = 0u;
  uint32_t count_begin = HRTIM1->sTimerxRegs[0].CNTxR;
  while (HAL_GetTick()-start < 5u) {
    if (HRTIM1->sTimerxRegs[0].TIMxISR & HRTIM_TIMISR_CMP2) {
      cmp_events++;
      HRTIM1->sTimerxRegs[0].TIMxICR = HRTIM_TIMICR_CMP2C;
    }
    uint32_t flags = ADC1->ISR;
    adc_seen |= flags;
    if (flags & ADC_ISR_JEOS) {
      adc_eos++;
      raw[0] = (uint16_t)ADC1->JDR1; raw[1] = (uint16_t)ADC1->JDR2;
      ADC1->ISR = ADC_ISR_JEOS|ADC_ISR_JEOC;
    }
  }
  /* Snapshot before stopping: stop changes ADC CR/JSQR. */
  uint32_t count_end = HRTIM1->sTimerxRegs[0].CNTxR;
  uint32_t adc_cr = ADC1->CR, jsqr = ADC1->JSQR;
  uint32_t adc_ier = ADC1->IER, tim_isr = HRTIM1->sTimerxRegs[0].TIMxISR;
  uint32_t hr_cr1 = HRTIM1->sCommonRegs.CR1, hr_adc2r = HRTIM1->sCommonRegs.ADC2R;
  uint32_t hr_mcr = HRTIM1->sMasterRegs.MCR;
  diag_irq_entries = diag_irq_jeos = 0u;
  ADC1->ISR = ADC_ISR_JEOS|ADC_ISR_JEOC;
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 1u, 0u);
  HAL_NVIC_ClearPendingIRQ(ADC1_2_IRQn);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  ADC1->IER |= ADC_IER_JEOSIE;
  start = HAL_GetTick();
  while (HAL_GetTick()-start < 5u) { }
  ADC1->IER &= ~ADC_IER_JEOSIE;
  uint32_t irq_entries = diag_irq_entries, irq_eos = diag_irq_jeos;
  uint32_t irq_flags = ADC1->ISR;
  stop_pwm();
  /* Positive control: existing software-triggered two-channel conversion. */
  uint8_t sw_ok = read_adc();
  char line[224];
  (void)snprintf(line, sizeof(line),
      "# DIAG_POLL PWM_OFF cmp2=%lu adc_eos=%lu ADC_SEEN=0x%08lX CNT=%lu,%lu TIM_ISR=0x%08lX\r\n",
      (unsigned long)cmp_events, (unsigned long)adc_eos, (unsigned long)adc_seen,
      (unsigned long)count_begin, (unsigned long)count_end, (unsigned long)tim_isr);
  send_text(line);
  (void)snprintf(line, sizeof(line),
      "# DIAG_REG ADC_CR=0x%08lX JSQR=0x%08lX IER=0x%08lX HR_CR1=0x%08lX ADC2R=0x%08lX MCR=0x%08lX\r\n",
      (unsigned long)adc_cr, (unsigned long)jsqr, (unsigned long)adc_ier,
      (unsigned long)hr_cr1, (unsigned long)hr_adc2r, (unsigned long)hr_mcr);
  send_text(line);
  (void)snprintf(line, sizeof(line),
      "# DIAG_IRQ entries=%lu jeos=%lu ISR=0x%08lX; SW_OK=%u raw=%u,%u; PWM_OFF\r\n",
      (unsigned long)irq_entries, (unsigned long)irq_eos, (unsigned long)irq_flags,
      sw_ok, raw[0], raw[1]);
  send_text(line);
  send_text("# DIAG_END PWM_OFF; existing fault retained; no motor drive\r\n");
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
  send_text("# STEP2B_R5 FIXED_VECTOR_TRG2; PWM OFF at boot\r\n"
            "# PA0/PA1 current ADC; 5mOhm x20; nominal VDDA=3.3V\r\n"
            "# z=zero; f=FireWater; t=text; x=stop; c=clear check_fault\r\n"
            "# d=PWM-off diagnosis; a=single 100ms fixed vector; u/v/w disabled; current trip ~0.8A\r\n");
  start_zero();
}

void board_check_poll(void)
{
  if (expired != 0u) {
    uint8_t reason = expired;
    stop_pwm();
    char line[192];
    (void)snprintf(line, sizeof(line),
        "# VECTOR_END reason=%u (1=timeout 2=current 3=ADC_stale 4=encoder); samples=%lu peak_mA=%lu angle_start=%lu.%02lu angle_now=%lu.%02lu\r\n",
        reason, (unsigned long)vector_samples, (unsigned long)(peak_counts*33000u/4095u),
        (unsigned long)(angle_start/100u), (unsigned long)(angle_start%100u),
        (unsigned long)(encoder_angle/100u), (unsigned long)(encoder_angle%100u));
    send_text(line);
  }
  if (active_pair != 0u) return; /* PWM ADC runs independently in its IRQ. */
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
                "# d=PWM-off diagnosis; a=single 100ms fixed vector; 2s cooldown; u/v/w disabled\r\n");
      break;
    case 'x': stop_pwm(); send_text("# STOP all bridge inputs low\r\n"); break;
    case 'z': start_zero(); break;
    case 'f': chart_mode = 1u;
      send_text("# CH: angle_deg,UA_mV,VA_mV,IU_mA,IV_mA,valid,zero_ok,pwm_pair,check_fault\r\n"); break;
    case 't': chart_mode = 0u; break;
    case 'a': start_vector(); break;
    case 'd': diagnose_sync(); break;
    case 'u': case 'v': case 'w': send_text("# DISABLED in STEP2B; a=fixed vector only\r\n"); break;
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
  uint8_t valid = sample_valid && zero_valid && !collecting && (!active_pair || active_pair == 4u) && !check_fault;
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
