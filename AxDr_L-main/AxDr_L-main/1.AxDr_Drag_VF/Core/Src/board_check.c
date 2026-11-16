/* Step 2B: single 100 ms fixed-vector pulse; no closed-loop rotation.
 * PWM-synchronised injected ADC, software current trip and timeout. */
#include "board_check.h"
#include "adc.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define ZERO_SAMPLES 256u
#define VECTOR_MS 100u
#define SWEEP_MS 400u
#define SWEEP_RAMP_MS 50u
#define SWEEP_MOVE_MS 300u
#define IDENT_ALIGN_MS 250u
#define IDENT_MOVE_MS 2000u
#define IDENT_TOTAL_MS 2500u
#define CURRENT_LIMIT_COUNTS 99 /* ~0.8 A: 5 mOhm, gain 20, VDDA 3.3 V */
static volatile uint16_t raw[2];
static uint16_t offset[2], minimum[2], maximum[2];
static uint32_t total[2], zero_count, adc_errors, sample_tick;
static uint8_t adc_ready, sample_valid, zero_valid, collecting, chart_mode;
static volatile uint8_t check_fault;
static uint8_t timer_ready;
static volatile uint32_t adc_tick, vector_samples, peak_counts;
static volatile uint32_t diag_irq_entries, diag_irq_jeos;
static uint32_t vector_start, next_allowed, encoder_angle, angle_start;
static volatile uint32_t encoder_tick;
static int32_t ident_travel, ident_base, ident_last_delta;
static uint32_t ident_previous, ident_points, ident_base_angle;
static uint8_t ident_have, ident_started;
static uint8_t encoder_valid;
static volatile uint8_t vector_direction; /* 0:a 1:b 2:g 3:j 4:n full-forward 5:m full-reverse */
static volatile uint8_t active_pair, expired;
static volatile uint32_t deadline;

/* Phase duty offsets in timer ticks. Offline-generated alpha/beta rotation,
 * normalised so max phase-to-phase spread never exceeds 120/8000 (1.5%).
 * Integer-degree lookup keeps trigonometry out of the 20 kHz ADC IRQ. */
static const int16_t sweep_phase[361][3] = {
  {80, -40, -40},
  {79, -38, -40},
  {78, -36, -41},
  {77, -35, -42},
  {76, -33, -43},
  {76, -32, -43},
  {75, -30, -44},
  {74, -29, -45},
  {73, -27, -46},
  {73, -26, -46},
  {72, -25, -47},
  {71, -23, -48},
  {71, -22, -48},
  {70, -21, -49},
  {69, -19, -50},
  {69, -18, -50},
  {68, -17, -51},
  {67, -15, -52},
  {67, -14, -52},
  {66, -13, -53},
  {66, -12, -53},
  {65, -10, -54},
  {64, -9, -55},
  {64, -8, -55},
  {63, -7, -56},
  {63, -6, -56},
  {62, -4, -57},
  {61, -3, -58},
  {61, -2, -58},
  {60, -1, -59},
  {60, 0, -60},
  {59, 1, -60},
  {58, 2, -61},
  {58, 3, -61},
  {57, 4, -62},
  {56, 6, -63},
  {56, 7, -63},
  {55, 8, -64},
  {55, 9, -64},
  {54, 10, -65},
  {53, 12, -66},
  {53, 13, -66},
  {52, 14, -67},
  {52, 15, -67},
  {51, 17, -68},
  {50, 18, -69},
  {50, 19, -69},
  {49, 21, -70},
  {48, 22, -71},
  {48, 23, -71},
  {47, 25, -72},
  {46, 26, -73},
  {46, 27, -73},
  {45, 29, -74},
  {44, 30, -75},
  {43, 32, -76},
  {43, 33, -76},
  {42, 35, -77},
  {41, 36, -78},
  {40, 38, -79},
  {40, 39, -80},
  {38, 40, -79},
  {36, 41, -78},
  {35, 42, -77},
  {33, 43, -76},
  {32, 43, -76},
  {30, 44, -75},
  {29, 45, -74},
  {27, 46, -73},
  {26, 46, -73},
  {25, 47, -72},
  {23, 48, -71},
  {22, 48, -71},
  {21, 49, -70},
  {19, 50, -69},
  {18, 50, -69},
  {17, 51, -68},
  {15, 52, -67},
  {14, 52, -67},
  {13, 53, -66},
  {12, 53, -66},
  {10, 54, -65},
  {9, 55, -64},
  {8, 55, -64},
  {7, 56, -63},
  {6, 56, -63},
  {4, 57, -62},
  {3, 58, -61},
  {2, 58, -61},
  {1, 59, -60},
  {0, 60, -60},
  {-1, 60, -59},
  {-2, 61, -58},
  {-3, 61, -58},
  {-4, 62, -57},
  {-6, 63, -56},
  {-7, 63, -56},
  {-8, 64, -55},
  {-9, 64, -55},
  {-10, 65, -54},
  {-12, 66, -53},
  {-13, 66, -53},
  {-14, 67, -52},
  {-15, 67, -52},
  {-17, 68, -51},
  {-18, 69, -50},
  {-19, 69, -50},
  {-21, 70, -49},
  {-22, 71, -48},
  {-23, 71, -48},
  {-25, 72, -47},
  {-26, 73, -46},
  {-27, 73, -46},
  {-29, 74, -45},
  {-30, 75, -44},
  {-32, 76, -43},
  {-33, 76, -43},
  {-35, 77, -42},
  {-36, 78, -41},
  {-38, 79, -40},
  {-39, 80, -40},
  {-40, 79, -38},
  {-41, 78, -36},
  {-42, 77, -35},
  {-43, 76, -33},
  {-43, 76, -32},
  {-44, 75, -30},
  {-45, 74, -29},
  {-46, 73, -27},
  {-46, 73, -26},
  {-47, 72, -25},
  {-48, 71, -23},
  {-48, 71, -22},
  {-49, 70, -21},
  {-50, 69, -19},
  {-50, 69, -18},
  {-51, 68, -17},
  {-52, 67, -15},
  {-52, 67, -14},
  {-53, 66, -13},
  {-53, 66, -12},
  {-54, 65, -10},
  {-55, 64, -9},
  {-55, 64, -8},
  {-56, 63, -7},
  {-56, 63, -6},
  {-57, 62, -4},
  {-58, 61, -3},
  {-58, 61, -2},
  {-59, 60, -1},
  {-60, 60, 0},
  {-60, 59, 1},
  {-61, 58, 2},
  {-61, 58, 3},
  {-62, 57, 4},
  {-63, 56, 6},
  {-63, 56, 7},
  {-64, 55, 8},
  {-64, 55, 9},
  {-65, 54, 10},
  {-66, 53, 12},
  {-66, 53, 13},
  {-67, 52, 14},
  {-67, 52, 15},
  {-68, 51, 17},
  {-69, 50, 18},
  {-69, 50, 19},
  {-70, 49, 21},
  {-71, 48, 22},
  {-71, 48, 23},
  {-72, 47, 25},
  {-73, 46, 26},
  {-73, 46, 27},
  {-74, 45, 29},
  {-75, 44, 30},
  {-76, 43, 32},
  {-76, 43, 33},
  {-77, 42, 35},
  {-78, 41, 36},
  {-79, 40, 38},
  {-80, 40, 39},
  {-79, 38, 40},
  {-78, 36, 41},
  {-77, 35, 42},
  {-76, 33, 43},
  {-76, 32, 43},
  {-75, 30, 44},
  {-74, 29, 45},
  {-73, 27, 46},
  {-73, 26, 46},
  {-72, 25, 47},
  {-71, 23, 48},
  {-71, 22, 48},
  {-70, 21, 49},
  {-69, 19, 50},
  {-69, 18, 50},
  {-68, 17, 51},
  {-67, 15, 52},
  {-67, 14, 52},
  {-66, 13, 53},
  {-66, 12, 53},
  {-65, 10, 54},
  {-64, 9, 55},
  {-64, 8, 55},
  {-63, 7, 56},
  {-63, 6, 56},
  {-62, 4, 57},
  {-61, 3, 58},
  {-61, 2, 58},
  {-60, 1, 59},
  {-60, 0, 60},
  {-59, -1, 60},
  {-58, -2, 61},
  {-58, -3, 61},
  {-57, -4, 62},
  {-56, -6, 63},
  {-56, -7, 63},
  {-55, -8, 64},
  {-55, -9, 64},
  {-54, -10, 65},
  {-53, -12, 66},
  {-53, -13, 66},
  {-52, -14, 67},
  {-52, -15, 67},
  {-51, -17, 68},
  {-50, -18, 69},
  {-50, -19, 69},
  {-49, -21, 70},
  {-48, -22, 71},
  {-48, -23, 71},
  {-47, -25, 72},
  {-46, -26, 73},
  {-46, -27, 73},
  {-45, -29, 74},
  {-44, -30, 75},
  {-43, -32, 76},
  {-43, -33, 76},
  {-42, -35, 77},
  {-41, -36, 78},
  {-40, -38, 79},
  {-40, -39, 79},
  {-38, -40, 79},
  {-36, -41, 78},
  {-35, -42, 77},
  {-33, -43, 76},
  {-32, -43, 76},
  {-30, -44, 75},
  {-29, -45, 74},
  {-27, -46, 73},
  {-26, -46, 73},
  {-25, -47, 72},
  {-23, -48, 71},
  {-22, -48, 71},
  {-21, -49, 70},
  {-19, -50, 69},
  {-18, -50, 69},
  {-17, -51, 68},
  {-15, -52, 67},
  {-14, -52, 67},
  {-13, -53, 66},
  {-12, -53, 66},
  {-10, -54, 65},
  {-9, -55, 64},
  {-8, -55, 64},
  {-7, -56, 63},
  {-6, -56, 63},
  {-4, -57, 62},
  {-3, -58, 61},
  {-2, -58, 61},
  {-1, -59, 60},
  {0, -59, 60},
  {1, -60, 59},
  {2, -61, 58},
  {3, -61, 58},
  {4, -62, 57},
  {6, -63, 56},
  {7, -63, 56},
  {8, -64, 55},
  {9, -64, 55},
  {10, -65, 54},
  {12, -66, 53},
  {13, -66, 53},
  {14, -67, 52},
  {15, -67, 52},
  {17, -68, 51},
  {18, -69, 50},
  {19, -69, 50},
  {21, -70, 49},
  {22, -71, 48},
  {23, -71, 48},
  {25, -72, 47},
  {26, -73, 46},
  {27, -73, 46},
  {29, -74, 45},
  {30, -75, 44},
  {32, -76, 43},
  {33, -76, 43},
  {35, -77, 42},
  {36, -78, 41},
  {38, -79, 40},
  {40, -80, 39},
  {40, -79, 38},
  {41, -78, 36},
  {42, -77, 35},
  {43, -76, 33},
  {43, -76, 32},
  {44, -75, 30},
  {45, -74, 29},
  {46, -73, 27},
  {46, -73, 26},
  {47, -72, 25},
  {48, -71, 23},
  {48, -71, 22},
  {49, -70, 21},
  {50, -69, 19},
  {50, -69, 18},
  {51, -68, 17},
  {52, -67, 15},
  {52, -67, 14},
  {53, -66, 13},
  {53, -66, 12},
  {54, -65, 10},
  {55, -64, 9},
  {55, -64, 8},
  {56, -63, 7},
  {56, -63, 6},
  {57, -62, 4},
  {58, -61, 3},
  {58, -61, 2},
  {59, -60, 1},
  {59, -60, 0},
  {60, -59, -1},
  {61, -58, -2},
  {61, -58, -3},
  {62, -57, -4},
  {63, -56, -6},
  {63, -56, -7},
  {64, -55, -8},
  {64, -55, -9},
  {65, -54, -10},
  {66, -53, -12},
  {66, -53, -13},
  {67, -52, -14},
  {67, -52, -15},
  {68, -51, -17},
  {69, -50, -18},
  {69, -50, -19},
  {70, -49, -21},
  {71, -48, -22},
  {71, -48, -23},
  {72, -47, -25},
  {73, -46, -26},
  {73, -46, -27},
  {74, -45, -29},
  {75, -44, -30},
  {76, -43, -32},
  {76, -43, -33},
  {77, -42, -35},
  {78, -41, -36},
  {79, -40, -38},
  {80, -40, -40}
};

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
    else if (active_pair == 4u && now-encoder_tick > 100u) cut_output(4u);
  }
}

void board_check_encoder(uint32_t angle, uint8_t ok)
{
  if (ok && active_pair == 4u && vector_direction >= 4u) {
    if (ident_have) {
      int32_t change = (int32_t)angle - (int32_t)ident_previous;
      if (change > 18000) change -= 36000;
      if (change < -18000) change += 36000;
      ident_travel += change;
      ident_last_delta = change;
    }
    ident_previous = angle;
    ident_have = 1u;
    ident_points++;
    if (!ident_started && HAL_GetTick()-vector_start >= IDENT_ALIGN_MS) {
      ident_base = ident_travel;
      ident_base_angle = angle;
      ident_started = 1u;
    }
  }
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
  if (vector_direction == 0u) {
    HRTIM1->sTimerxRegs[0].CMP1xR = 4000u - delta;
    HRTIM1->sTimerxRegs[1].CMP1xR = 4000u + delta / 2u;
    HRTIM1->sTimerxRegs[2].CMP1xR = 4000u + delta / 2u;
  } else if (vector_direction >= 2u) {
    uint32_t degree;
    if (vector_direction >= 4u) {
      degree = elapsed <= IDENT_ALIGN_MS ? 0u :
          (elapsed-IDENT_ALIGN_MS)*360u/IDENT_MOVE_MS;
      if (degree > 360u) degree = 360u;
      if (vector_direction == 5u) degree = 360u-degree;
    } else {
      degree = elapsed <= SWEEP_RAMP_MS ? 0u :
          (elapsed-SWEEP_RAMP_MS)*90u/SWEEP_MOVE_MS;
      if (degree > 90u) degree = 90u;
      if (vector_direction == 3u) degree = 90u-degree;
    }
    uint32_t ramp = elapsed < SWEEP_RAMP_MS ? elapsed : SWEEP_RAMP_MS;
    for (uint32_t phase = 0u; phase < 3u; phase++) {
      int32_t ticks = (int32_t)sweep_phase[degree][phase] * (int32_t)ramp / (int32_t)SWEEP_RAMP_MS;
      HRTIM1->sTimerxRegs[phase].CMP1xR = (uint32_t)(4000 - ticks);
    }
  } else {
    /* Pure beta: A=0, B=+3/4 delta, C=-3/4 delta.
     * Same 120-tick max line difference as a; vector magnitude is sqrt(3)/2
     * of a, deliberately conservative rather than increasing line voltage. */
    uint32_t beta = delta * 3u / 4u;
    HRTIM1->sTimerxRegs[0].CMP1xR = 4000u;
    HRTIM1->sTimerxRegs[1].CMP1xR = 4000u - beta;
    HRTIM1->sTimerxRegs[2].CMP1xR = 4000u + beta;
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

static char vector_label(uint8_t direction)
{
  static const char labels[] = "abgjnm";
  return direction < 6u ? labels[direction] : '?';
}

static void start_vector(uint8_t direction)
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
  vector_direction = direction;
  vector_samples = peak_counts = 0u;
  angle_start = encoder_angle;
  vector_start = adc_tick = HAL_GetTick();
  ident_travel = ident_base = ident_last_delta = 0;
  ident_points = ident_base_angle = 0u;
  ident_have = ident_started = 0u;
  uint32_t duration = direction >= 4u ? IDENT_TOTAL_MS : (direction >= 2u ? SWEEP_MS : VECTOR_MS);
  deadline = vector_start + duration;
  next_allowed = vector_start + duration + (direction >= 4u ? 5000u : 2000u);
  active_pair = 4u;
  HRTIM1->sCommonRegs.OENR = 0x3Fu;
  __set_PRIMASK(primask);
  char line[192];
  (void)snprintf(line, sizeof(line),
      "# VECTOR_BEGIN R9 dir=%c e_start=%u e_end=%u TRG2; duration_ms=%lu; ramp=50ms; max_line_duty=1.5percent (~0.18V at 12V); x=stop\r\n",
      vector_label(direction),
      (direction == 5u ? 360u : ((direction == 1u || direction == 3u) ? 90u : 0u)),
      (direction == 4u ? 360u : ((direction == 1u || direction == 2u) ? 90u : 0u)),
      (unsigned long)duration);
  send_text(line);
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
  send_text("# DIAG_BEGIN R9 PWM_OFF; route=TA_CMP2->HRTIM_TRG2->ADC1; stages=POLL,IRQ,SW\r\n");
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
  send_text("# STEP2B_R9 POLE_DIRECTION_SURVEY; PWM OFF at boot\r\n"
            "# PA0/PA1 current ADC; 5mOhm x20; nominal VDDA=3.3V\r\n"
            "# z=zero; f=FireWater; t=text; x=stop; c=clear check_fault\r\n"
            "# d=PWM-off diagnosis; a/b=fixed 100ms; g/j=90deg 400ms; n/m=360deg survey 2500ms; u/v/w disabled; current trip ~0.8A\r\n");
  start_zero();
}

void board_check_poll(void)
{
  if (expired != 0u) {
    uint8_t reason = expired;
    stop_pwm();
    char line[256];
    (void)snprintf(line, sizeof(line),
        "# VECTOR_END reason=%u (1=timeout 2=current 3=ADC_stale 4=encoder); samples=%lu peak_mA=%lu angle_start=%lu.%02lu angle_now=%lu.%02lu dir=%c\r\n",
        reason, (unsigned long)vector_samples, (unsigned long)(peak_counts*33000u/4095u),
        (unsigned long)(angle_start/100u), (unsigned long)(angle_start%100u),
        (unsigned long)(encoder_angle/100u), (unsigned long)(encoder_angle%100u),
        vector_label(vector_direction));
    send_text(line);
    if (vector_direction >= 4u) {
      int32_t travel = ident_travel-ident_base;
      uint32_t magnitude = travel < 0 ? (uint32_t)-travel : (uint32_t)travel;
      uint32_t ratio100 = (ident_started && magnitude) ? 3600000u/magnitude : 0u;
      uint32_t nearest = (ratio100+50u)/100u;
      (void)snprintf(line, sizeof(line),
          "# IDENT dir=%c reason=%u base_deg=%lu.%02lu travel_cd=%ld points=%lu last_delta_cd=%ld ratio=%lu.%02lu pole_candidate=%lu; TENTATIVE NOT_SAVED\r\n",
          vector_label(vector_direction), reason,
          (unsigned long)(ident_base_angle/100u), (unsigned long)(ident_base_angle%100u),
          (long)travel, (unsigned long)ident_points, (long)ident_last_delta,
          (unsigned long)(ratio100/100u), (unsigned long)(ratio100%100u), (unsigned long)nearest);
      send_text(line);
    }
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
                "# d=PWM-off diagnosis; a/b=fixed 100ms; g/j=90deg 400ms; n/m=360deg survey 2500ms; 2s cooldown; u/v/w disabled\r\n");
      break;
    case 'x': stop_pwm(); send_text("# STOP all bridge inputs low\r\n"); break;
    case 'z': start_zero(); break;
    case 'f': chart_mode = 1u;
      send_text("# CH: angle_deg,UA_mV,VA_mV,IU_mA,IV_mA,valid,zero_ok,pwm_pair,check_fault\r\n"); break;
    case 't': chart_mode = 0u; break;
    case 'a': start_vector(0u); break;
    case 'b': start_vector(1u); break;
    case 'g': start_vector(2u); break;
    case 'j': start_vector(3u); break;
    case 'n': start_vector(4u); break;
    case 'm': start_vector(5u); break;
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
