/* Bench motor tests with PWM-synchronised injected ADC, software current trip and timeout. */
#include "board_check.h"
#include "adc.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define ZERO_SAMPLES 256u
#define VECTOR_MS 100u
#define SWEEP_MS 400u
#define SWEEP_RAMP_MS 50u
#define SWEEP_MOVE_MS 300u
#define IDENT_ALIGN_MS 250u
#define IDENT_MOVE_MS 2000u
#define IDENT_TOTAL_MS 2500u
#define ALIGN_TOTAL_MS 700u
#define ALIGN_WINDOW_MS 200u
#define ALIGN_POLE_PAIRS 7
#define ALIGN_ENCODER_SIGN (-1)
#define APPROACH_TOTAL_MS 3700u
#define APPROACH_MAX_DIFF_CD 500 /* 5 electrical degrees: provisional repeatability screen */
#define LOCAL_TOTAL_MS 2700u
#define LOCAL_HALF_E_DEG 25
#define LOCAL_MIN_CD 1000u
#define LOCAL_MAX_CD 3000u
#define ALIGN_MAX_SPAN_CD 20 /* mechanical 0.20 degrees */
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
static int32_t align_sum, align_min, align_max;
static uint32_t align_anchor, align_count, align_first_tick, align_last_tick;
static uint8_t align_previous_valid;
static int32_t align_previous_offset;
static uint32_t local_min_angle, local_max_angle, local_points;
static int32_t local_zero_sum[2];
static uint32_t local_zero_count[2];
static volatile int32_t local_command_deg;
static volatile uint8_t local_stage;
static uint32_t approach_anchor[4], approach_count[4], approach_first[4], approach_last[4];
static int32_t approach_sum[4], approach_min[4], approach_max[4];
static volatile uint32_t current_check_count;
static volatile int32_t current_check_sum[2];
static volatile int32_t current_check_min[2], current_check_max[2];
static uint8_t map_next_step;
static volatile uint32_t map_degree;
/* Current checks in the stator frame and short torque pulses in the rotor frame. */
#define TRACK_ALPHA 12u
#define TRACK_BETA  13u
#define TRACK_APPROACH 14u
#define TORQUE_POSITIVE 15u
#define TORQUE_NEGATIVE 16u
#define TORQUE_ALIGN_MS 850u
#define TORQUE_PULSE_MS 500u
#define TORQUE_RAMP_MS 80u
#define TORQUE_STATS_MS 100u
#define TORQUE_TARGET_MA 100
#define TORQUE_ENCODER_TIMEOUT_MS 8u
#define TORQUE_START_MIN_CD 2000
#define TORQUE_START_MAX_CD 2200
#define TORQUE_TRAVEL_LIMIT_CD 300
#define TORQUE_ALIGN_REJECTED 10u
#define TORQUE_D_MAX_TICKS 48
#define TORQUE_Q_MAX_TICKS 34
static uint8_t is_torque(uint8_t mode) { return mode == TORQUE_POSITIVE || mode == TORQUE_NEGATIVE; }

/* Encoder updates run in foreground. Only this test uses the rotor angle in ADC IRQ. */
static struct {
  volatile uint32_t angle, tick, align_count;
  volatile int32_t align_sum, align_min, align_max;
  uint32_t anchor;
  volatile uint32_t count, limited, early_count, early_limited;
  volatile int32_t sum_d, sum_q, early_sum_d, early_sum_q;
  volatile int32_t min_travel, max_travel, end_travel;
  volatile uint8_t aligned;
} torque;
static uint8_t is_track(uint8_t mode) { return mode == TRACK_ALPHA || mode == TRACK_BETA; }
#define TRACK_TARGET_MA 200
#define TRACK_RAMP_MS 100u
#define TRACK_BLOCK 20u /* 20 synchronous samples per PI update, nominal 1 ms */
static struct {
  uint32_t sub, n, limited, wrong_blocks;
  int32_t adc_sum[2], integral[2], sum[2], lo[2], hi[2];
} track;
/* Voltage q keeps its original timing. Current 3 holds each departure at full
 * reference for 700 ms. Use the same endpoints for control and diagnostics. */
#define CA_DEPART_HOLD_MS 700u
#define CA_SWEEP_MS 600u
#define CA_ZERO_HOLD_MS 700u
#define CA_NEG_END_MS (TRACK_RAMP_MS + CA_DEPART_HOLD_MS)
#define CA_ZERO1_END_MS (CA_NEG_END_MS + CA_SWEEP_MS + CA_ZERO_HOLD_MS)
#define CA_POS_START_MS (CA_ZERO1_END_MS + CA_SWEEP_MS)
#define CA_POS_END_MS (CA_POS_START_MS + CA_DEPART_HOLD_MS)
#define CA_TOTAL_MS (CA_POS_END_MS + CA_SWEEP_MS + CA_ZERO_HOLD_MS)
/* Sample the last 40 ms before the outward sweep through the +45 hold.
 * Capture only in foreground encoder polling, no added UART during drive. */
#define TRACE_BEGIN_MS (CA_ZERO1_END_MS-40u)
#define TRACE_END_MS CA_POS_END_MS
static struct {
  uint16_t ms, cd, current_age_ms;
  int16_t ia_mA, ib_mA;
} trace_pt[80];
static volatile int32_t trace_current_a, trace_current_b;
static volatile uint32_t trace_current_ms;
static uint32_t trace_n;
static uint8_t trace_overflow;
/* Windows 0/1: zero holds, 2/3: late departure, 4/5: early departure (current only). */
static struct { uint32_t n,limited,bad; int32_t sum_a,sum_b; } approach_i[6];
static const uint16_t window_end_ms[6]={CA_ZERO1_END_MS,CA_TOTAL_MS,CA_NEG_END_MS,CA_POS_END_MS,
                                        TRACK_RAMP_MS+100u,CA_POS_START_MS+100u};
static int32_t approach_window(uint32_t ms) {
  for (uint32_t i=0;i<6;i++) {
    uint32_t end=window_end_ms[i], width=i<2u ? 200u : 100u;
    if (ms>=end-width && ms<end) return (int32_t)i;
  }
  return -1;
}
/* Unit vector scaled by TRACK_TARGET_MA, degrees 0..45. */
static const int16_t approach_ref[46][2]={
  {200,0},{200,3},{200,7},{200,10},{200,14},{199,17},{199,21},{199,24},
  {198,28},{198,31},{197,35},{196,38},{196,42},{195,45},{194,48},{193,52},
  {192,55},{191,58},{190,62},{189,65},{188,68},{187,72},{185,75},{184,78},
  {183,81},{181,85},{180,88},{178,91},{177,94},{175,97},{173,100},{171,103},
  {170,106},{168,109},{166,112},{164,115},{162,118},{160,120},{158,123},{155,126},
  {153,129},{151,131},{149,134},{146,136},{144,139},{141,141}
};
static uint8_t encoder_valid;
static volatile uint8_t vector_direction; /* 0:a 1:b 2:g 3:j 4:n full-forward 5:m full-reverse 6:l zero alignment 7:k local scan 8:i alpha check 9:y beta check */
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
    else if (active_pair == 4u) {
      if (is_torque(vector_direction)) {
        if (now-vector_start >= TORQUE_ENCODER_TIMEOUT_MS && now-torque.tick >= TORQUE_ENCODER_TIMEOUT_MS)
          cut_output(4u);
      } else if (now-encoder_tick > 100u) cut_output(4u);
    }
  }
}

/* Degree values below are centidegrees, wrapped to [0,36000). */
static int32_t wrap_angle_cd(int32_t angle)
{
  angle %= 36000;
  return angle < 0 ? angle+36000 : angle;
}

static int32_t signed_angle_cd(int32_t angle)
{
  angle = wrap_angle_cd(angle);
  return angle > 18000 ? angle-36000 : angle;
}

/* One bounded triangle: 250ms settle, 1s outward, 200ms hold,
 * 1s return, 250ms hold. Stage numbers are exposed in telemetry. */
static int32_t local_target(uint32_t elapsed, uint8_t *stage)
{
  if (elapsed < 250u) { *stage = 1u; return -LOCAL_HALF_E_DEG; }
  if (elapsed < 1250u) {
    *stage = 2u;
    return -LOCAL_HALF_E_DEG + (int32_t)((elapsed-250u)*50u/1000u);
  }
  if (elapsed < 1450u) { *stage = 3u; return LOCAL_HALF_E_DEG; }
  if (elapsed < 2450u) {
    *stage = 4u;
    return LOCAL_HALF_E_DEG - (int32_t)((elapsed-1450u)*50u/1000u);
  }
  *stage = 5u; return -LOCAL_HALF_E_DEG;
}

/* Sweep into field=0 from opposite electrical sides; hold each for 700ms. */
static int32_t approach_target(uint32_t elapsed, uint8_t *stage, uint8_t current_test)
{
  uint32_t neg_end = current_test ? CA_NEG_END_MS : 300u;
  uint32_t zero1_end = current_test ? CA_ZERO1_END_MS : 1600u;
  uint32_t pos_end = current_test ? CA_POS_END_MS : 2400u;
  if (elapsed < neg_end) { *stage=1u; return -45; }
  if (elapsed < neg_end+CA_SWEEP_MS) { *stage=2u; return -45+(int32_t)((elapsed-neg_end)*45u/CA_SWEEP_MS); }
  if (elapsed < zero1_end) { *stage=3u; return 0; }
  if (elapsed < zero1_end+CA_SWEEP_MS) { *stage=4u; return (int32_t)((elapsed-zero1_end)*45u/CA_SWEEP_MS); }
  if (elapsed < pos_end) { *stage=5u; return 45; }
  if (elapsed < pos_end+CA_SWEEP_MS) { *stage=6u; return 45-(int32_t)((elapsed-pos_end)*45u/CA_SWEEP_MS); }
  *stage=7u; return 0;
}

uint8_t board_check_torque_active(void)
{
  return active_pair == 4u && is_torque(vector_direction);
}

void board_check_encoder(uint32_t angle, uint8_t ok)
{
  if (ok && active_pair == 4u && is_torque(vector_direction)) {
    uint32_t primask=__get_PRIMASK();
    __disable_irq();
    if (active_pair == 4u) {
      uint32_t now=HAL_GetTick(), elapsed=now-vector_start;
      torque.angle=angle;
      torque.tick=now;
      if (elapsed >= TORQUE_ALIGN_MS-ALIGN_WINDOW_MS && elapsed < TORQUE_ALIGN_MS) {
        int32_t pos=(int32_t)angle;
        if (!torque.align_count || pos<torque.align_min) torque.align_min=pos;
        if (!torque.align_count || pos>torque.align_max) torque.align_max=pos;
        torque.align_sum+=pos;
        torque.align_count++;
      }
    }
    __set_PRIMASK(primask);
  }
  if (ok && active_pair == 4u && (vector_direction == 10u || vector_direction == TRACK_APPROACH)) {
    uint32_t elapsed = HAL_GetTick()-vector_start;
    if (vector_direction==TRACK_APPROACH && elapsed>=TRACE_BEGIN_MS && elapsed<TRACE_END_MS) {
      /* Keep the two currents and their timestamp from the same IRQ update. */
      uint32_t primask=__get_PRIMASK();
      __disable_irq();
      uint32_t capture_ms=HAL_GetTick()-vector_start;
      uint32_t current_ms=trace_current_ms;
      int32_t ia=trace_current_a, ib=trace_current_b;
      __set_PRIMASK(primask);
      if (capture_ms<TRACE_END_MS && (!trace_n || capture_ms!=trace_pt[trace_n-1u].ms)) {
        if (trace_n<sizeof(trace_pt)/sizeof(trace_pt[0])) {
          trace_pt[trace_n].ms=(uint16_t)capture_ms;
          trace_pt[trace_n].cd=(uint16_t)angle;
          trace_pt[trace_n].ia_mA=(int16_t)ia;
          trace_pt[trace_n].ib_mA=(int16_t)ib;
          trace_pt[trace_n].current_age_ms=(uint16_t)(capture_ms-current_ms);
          trace_n++;
        } else trace_overflow=1u;
      }
    }
    int32_t index = vector_direction==TRACK_APPROACH ? approach_window(elapsed) :
        (elapsed>=1400u && elapsed<1600u ? 0 : (elapsed>=3500u && elapsed<3700u ? 1 : -1));
    if (index >= 0 && index < 4) {
      if (!approach_count[index]) {
        approach_anchor[index]=angle; approach_first[index]=HAL_GetTick();
      }
      int32_t displacement=signed_angle_cd((int32_t)angle-(int32_t)approach_anchor[index]);
      if (!approach_count[index] || displacement<approach_min[index]) approach_min[index]=displacement;
      if (!approach_count[index] || displacement>approach_max[index]) approach_max[index]=displacement;
      approach_sum[index]+=displacement;
      approach_count[index]++;
      approach_last[index]=HAL_GetTick();
    }
  }
  if (ok && active_pair == 4u && vector_direction == 7u) {
    if (angle <= LOCAL_MIN_CD || angle >= LOCAL_MAX_CD) {
      cut_output(8u); /* Stop on measured mechanical range boundary. */
    }
    if (!local_points || angle < local_min_angle) local_min_angle = angle;
    if (!local_points || angle > local_max_angle) local_max_angle = angle;
    local_points++;
    uint8_t stage;
    int32_t command = local_target(HAL_GetTick()-vector_start, &stage);
    if (active_pair == 4u && command >= -1 && command <= 1 && (stage == 2u || stage == 4u)) {
      uint32_t index = stage == 2u ? 0u : 1u;
      local_zero_sum[index] += (int32_t)angle;
      local_zero_count[index]++;
    }
  }
  if (ok && active_pair == 4u && (vector_direction == 6u || vector_direction == 8u || vector_direction == 9u || vector_direction == 11u) &&
      HAL_GetTick()-vector_start >= ALIGN_TOTAL_MS-ALIGN_WINDOW_MS) {
    if (!align_count) {
      align_anchor = angle;
      align_first_tick = HAL_GetTick();
    }
    int32_t displacement = signed_angle_cd((int32_t)angle-(int32_t)align_anchor);
    if (!align_count || displacement < align_min) align_min = displacement;
    if (!align_count || displacement > align_max) align_max = displacement;
    align_sum += displacement;
    align_count++;
    align_last_tick = HAL_GetTick();
  }
  if (ok && active_pair == 4u && (vector_direction == 4u || vector_direction == 5u)) {
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

static int32_t bounded(int32_t x, int32_t lo, int32_t hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}

/* PI in 1/1024 timer ticks. Conditional integration, also bound stored state.
 * Kp=32/1024 tick/mA, Ki=2/1024 tick/(mA*update). */
static int32_t track_pi(uint32_t axis, int32_t err, int32_t lo, int32_t hi, uint8_t *limited)
{
  int32_t old = track.integral[axis];
  int32_t trial = old + 2*err, prop = 32*err;
  if (!((trial+prop > hi*1024 && err > 0) || (trial+prop < lo*1024 && err < 0)))
    track.integral[axis] = bounded(trial, lo*1024, hi*1024);
  int32_t out = track.integral[axis] + prop;
  if (trial+prop >= hi*1024 || trial+prop <= lo*1024) *limited = 1u;
  return bounded(out, lo*1024, hi*1024)/1024;
}

/* Inputs are corrected zero-minus-ADC counts. Raw per-sample trip is upstream. */
static uint8_t read_track_current(int32_t u, int32_t v, int32_t *a, int32_t *b)
{
  track.adc_sum[0]+=u;
  track.adc_sum[1]+=v;
  if (++track.sub < TRACK_BLOCK) return 0u;
  *a=track.adc_sum[0]*33000/(4095*(int32_t)TRACK_BLOCK);
  int32_t phase_v=track.adc_sum[1]*33000/(4095*(int32_t)TRACK_BLOCK);
  *b=(*a+2*phase_v)*591/1024; /* 1/sqrt(3) */
  track.sub=0u;
  track.adc_sum[0]=track.adc_sum[1]=0;
  return 1u;
}

static void track_sample(uint32_t elapsed, int32_t u, int32_t v)
{
  int32_t a,b;
  if (!read_track_current(u,v,&a,&b)) return;
  uint8_t beta_test = vector_direction == TRACK_BETA;
  uint8_t sweep = vector_direction == TRACK_APPROACH;
  int32_t da=beta_test ? 0 : TRACK_TARGET_MA, db=beta_test ? TRACK_TARGET_MA : 0;
  if (sweep) {
    uint8_t stage;
    int32_t deg=approach_target(elapsed,&stage,1u), idx=deg<0 ? -deg : deg;
    da=approach_ref[idx][0]; db=deg<0 ? -approach_ref[idx][1] : approach_ref[idx][1];
    local_command_deg=deg; local_stage=stage;
  }
  if (elapsed >= TRACK_RAMP_MS && a*da+b*db < -60*TRACK_TARGET_MA) {
    if (++track.wrong_blocks >= 25u) { cut_output(9u); return; }
  } else track.wrong_blocks = 0u;
  int32_t ref = elapsed < TRACK_RAMP_MS ? TRACK_TARGET_MA*(int32_t)elapsed/(int32_t)TRACK_RAMP_MS : TRACK_TARGET_MA;
  uint8_t limited = 0u;
  int32_t va = track_pi(0u,da*ref/TRACK_TARGET_MA-a,beta_test ? -12 : 0,beta_test ? 12 : 64,&limited);
  /* Sweep: leave enough room for beta after the alpha PI update so that
   * |B-C| and |A-B|,|A-C| all remain within the R18 106-tick limit. */
  int32_t room = sweep ? bounded((106-va-va/2)*1024/887,0,62) : 12;
  int32_t vb = track_pi(1u,db*ref/TRACK_TARGET_MA-b,beta_test ? 0 : -room,beta_test ? 62 : room,&limited);
  /* Both rectangles bound line spread to 106 ticks. On beta the largest
   * difference is B-C: 2*trunc(62*887/1024) = 106. */
  int32_t side = vb*887/1024;
  HRTIM1->sTimerxRegs[0].CMP1xR = (uint32_t)(4000-va);
  HRTIM1->sTimerxRegs[1].CMP1xR = (uint32_t)(4000+va/2-side);
  HRTIM1->sTimerxRegs[2].CMP1xR = (uint32_t)(4000+va/2+side);
  if (sweep) {
    trace_current_a=a;
    trace_current_b=b;
    trace_current_ms=elapsed;
    int32_t i=approach_window(elapsed);
    if (i>=0) {
      approach_i[i].n++; approach_i[i].limited+=limited;
      approach_i[i].sum_a+=a; approach_i[i].sum_b+=b;
      if (a<da-60 || a>da+60 || b<db-60 || b>db+60) approach_i[i].bad++;
    }
    return;
  }
  uint32_t align_ms=is_torque(vector_direction) ? TORQUE_ALIGN_MS : ALIGN_TOTAL_MS;
  if (elapsed >= align_ms-ALIGN_WINDOW_MS) {
    int32_t m[2] = {a,b};
    for (uint32_t i=0u;i<2u;i++) {
      if (!track.n || m[i]<track.lo[i]) track.lo[i]=m[i];
      if (!track.n || m[i]>track.hi[i]) track.hi[i]=m[i];
      track.sum[i]+=m[i];
    }
    track.n++; track.limited+=limited;
  }
}

static void torque_sample(uint32_t elapsed, int32_t u, int32_t v)
{
  uint32_t pos=torque.angle;
  if (elapsed < TORQUE_ALIGN_MS) {
    int32_t movement=signed_angle_cd((int32_t)pos-(int32_t)angle_start);
    if (movement < -100 || movement > 100) {
      cut_output(8u);
      return;
    }
    track_sample(elapsed,u,v);
    return;
  }

  if (!torque.aligned) {
    uint32_t n=torque.align_count, current_n=track.n;
    int32_t mean_a=current_n ? track.sum[0]/(int32_t)current_n : 0;
    int32_t mean_b=current_n ? track.sum[1]/(int32_t)current_n : 0;
    int32_t anchor=n ? torque.align_sum/(int32_t)n : 0;
    if (n<80u || torque.align_max-torque.align_min>ALIGN_MAX_SPAN_CD ||
        anchor<TORQUE_START_MIN_CD || anchor>TORQUE_START_MAX_CD ||
        pos<(uint32_t)(anchor-ALIGN_MAX_SPAN_CD) || pos>(uint32_t)(anchor+ALIGN_MAX_SPAN_CD) ||
        current_n<190u || current_n>210u || track.limited ||
        mean_a<170 || mean_a>230 || mean_b< -30 || mean_b>30 ||
        track.lo[0]<140 || track.hi[0]>260 || track.lo[1]< -60 || track.hi[1]>60) {
      cut_output(TORQUE_ALIGN_REJECTED);
      return;
    }
    torque.anchor=(uint32_t)anchor;
    track.sub=0u;
    track.adc_sum[0]=track.adc_sum[1]=0;
    track.integral[0]=bounded(track.integral[0],-TORQUE_D_MAX_TICKS*1024,TORQUE_D_MAX_TICKS*1024);
    track.integral[1]=bounded(track.integral[1],-TORQUE_Q_MAX_TICKS*1024,TORQUE_Q_MAX_TICKS*1024);
    torque.aligned=1u;
  }

  int32_t travel=signed_angle_cd((int32_t)pos-(int32_t)torque.anchor);
  torque.end_travel=travel;
  if (travel<torque.min_travel) torque.min_travel=travel;
  if (travel>torque.max_travel) torque.max_travel=travel;
  if (travel<=-TORQUE_TRAVEL_LIMIT_CD || travel>=TORQUE_TRAVEL_LIMIT_CD) {
    cut_output(8u);
    return;
  }
  int32_t a,b;
  if (!read_track_current(u,v,&a,&b)) return;

  /* The travel guard bounds |theta_e| to 21 degrees. Reuse the sine/cosine table. */
  int32_t theta_e=ALIGN_ENCODER_SIGN*ALIGN_POLE_PAIRS*travel;
  uint32_t deg=(uint32_t)((theta_e<0 ? -theta_e : theta_e)+50)/100u;
  int32_t c=approach_ref[deg][0], sn=approach_ref[deg][1];
  if (theta_e<0) sn=-sn;
  int32_t id=(c*a+sn*b)/TRACK_TARGET_MA;
  int32_t iq=(c*b-sn*a)/TRACK_TARGET_MA;
  int32_t q_target=vector_direction==TORQUE_POSITIVE ? TORQUE_TARGET_MA : -TORQUE_TARGET_MA;
  uint32_t pulse_ms=elapsed-TORQUE_ALIGN_MS;
  int32_t ramp_ms=(int32_t)(pulse_ms<TORQUE_RAMP_MS ? pulse_ms : TORQUE_RAMP_MS);
  int32_t id_ref=TRACK_TARGET_MA*((int32_t)TORQUE_RAMP_MS-ramp_ms)/(int32_t)TORQUE_RAMP_MS;
  int32_t iq_ref=q_target*ramp_ms/(int32_t)TORQUE_RAMP_MS;
  if (pulse_ms>=TORQUE_RAMP_MS && (q_target>0 ? iq : -iq)< -40) {
    if (++track.wrong_blocks>=25u) { cut_output(9u); return; }
  } else track.wrong_blocks=0u;

  uint8_t limited=0u;
  int32_t vd=track_pi(0u,id_ref-id,-TORQUE_D_MAX_TICKS,TORQUE_D_MAX_TICKS,&limited);
  int32_t vq=track_pi(1u,iq_ref-iq,-TORQUE_Q_MAX_TICKS,TORQUE_Q_MAX_TICKS,&limited);
  /* Inverse Park and Clarke. The d/q bounds keep phase spread below 106 ticks. */
  int32_t va=(c*vd-sn*vq)/TRACK_TARGET_MA;
  int32_t vb=(sn*vd+c*vq)/TRACK_TARGET_MA;
  int32_t side=vb*887/1024;
  HRTIM1->sTimerxRegs[0].CMP1xR=(uint32_t)(4000-va);
  HRTIM1->sTimerxRegs[1].CMP1xR=(uint32_t)(4000+va/2-side);
  HRTIM1->sTimerxRegs[2].CMP1xR=(uint32_t)(4000+va/2+side);
  /* Compare the first 100 ms at full reference with the final 100 ms. */
  if (pulse_ms>=TORQUE_RAMP_MS && pulse_ms<TORQUE_RAMP_MS+TORQUE_STATS_MS) {
    torque.early_count++;
    torque.early_limited+=limited;
    torque.early_sum_d+=id;
    torque.early_sum_q+=iq;
  }
  if (pulse_ms>=TORQUE_PULSE_MS-TORQUE_STATS_MS && pulse_ms<TORQUE_PULSE_MS) {
    torque.count++;
    torque.limited+=limited;
    torque.sum_d+=id;
    torque.sum_q+=iq;
  }
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
  if (is_torque(vector_direction)) { torque_sample(elapsed,-iu,-iv); return; }
  if (is_track(vector_direction) || vector_direction==TRACK_APPROACH) { track_sample(elapsed, -iu, -iv); return; }
  if ((vector_direction == 8u || vector_direction == 9u || vector_direction == 11u) && elapsed >= 500u) {
    int32_t values[2] = {-iu, -iv}; /* Display convention: zero minus ADC. */
    for (uint32_t i = 0u; i < 2u; i++) {
      current_check_sum[i] += values[i];
      if (!current_check_count || values[i] < current_check_min[i]) current_check_min[i] = values[i];
      if (!current_check_count || values[i] > current_check_max[i]) current_check_max[i] = values[i];
    }
    current_check_count++;
  }
  uint32_t delta = elapsed < 50u ? elapsed * 80u / 50u : 80u;
  if (vector_direction == 0u || vector_direction == 6u || vector_direction == 8u) {
    HRTIM1->sTimerxRegs[0].CMP1xR = 4000u - delta;
    HRTIM1->sTimerxRegs[1].CMP1xR = 4000u + delta / 2u;
    HRTIM1->sTimerxRegs[2].CMP1xR = 4000u + delta / 2u;
  } else if ((vector_direction >= 2u && vector_direction <= 7u) || vector_direction == 10u || vector_direction == 11u) {
    uint32_t degree;
    if (vector_direction == 11u) {
      degree = map_degree;
      local_command_deg = (int32_t)degree;
      local_stage = elapsed < 50u ? 1u : (elapsed < 500u ? 2u : 3u);
    } else if (vector_direction == 7u || vector_direction == 10u) {
      uint8_t stage;
      int32_t target = vector_direction == 10u ? approach_target(elapsed, &stage,0u) : local_target(elapsed, &stage);
      local_command_deg = target;
      local_stage = stage;
      degree = target < 0 ? (uint32_t)(360+target) : (uint32_t)target;
    } else if (vector_direction >= 4u) {
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
    /* DTPRSC=0 => MUL8: tDT=tHRTIM/8. At 160MHz, 256 ticks=200ns.
     * MCU output deadtime; actual DRV8300 gate deadtime needs measurement. */
    t->DTxR = 256u | (256u << 16);
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
  static const char labels[] = "abgjnmlkiyqho2356";
  return direction < sizeof(labels)-1u ? labels[direction] : '?';
}

static void start_vector(uint8_t direction)
{
  uint32_t now = HAL_GetTick();
  if (is_torque(direction) && (encoder_angle<TORQUE_START_MIN_CD || encoder_angle>TORQUE_START_MAX_CD)) {
    send_text("# FOC_REJECTED start_mech_deg=20..22; check s; PWM_OFF\r\n");
    return;
  }
  if (direction == 7u && (encoder_angle < 1200u || encoder_angle > 2800u)) {
    send_text("# LOCAL_REJECTED: start mechanical angle must be 12..28deg; recommended 17..22deg; PWM_OFF\r\n");
    return;
  }
  if (active_pair || expired || collecting || !zero_valid || !sample_valid ||
      check_fault || !encoder_valid || now-encoder_tick > 100u ||
      (next_allowed && (int32_t)(now-next_allowed) < 0)) {
    send_text("# VECTOR_REJECTED: need zero_ok, ADC, encoder, no fault; idle + cooldown (long tests: 5s)\r\n");
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
  current_check_count = 0u;
  for (uint32_t i = 0u; i < 2u; i++) {
    current_check_sum[i] = current_check_min[i] = current_check_max[i] = 0;
  }
  for (uint32_t i=0u;i<4u;i++) {
    approach_count[i]=approach_anchor[i]=approach_first[i]=approach_last[i]=0u;
    approach_sum[i]=approach_min[i]=approach_max[i]=0;
  }
  if (is_track(direction) || direction==TRACK_APPROACH || is_torque(direction)) memset(&track, 0, sizeof(track));
  if (is_torque(direction)) {
    memset(&torque,0,sizeof(torque));
    torque.angle=encoder_angle;
    torque.tick=encoder_tick;
  }
  if (direction==TRACK_APPROACH) {
    memset(approach_i,0,sizeof(approach_i));
    trace_n=0u; trace_overflow=0u;
    trace_current_a=trace_current_b=0; trace_current_ms=0u;
  }
  vector_direction = direction;
  if (direction == 11u) map_degree = (uint32_t)map_next_step * 30u;
  vector_samples = peak_counts = 0u;
  angle_start = encoder_angle;
  vector_start = adc_tick = HAL_GetTick();
  ident_travel = ident_base = ident_last_delta = 0;
  ident_points = ident_base_angle = 0u;
  ident_have = ident_started = 0u;
  align_count = align_anchor = align_first_tick = align_last_tick = 0u;
  align_sum = align_min = align_max = 0;
  local_min_angle = local_max_angle = encoder_angle;
  local_points = 0u;
  local_zero_sum[0] = local_zero_sum[1] = 0;
  local_zero_count[0] = local_zero_count[1] = 0u;
  local_command_deg = -LOCAL_HALF_E_DEG;
  local_stage = 1u;
  uint32_t duration = is_torque(direction) ? TORQUE_ALIGN_MS+TORQUE_PULSE_MS : direction == TRACK_APPROACH ? CA_TOTAL_MS : direction == 10u ? APPROACH_TOTAL_MS : direction == 7u ? LOCAL_TOTAL_MS : ((direction == 6u || direction >= 8u) ? ALIGN_TOTAL_MS :
      (direction >= 4u ? IDENT_TOTAL_MS : (direction >= 2u ? SWEEP_MS : VECTOR_MS)));
  deadline = vector_start + duration;
  next_allowed = vector_start + duration + (direction >= 4u ? 5000u : 2000u);
  active_pair = 4u;
  HRTIM1->sCommonRegs.OENR = 0x3Fu;
  __set_PRIMASK(primask);
  /* Keep UART quiet during drive so encoder polling can meet its deadline. */
  if (is_torque(direction)) return;
  char line[192];
  if (direction==TRACK_APPROACH) {
    send_text("# VECTOR_BEGIN R24 dir=3 CURRENT_APPROACH; -45->0 HOLD700ms ->+45->0 HOLD700ms; 200mA ramp100ms depart_hold700ms total4700ms max_line_ticks=106/8000; x=stop; NOT_SAVED\r\n");
    return;
  }
  if (is_track(direction)) {
    (void)snprintf(line,sizeof(line),"# VECTOR_BEGIN R24 dir=%c STATOR_CURRENT_PI; target_ab_mA=%d,%d ramp_ms=100 duration_ms=700 max_line_ticks=106/8000; x=stop\r\n",
        vector_label(direction),direction==TRACK_ALPHA ? TRACK_TARGET_MA : 0,direction==TRACK_BETA ? TRACK_TARGET_MA : 0);
    send_text(line);
    return;
  }
  (void)snprintf(line, sizeof(line),
      "# VECTOR_BEGIN R24 dir=%c e_start=%ld e_end=%ld TRG2; duration_ms=%lu; ramp=50ms; max_line_duty=1.5percent (~0.18V at 12V); x=stop\r\n",
      vector_label(direction),
      (long)(direction == 11u ? (int32_t)map_degree : direction == 10u ? -45 : direction == 7u ? -25 : (direction == 5u ? 360 : ((direction == 1u || direction == 3u || direction == 9u) ? 90 : 0))),
      (long)(direction == 11u ? (int32_t)map_degree : direction == 7u ? -25 : (direction == 4u ? 360 : ((direction == 1u || direction == 2u || direction == 9u) ? 90 : 0))),
      (unsigned long)duration);
  send_text(line);
  if (direction == 11u) send_text("# MAP_PLAN one point only; last200ms stats; next h after stop +5s; no angle feedback; NOT_SAVED\r\n");
  if (direction == 10u) send_text("# APPROACH_PLAN -45->0 HOLD700ms ->+45->0 HOLD700ms; final200ms stats each; NOT_SAVED\r\n");
  if (direction == 7u) send_text("# LOCAL_PLAN e_deg=-25->+25->-25; 2700ms; measured mech guard=10..30deg; x=stop\r\n");
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
  send_text("# DIAG_BEGIN R15 PWM_OFF; route=TA_CMP2->HRTIM_TRG2->ADC1; stages=POLL,IRQ,SW\r\n");
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

/* Register audit only: outputs disabled and control pins held GPIO-low.
 * Register readback cannot certify actual MOS gate waveforms/deadtime. */
static void audit_pwm(void)
{
  if (active_pair || expired || collecting) {
    send_text("# AUDIT_REJECTED: PWM must be idle; wait for zero calibration\r\n"); return;
  }
  stop_pwm();
  if (!configure_timer()) {
    stop_pwm(); send_text("# AUDIT_FAIL DLL_TIMEOUT PWM_OFF\r\n"); return;
  }
  configure_adc_trigger();
  /* Snapshot while idle; ADC trigger configured but no ADC/timers started. */
  uint32_t jsqr = ADC1->JSQR, smpr = ADC1->SMPR1, cfgr = ADC1->CFGR;
  uint32_t sysclk = HAL_RCC_GetSysClockFreq();
  uint32_t common_ccr = ADC12_COMMON->CCR;
  send_text("# AUDIT_BEGIN R15 PWM_OFF; no motor drive; MCU DT=200ns; displayed current sign corrected\r\n");
  char line[256];
  for (uint32_t i = 0u; i < 3u; i++) {
    HRTIM_Timerx_TypeDef *timer = &HRTIM1->sTimerxRegs[i];
    (void)snprintf(line, sizeof(line),
        "# AUDIT_PWM phase=%c CR=0x%08lX PER=%lu CMP1=%lu DT=0x%08lX OUT=0x%08lX SET1=0x%08lX RST1=0x%08lX SET2=0x%08lX RST2=0x%08lX\r\n",
        'A'+i, (unsigned long)timer->TIMxCR, (unsigned long)timer->PERxR,
        (unsigned long)timer->CMP1xR, (unsigned long)timer->DTxR, (unsigned long)timer->OUTxR,
        (unsigned long)timer->SETx1R, (unsigned long)timer->RSTx1R,
        (unsigned long)timer->SETx2R, (unsigned long)timer->RSTx2R);
    send_text(line);
  }
  (void)snprintf(line, sizeof(line),
      "# AUDIT_ADC SYSCLK=%lu CCR=0x%08lX JSQR=0x%08lX SMPR1=0x%08lX CFGR=0x%08lX ADC2R=0x%08lX CMP2=%lu\r\n",
      (unsigned long)sysclk, (unsigned long)common_ccr, (unsigned long)jsqr,
      (unsigned long)smpr, (unsigned long)cfgr, (unsigned long)HRTIM1->sCommonRegs.ADC2R,
      (unsigned long)HRTIM1->sTimerxRegs[0].CMP2xR);
  stop_pwm();
  send_text(line);
  send_text("# AUDIT_PLAN PWM 20kHz; DT bits=256,DTPRSC0(MUL8), nominal200ns@160MHz; max line delta=120ticks=750ns; ADC phase=10us\r\n"
            "# AUDIT_MAP PA8/9=A PA10/11=B PB12/13=C; ADC1 rank1 PA0/CH1 rank2 PA1/CH2; phase I=(zero-ADC); raw ADC unchanged\r\n"
            "# AUDIT_END PWM_OFF; gate waveforms/current wiring still require physical verification\r\n");
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
  send_text("# STEP2B_R24_FOC_TEST STATOR_CURRENT_PI MCU_DT_200NS; PWM OFF at boot\r\n"
            "# PA0/PA1 current ADC; 5mOhm x20; nominal VDDA=3.3V\r\n"
            "# z=zero; f=FireWater; t=text; x=stop; c=clear fault; o/2=alpha/beta 200mA; 3=current approach; 4=replay trace; 5/6=temp align + Iq +/-100mA; start 20..22deg; 850ms lock + 500ms pulse; h=vector map\r\n"
            "# d=PWM-off diagnosis; a/b=fixed 100ms; g/j=90deg 400ms; n/m=360deg survey 2500ms; l=align700ms; k=local2700ms; q=two-sided-hold3700ms; e=auditOFF; i/y=current-check700ms; u/v/w disabled; current trip ~0.8A\r\n");
  start_zero();
}

/* Foreground only, after PWM and injected IRQ are disabled. */
static void report_trace(uint8_t reason)
{
  char line[240];
  if (trace_n<2u) { send_text("# CA_TRACE insufficient_samples\r\n"); return; }
  uint32_t last=trace_n-1u, peak=1u, maxgap=0, ten=0, ninety=0;
  uint32_t min_age=trace_pt[0].current_age_ms, max_age=min_age;
  int32_t net=signed_angle_cd((int32_t)trace_pt[last].cd-trace_pt[0].cd);
  int32_t sign=net<0 ? -1 : 1, total=net*sign, biggest=0;
  for (uint32_t i=1u;i<=last;i++) {
    uint32_t gap=trace_pt[i].ms-trace_pt[i-1u].ms;
    if (trace_pt[i].current_age_ms < min_age) min_age=trace_pt[i].current_age_ms;
    if (trace_pt[i].current_age_ms > max_age) max_age=trace_pt[i].current_age_ms;
    int32_t step=signed_angle_cd((int32_t)trace_pt[i].cd-trace_pt[i-1u].cd);
    if (gap>maxgap) maxgap=gap;
    if (i==1u || step*step>biggest*biggest) {peak=i; biggest=step;}
    int32_t prog=sign*signed_angle_cd((int32_t)trace_pt[i].cd-trace_pt[0].cd);
    if (!ten && 10*prog>=total) ten=i;
    if (!ninety && 10*prog>=9*total) ninety=i;
  }
  uint8_t complete=reason==1u && !trace_overflow && maxgap<=40u &&
      trace_pt[0].ms<CA_ZERO1_END_MS && TRACE_END_MS-trace_pt[last].ms<=40u;
  (void)snprintf(line,sizeof(line),"# CA_TRACE complete=%u n=%lu net_cd=%ld max_step_cd=%ld step_ms=%u..%u max_gap_ms=%lu current_age_ms=%lu..%lu\r\n",
      complete,(unsigned long)trace_n,(long)net,(long)biggest,trace_pt[peak-1u].ms,trace_pt[peak].ms,(unsigned long)maxgap,
      (unsigned long)min_age,(unsigned long)max_age);
  send_text(line);
  if (complete && total>=50 && ten && ninety>=ten) {
    /* Bound first-crossing time; if both thresholds cross in one sample,
     * report 0..sample_interval, never claim instantaneous motion. */
    unsigned lo=ninety>ten ? (unsigned)(trace_pt[ninety-1u].ms-trace_pt[ten].ms) : 0u;
    unsigned hi=(unsigned)(trace_pt[ninety].ms-trace_pt[ten-1u].ms);
    (void)snprintf(line,sizeof(line),"# CA_MOTION first_cross_10_90_ms=%u..%u; positive sweep=2100..2700ms; +45 hold=2700..3400ms\r\n",lo,hi);
    send_text(line);
  } else send_text("# CA_MOTION no_duration (incomplete trace or net <0.50deg)\r\n");
}

/* Replay up to eight timestamped angle/current samples per row. */
static void dump_trace(void)
{
  if (active_pair || expired) { send_text("# CA_POINTS wait for stop\r\n"); return; }
  if (!trace_n) { send_text("# CA_POINTS no samples retained\r\n"); return; }
  send_text("# CA_POINTS format=elapsed_ms:mech_cd:Ialpha_mA:Ibeta_mA:current_age_ms; last command 3 capture\r\n");
  for (uint32_t i=0;i<trace_n;) {
    char line[300]="# CA_PTS"; size_t n=strlen(line);
    for (uint32_t j=0u;j<8u && i<trace_n;j++,i++)
      n+=(size_t)snprintf(line+n,sizeof(line)-n," %u:%u:%d:%d:%u",trace_pt[i].ms,trace_pt[i].cd,
          (int)trace_pt[i].ia_mA,(int)trace_pt[i].ib_mA,trace_pt[i].current_age_ms);
    send_text(strcat(line,"\r\n"));
  }
}

static void report_current_approach(uint8_t reason)
{
  char line[256];
  int32_t mech[4], field[4];
  uint8_t all_current=1u, all_mech=1u;
  for (uint32_t i=0;i<4;i++) {
    uint32_t n=approach_i[i].n, m=approach_count[i];
    int32_t expected=i<2 ? 0 : (i==2 ? -45 : 45);
    int32_t da=approach_ref[i<2 ? 0 : 45][0], db=i<2 ? 0 : (i==2 ? -1 : 1)*approach_ref[45][1];
    int32_t a=n ? approach_i[i].sum_a/(int32_t)n : 0, b=n ? approach_i[i].sum_b/(int32_t)n : 0;
    field[i]=n && (a||b) ? (int32_t)(atan2f((float)b,(float)a)*5729.57795f) : 0;
    int32_t err=signed_angle_cd(field[i]-expected*100);
    uint32_t width=i<2 ? 200u : 100u;
    uint8_t current=reason==1u && n>=width-10u && n<=width+10u && !approach_i[i].limited && !approach_i[i].bad &&
        a>=da-30 && a<=da+30 && b>=db-30 && b<=db+30 && err>=-500 && err<=500;
    uint32_t span=m ? (uint32_t)(approach_max[i]-approach_min[i]) : 0u;
    mech[i]=m ? wrap_angle_cd((int32_t)approach_anchor[i]+approach_sum[i]/(int32_t)m) : 0;
    uint8_t sensed=reason==1u && m>=(i<2 ? 8u : 4u) && approach_last[i]-approach_first[i]>=(i<2 ? 140u:60u) &&
        vector_start+window_end_ms[i]-approach_last[i]<=60u && span<=ALIGN_MAX_SPAN_CD;
    all_current &= current; all_mech &= sensed;
    (void)snprintf(line,sizeof(line),"# CA_%s side=%u cmd_deg=%ld blocks=%lu mean_ab_mA=%ld,%ld field_cd=%ld limit_blocks=%lu bad_blocks=%lu current_ok=%u mech_n=%lu mech_cd=%ld span_cd=%lu stable=%u\r\n",
        i<2 ? "HOLD":"FROM",(unsigned)(1u+i%2u),(long)expected,(unsigned long)n,(long)a,(long)b,(long)field[i],
        (unsigned long)approach_i[i].limited,(unsigned long)approach_i[i].bad,current,(unsigned long)m,(long)mech[i],(unsigned long)span,sensed);
    send_text(line);
  }
  /* Early blocks are diagnostic only. Late FROM windows above decide usability. */
  for (uint32_t side=0;side<2;side++) {
    uint32_t i=4u+side, n=approach_i[i].n;
    if (!n) {
      (void)snprintf(line,sizeof(line),"# CA_EARLY side=%u blocks=0 no_data\r\n",(unsigned)(1u+side));
    } else {
      (void)snprintf(line,sizeof(line),"# CA_EARLY side=%u cmd_deg=%d blocks=%lu mean_ab_mA=%ld,%ld limit_blocks=%lu\r\n",
          (unsigned)(1u+side),side ? 45 : -45,(unsigned long)n,
          (long)(approach_i[i].sum_a/(int32_t)n),(long)(approach_i[i].sum_b/(int32_t)n),(unsigned long)approach_i[i].limited);
    }
    send_text(line);
  }
  int32_t move1=signed_angle_cd(mech[0]-mech[2]), move2=signed_angle_cd(mech[1]-mech[3]);
  int32_t diff=signed_angle_cd(ALIGN_ENCODER_SIGN*ALIGN_POLE_PAIRS*(mech[1]-mech[0]));
  int32_t field_diff=signed_angle_cd(field[1]-field[0]);
  uint8_t usable=reason==1u && !check_fault && all_current && all_mech && field_diff>=-500 && field_diff<=500 &&
      move1 < -ALIGN_MAX_SPAN_CD && move2 > ALIGN_MAX_SPAN_CD;
  (void)snprintf(line,sizeof(line),"# CA_COMPARE usable=%u all_current_ok=%u all_stable=%u move_mech_cd=%ld,%ld hold_field_diff_cd=%ld stop_diff_e_cd=%ld repeatability_ok=%u limit_e_cd=500; SCREEN_ONLY NOT_SAVED\r\n",
      usable,all_current,all_mech,(long)move1,(long)move2,(long)field_diff,(long)diff,
      (unsigned)(usable && diff>=-APPROACH_MAX_DIFF_CD && diff<=APPROACH_MAX_DIFF_CD));
  send_text(line);
}

void board_check_poll(void)
{
  if (expired != 0u) {
    uint8_t reason = expired;
    stop_pwm();
    char line[256];
    (void)snprintf(line, sizeof(line),
        "# VECTOR_END reason=%u (1=timeout 2=current 3=ADC_stale 4=encoder 8=mech_range 9=wrong_sign 10=align_reject); samples=%lu peak_mA=%lu angle_start=%lu.%02lu angle_now=%lu.%02lu dir=%c\r\n",
        reason, (unsigned long)vector_samples, (unsigned long)(peak_counts*33000u/4095u),
        (unsigned long)(angle_start/100u), (unsigned long)(angle_start%100u),
        (unsigned long)(encoder_angle/100u), (unsigned long)(encoder_angle%100u),
        vector_label(vector_direction));
    send_text(line);
    if (is_torque(vector_direction)) {
      uint32_t n=torque.align_count, nn=track.n, qn=torque.count;
      int32_t mean_mech=n ? torque.align_sum/(int32_t)n : 0;
      int32_t alpha=nn ? track.sum[0]/(int32_t)nn : 0, beta=nn ? track.sum[1]/(int32_t)nn : 0;
      int32_t id=qn ? torque.sum_d/(int32_t)qn : 0, iq=qn ? torque.sum_q/(int32_t)qn : 0;
      int32_t target=vector_direction==TORQUE_POSITIVE ? TORQUE_TARGET_MA : -TORQUE_TARGET_MA;
      (void)snprintf(line,sizeof(line),"# FOC_ALIGN ready=%u mech_cd=%ld span_cd=%ld enc_n=%lu current_ab_mA=%ld,%ld RAM_ONLY\r\n",
          torque.aligned,(long)mean_mech,(long)(n ? torque.align_max-torque.align_min : 0),(unsigned long)n,(long)alpha,(long)beta);
      send_text(line);
      uint32_t early_n=torque.early_count;
      if (early_n) {
        (void)snprintf(line,sizeof(line),"# FOC_EARLY pulse_ms=%u..%u mean_dq_mA=%ld,%ld blocks=%lu limit_blocks=%lu\r\n",
            (unsigned)TORQUE_RAMP_MS,(unsigned)(TORQUE_RAMP_MS+TORQUE_STATS_MS),
            (long)(torque.early_sum_d/(int32_t)early_n),(long)(torque.early_sum_q/(int32_t)early_n),
            (unsigned long)early_n,(unsigned long)torque.early_limited);
        send_text(line);
      }
      (void)snprintf(line,sizeof(line),"# FOC_PULSE reason=%u target_dq_mA=0,%ld mean_dq_mA=%ld,%ld final100ms_blocks=%lu limit_blocks=%lu travel_cd=%ld min_cd=%ld max_cd=%ld PWM_OFF\r\n",
          reason,(long)target,(long)id,(long)iq,(unsigned long)qn,(unsigned long)torque.limited,
          (long)torque.end_travel,(long)torque.min_travel,(long)torque.max_travel);
      send_text(line);
    }
    if (vector_direction==TRACK_APPROACH) { report_current_approach(reason); report_trace(reason); }
    if (is_track(vector_direction)) {
      uint32_t n=track.n;
      int32_t a=n ? track.sum[0]/(int32_t)n : 0, b=n ? track.sum[1]/(int32_t)n : 0;
      /* Bench screen, not current gain calibration. Check the range too, so
       * oscillation cannot pass just because its mean matches the command. */
      int32_t ref_a=vector_direction==TRACK_ALPHA ? TRACK_TARGET_MA : 0;
      int32_t ref_b=vector_direction==TRACK_BETA ? TRACK_TARGET_MA : 0;
      uint8_t ok=reason==1u && n>=190u && n<=210u && a>=ref_a-30 && a<=ref_a+30 && b>=ref_b-30 && b<=ref_b+30 &&
          track.lo[0]>=ref_a-60 && track.hi[0]<=ref_a+60 && track.lo[1]>=ref_b-60 && track.hi[1]<=ref_b+60 && !track.limited;
      (void)snprintf(line,sizeof(line),
          "# TRACK_RESULT screen_ok=%u reason=%u blocks=%lu limit_blocks=%lu target_ab_mA=%ld,%ld mean_ab_mA=%ld,%ld; last200ms PWM_OFF NOT_SAVED\r\n",
          ok,reason,(unsigned long)n,(unsigned long)track.limited,(long)ref_a,(long)ref_b,(long)a,(long)b);
      send_text(line);
      if (n) {
        (void)snprintf(line,sizeof(line),"# TRACK_RANGE block_1ms_mA alpha=%ld..%ld beta=%ld..%ld; peak_mA in VECTOR_END is per-sample phase peak\r\n",
            (long)track.lo[0],(long)track.hi[0],(long)track.lo[1],(long)track.hi[1]);
        send_text(line);
      }
    }
    if (vector_direction == 4u || vector_direction == 5u) {
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
    if (vector_direction == 11u) {
      uint32_t count = current_check_count;
      uint32_t span = align_count ? (uint32_t)(align_max-align_min) : 0u;
      uint32_t coverage = align_count ? align_last_tick-align_first_tick : 0u;
      uint32_t age = align_count ? vector_start+ALIGN_TOTAL_MS-align_last_tick : 999u;
      uint8_t usable = reason == 1u && !check_fault && count >= 3500u && count <= 4100u &&
          align_count >= 8u && coverage >= 140u && age <= 60u && span <= ALIGN_MAX_SPAN_CD;
      (void)snprintf(line, sizeof(line),
          "# MAP_POINT step=%u/12 cmd_e_deg=%lu reason=%u adc_n=%lu mech_n=%lu window_ms=%lu span_mech_cd=%lu usable=%u; SAMPLED_CURRENT_ONLY\r\n",
          (unsigned int)(map_degree/30u+1u), (unsigned long)map_degree, reason,
          (unsigned long)count, (unsigned long)align_count, (unsigned long)coverage, (unsigned long)span, usable);
      send_text(line);
      if (count && reason == 1u) {
        float u = (float)current_check_sum[0]/(float)count*33000.0f/4095.0f;
        float v = (float)current_check_sum[1]/(float)count*33000.0f/4095.0f;
        float beta = (u+2.0f*v)*0.577350269f;
        float magnitude = sqrtf(u*u+beta*beta);
        int32_t measured = wrap_angle_cd((int32_t)(atan2f(beta,u)*5729.57795f));
        int32_t a = sweep_phase[map_degree][0], b = sweep_phase[map_degree][1], c = sweep_phase[map_degree][2];
        int32_t expected = wrap_angle_cd((int32_t)(atan2f((float)(b-c)*0.577350269f,
            (float)(2*a-b-c)/3.0f)*5729.57795f));
        (void)snprintf(line, sizeof(line),
            "# MAP_MEAN U_V_W_mA=%ld,%ld,%ld alpha_beta_mA=%ld,%ld mag_mA=%ld; W_RECONSTRUCTED\r\n",
            (long)u,(long)v,(long)(-u-v),(long)u,(long)beta,(long)magnitude);
        send_text(line);
        (void)snprintf(line, sizeof(line),
            "# MAP_ANGLE expected_e_cd=%ld measured_e_cd=%ld error_e_cd=%ld angle_valid=%u; ticks=%ld,%ld,%ld; NOT_ZERO_CAL\r\n",
            (long)expected,(long)measured,(long)signed_angle_cd(measured-expected),
            (unsigned int)(usable && magnitude >= 80.0f),(long)a,(long)b,(long)c);
        send_text(line);
        (void)snprintf(line, sizeof(line),
            "# MAP_RANGE U_counts=%ld..%ld V_counts=%ld..%ld; fixed ADC phase=10us; NOT_GAIN_CAL\r\n",
            (long)current_check_min[0],(long)current_check_max[0],(long)current_check_min[1],(long)current_check_max[1]);
        send_text(line);
      }
      /* Do not silently skip unstable, interrupted or faulted points. */
      if (usable) map_next_step = (uint8_t)((map_next_step+1u)%12u);
      (void)snprintf(line,sizeof(line),"# MAP_NEXT cmd_e_deg=%u; manual h only; wait5s; reset restarts at0\r\n",
          (unsigned int)(map_next_step*30u));
      send_text(line);
    }
    if (vector_direction == 8u || vector_direction == 9u) {
      uint32_t count = current_check_count;
      uint32_t span = align_count ? (uint32_t)(align_max-align_min) : 0u;
      (void)snprintf(line, sizeof(line),
          "# CURRENT_CHECK dir=%c reason=%u adc_n=%lu mech_span_cd=%lu phase_sign=zero_minus_ADC; DISPLAY_SIGN_CORRECTED\r\n",
          vector_label(vector_direction), reason, (unsigned long)count, (unsigned long)span);
      send_text(line);
      if (count && reason == 1u) {
        int32_t u = (int32_t)((int64_t)current_check_sum[0]*33000/(int32_t)count/4095);
        int32_t v = (int32_t)((int64_t)current_check_sum[1]*33000/(int32_t)count/4095);
        int32_t w = -u-v;
        int32_t beta = (u+2*v)*577/1000;
        (void)snprintf(line, sizeof(line),
            "# CURRENT_MEAN phase_u_v_w_mA=%ld,%ld,%ld alpha_beta_mA=%ld,%ld; old_sign_reference=%ld,%ld,%ld\r\n",
            (long)u, (long)v, (long)w, (long)u, (long)beta, (long)-u, (long)-v, (long)-w);
        send_text(line);
        (void)snprintf(line, sizeof(line),
            "# CURRENT_RANGE phase_u_counts=%ld..%ld phase_v_counts=%ld..%ld; expected_pattern=%s; NOT_CALIBRATED\r\n",
            (long)current_check_min[0], (long)current_check_max[0],
            (long)current_check_min[1], (long)current_check_max[1],
            vector_direction == 8u ? "Upos_Vneg_Wneg" : "Unear0_Vpos_Wneg");
        send_text(line);
      }
    }
    if (vector_direction == 10u) {
      int32_t offsets[2]={0,0};
      uint8_t both_stable=reason==1u && !check_fault;
      uint32_t window_end[2]={1600u,3700u};
      for (uint32_t i=0u;i<2u;i++) {
        uint32_t count=approach_count[i];
        uint32_t span=count ? (uint32_t)(approach_max[i]-approach_min[i]) : 0u;
        uint32_t coverage=count ? approach_last[i]-approach_first[i] : 0u;
        uint32_t age=count ? vector_start+window_end[i]-approach_last[i] : 999u;
        uint8_t stable=reason==1u && count>=8u && coverage>=140u && age<=60u && span<=ALIGN_MAX_SPAN_CD;
        if (!stable) both_stable=0u;
        int32_t mean=count ? wrap_angle_cd((int32_t)approach_anchor[i]+approach_sum[i]/(int32_t)count) : 0;
        offsets[i]=wrap_angle_cd(-ALIGN_POLE_PAIRS*mean);
        (void)snprintf(line,sizeof(line),
            "# APPROACH_SIDE side=%u stable=%u n=%lu window_ms=%lu span_mech_cd=%lu mech_cd=%ld offset_e_cd=%ld; NOT_SAVED\r\n",
            (unsigned int)(i+1u),stable,(unsigned long)count,(unsigned long)coverage,
            (unsigned long)span,(long)mean,(long)offsets[i]);
        send_text(line);
      }
      int32_t difference=signed_angle_cd(offsets[1]-offsets[0]);
      int32_t magnitude=difference<0 ? -difference : difference;
      (void)snprintf(line,sizeof(line),
          "# APPROACH_COMPARE reason=%u both_stable=%u diff_e_cd=%ld repeatability_ok=%u limit_e_cd=500; SCREEN_ONLY NOT_SAVED\r\n",
          reason,both_stable,(long)difference,(unsigned int)(both_stable && magnitude<=APPROACH_MAX_DIFF_CD));
      send_text(line);
    }
    if (vector_direction == 7u) {
      (void)snprintf(line, sizeof(line),
          "# LOCAL_END reason=%u points=%lu mech_min_cd=%lu mech_max_cd=%lu zero_n=%lu,%lu; NOT_SAVED\r\n",
          reason, (unsigned long)local_points, (unsigned long)local_min_angle, (unsigned long)local_max_angle,
          (unsigned long)local_zero_count[0], (unsigned long)local_zero_count[1]);
      send_text(line);
      if (reason == 1u && local_zero_count[0] >= 2u && local_zero_count[1] >= 2u) {
        int32_t forward = local_zero_sum[0]/(int32_t)local_zero_count[0];
        int32_t backward = local_zero_sum[1]/(int32_t)local_zero_count[1];
        (void)snprintf(line, sizeof(line),
            "# LOCAL_ZERO_BIN command_e=-1..+1deg; forward_mech_cd=%ld reverse_mech_cd=%ld gap_mech_cd=%ld; DYNAMIC_NOT_ZERO_CAL\r\n",
            (long)forward, (long)backward, (long)(backward-forward));
        send_text(line);
      }
    }
    if (vector_direction == 6u) {
      uint32_t span = align_count ? (uint32_t)(align_max-align_min) : 0u;
      uint8_t stable = reason == 1u && !check_fault && align_count >= 8u &&
          align_last_tick-align_first_tick >= 140u && span <= ALIGN_MAX_SPAN_CD &&
          (int32_t)(deadline-align_last_tick) >= 0 && deadline-align_last_tick <= 60u;
      (void)snprintf(line, sizeof(line),
          "# ALIGN_%s reason=%u samples=%lu window_ms=%lu span_mech_cd=%lu peak_mA=%lu; NOT_SAVED\r\n",
          stable ? "STABLE_CANDIDATE" : "REJECTED", reason, (unsigned long)align_count,
          (unsigned long)(align_count ? align_last_tick-align_first_tick : 0u),
          (unsigned long)span, (unsigned long)(peak_counts*33000u/4095u));
      send_text(line);
      if (stable) {
        int32_t mean = wrap_angle_cd((int32_t)align_anchor + align_sum/(int32_t)align_count);
        /* Convention: theta_e = wrap(sign * pole_pairs * theta_m - offset).
         * Offset is measured at command field=0, not applied to control yet. */
        int32_t offset_cd = wrap_angle_cd(ALIGN_ENCODER_SIGN*ALIGN_POLE_PAIRS*mean);
        int32_t repeat = align_previous_valid ? signed_angle_cd(offset_cd-align_previous_offset) : 0;
        (void)snprintf(line, sizeof(line),
            "# ALIGN_RESULT pp=7 enc_sign=-1 mech_deg=%ld.%02ld offset_deg=%ld.%02ld prev_valid=%u diff_e_cd=%ld; theta_e=wrap(-7*mech-offset); TENTATIVE_NOT_SAVED\r\n",
            (long)(mean/100), (long)(mean%100), (long)(offset_cd/100), (long)(offset_cd%100),
            align_previous_valid, (long)repeat);
        send_text(line);
        align_previous_offset = offset_cd;
        align_previous_valid = 1u;
      }
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
      send_text("# z=zero; f=FireWater; t=text; x=stop; c=clear; o/2=alpha/beta 200mA; 3=current approach; 4=replay trace; 5/6=temp align + Iq +/-100mA; 20..22deg start; h=vector map\r\n"
                "# Torque 5/6: start at 20..22deg mechanical; align 850ms; pulse 500ms; x=stop. d=PWM-off diagnosis; a/b=fixed; g/j=90deg; n/m=360deg; l/k/q=align; e=audit; i/y=current checks\r\n");
      break;
    case 'x': stop_pwm(); send_text("# STOP all bridge inputs low\r\n"); break;
    case 'z': start_zero(); break;
    case 'f': chart_mode = 1u;
      send_text("# CH: angle_deg,UA_mV,VA_mV,IU_mA,IV_mA,valid,zero_ok,pwm_pair,check_fault,local_cmd_e_deg,local_stage\r\n"); break;
    case 't': chart_mode = 0u; break;
    case 'a': start_vector(0u); break;
    case 'b': start_vector(1u); break;
    case 'g': start_vector(2u); break;
    case 'j': start_vector(3u); break;
    case 'n': start_vector(4u); break;
    case 'm': start_vector(5u); break;
    case 'l': start_vector(6u); break;
    case 'k': start_vector(7u); break;
    case 'e': audit_pwm(); break;
    case 'i': start_vector(8u); break;
    case 'y': start_vector(9u); break;
    case 'q': start_vector(10u); break;
    case 'h': start_vector(11u); break;
    case 'o': start_vector(TRACK_ALPHA); break;
    case '2': start_vector(TRACK_BETA); break;
    case '3': start_vector(TRACK_APPROACH); break;
    case '4': dump_trace(); break;
    case '5': start_vector(TORQUE_POSITIVE); break;
    case '6': start_vector(TORQUE_NEGATIVE); break;
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
  int32_t ma0 = valid ? ((int32_t)offset[0] - raw[0]) * 33000 / 4095 : 0;
  int32_t ma1 = valid ? ((int32_t)offset[1] - raw[1]) * 33000 / 4095 : 0;
  if (chart_mode != 0u) {
    (void)snprintf(line, sizeof(line), "diag:%lu.%02lu,%lu,%lu,%ld,%ld,%u,%u,%u,%u,%ld,%u\r\n",
        (unsigned long)(angle_centideg / 100u), (unsigned long)(angle_centideg % 100u),
        (unsigned long)mv0, (unsigned long)mv1, (long)ma0, (long)ma1,
        (unsigned int)(valid && encoder_ok), zero_valid, active_pair, check_fault,
        (long)(active_pair == 4u && (vector_direction == 7u || vector_direction == 10u || vector_direction == 11u) ? local_command_deg : 0),
        (unsigned int)(active_pair == 4u && (vector_direction == 7u || vector_direction == 10u || vector_direction == 11u) ? local_stage : 0u));
  } else {
    (void)snprintf(line, sizeof(line),
        "# ADC raw=%u,%u mV=%lu,%lu zero=%u,%u I_mA=%ld,%ld valid=%u zero_ok=%u pwm_pair=%u check_fault=%u adc_err=%lu\r\n",
        raw[0], raw[1], (unsigned long)mv0, (unsigned long)mv1, offset[0], offset[1],
        (long)ma0, (long)ma1, valid, zero_valid, active_pair, check_fault, (unsigned long)adc_errors);
  }
  send_text(line);
}
