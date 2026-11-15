#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H
#include "current_sense.h"

/* Stationary alpha/beta current test in the existing ADC sign convention.
 * No encoder commutation, electrical-zero update, speed or position loop. */
#define ILOOP_VOLTAGE_LIMIT 0.005f
#define ILOOP_PHASE_LIMIT 0.5f
#define ILOOP_PROBE_VOLTAGE 0.003f
#define ILOOP_KP 0.015f       /* normalized Vbus / A */
#define ILOOP_KI 0.3f         /* normalized Vbus / (A*s) */
#define ILOOP_KP_MAX 0.06f
#define ILOOP_KI_MAX 1.2f
typedef enum {
    ILOOP_IDLE, ILOOP_BASELINE, ILOOP_ALPHA_POS, ILOOP_ALPHA_NEG,
    ILOOP_BETA_POS, ILOOP_BETA_NEG, ILOOP_SETTLE, ILOOP_RAMP,
    ILOOP_HOLD, ILOOP_DONE, ILOOP_FAULT
} current_loop_state_t;
typedef enum {
    ILOOP_OK, ILOOP_ERR_CURRENT, ILOOP_ERR_SAMPLE, ILOOP_ERR_LINK,
    ILOOP_ERR_ENCODER, ILOOP_ERR_HARDWARE, ILOOP_ERR_SPEED,
    ILOOP_ERR_DIRECTION, ILOOP_ERR_WRONG_WAY
} current_loop_fault_t;
typedef struct {
    current_loop_state_t state;
    current_loop_fault_t fault;
    uint32_t started_ms, entered_ms, heartbeat_ms, hold_ms;
    uint32_t valid_samples, invalid_samples, probe_samples, wrong_way_count;
    uint32_t mean_samples;
    float target, reference, alpha, beta, error;
    float kp, ki; /* Latched for this test; shared by alpha and beta. */
    float out_alpha, out_beta, integral_alpha, integral_beta;
    float probe_sum_alpha, probe_sum_beta, probe_alpha[4], probe_beta[4];
    float mean_alpha, mean_beta, mean_error, peak;
    int8_t polarity_alpha, polarity_beta;
    uint8_t valid, filtered, limited;
} current_loop_t;
void current_loop_init(current_loop_t *t);
int current_loop_active(const current_loop_t *t);
int current_loop_start(current_loop_t *t, uint32_t now, int target_ma, uint32_t hold_ms);
int current_loop_start_gains(current_loop_t *t, uint32_t now, int target_ma,
                             uint32_t hold_ms, float kp, float ki);
void current_loop_stop(current_loop_t *t);
void current_loop_fail(current_loop_t *t, current_loop_fault_t fault);
/* Exactly once per 20 kHz ADC pair, also while idle to refresh feedback. */
void current_loop_step(current_loop_t *t, uint32_t now, const current_sense_t *c,
                       int encoder_ok, float rpm);
#endif
