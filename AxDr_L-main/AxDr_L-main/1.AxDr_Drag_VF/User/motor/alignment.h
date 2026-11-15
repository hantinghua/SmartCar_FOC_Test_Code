#ifndef ALIGNMENT_H
#define ALIGNMENT_H
#include <stdint.h>

/* Voltage values are alpha/beta amplitudes normalized to Vbus, NOT volts. */
typedef struct {
    float align_u, run_u, stable_deg, rpm_limit;
    uint32_t ramp_ms, hold_ms, stable_ms, settle_timeout_ms;
    uint32_t pause_ms, run_ramp_ms, run_ms;
    uint8_t pole_pairs, encoder_bits, auto_run;
    int8_t direction;
} align_config_t;

typedef enum {
    ALIGN_IDLE, ALIGN_RAMP, ALIGN_HOLD, ALIGN_SETTLE, ALIGN_PAUSE,
    ALIGN_TRANSITION, ALIGN_RUN, ALIGN_DONE, ALIGN_FAULT
} align_state_t;
typedef enum {
    ALIGN_OK, ALIGN_ERR_ENCODER, ALIGN_ERR_UNSTABLE, ALIGN_ERR_LINK,
    ALIGN_ERR_SPEED, ALIGN_ERR_HARDWARE
} align_fault_t;

typedef struct {
    align_config_t cfg;
    align_state_t state;
    align_fault_t fault;
    uint32_t entered_ms, started_ms, heartbeat_ms, window_ms, samples;
    float zero_rad, zero_elec_deg, span_deg, rpm;
    float window_ref, window_min, window_max, window_sum;
    float vd, vq;
    uint8_t zero_valid;
} alignment_t;

void alignment_init(alignment_t *a);
int alignment_config_valid(const align_config_t *cfg);
int alignment_active(const alignment_t *a);
int alignment_start(alignment_t *a, uint32_t now);
void alignment_stop(alignment_t *a);
void alignment_fail(alignment_t *a, align_fault_t fault);
/* Called once per millisecond, including while idle. angle is mechanical radians. */
void alignment_tick(alignment_t *a, uint32_t now, float angle, int encoder_ok, float rpm);
/* Called at PWM rate. Fixed alpha axis during alignment, encoder frame during run. */
void alignment_voltage(const alignment_t *a, float angle, float *alpha, float *beta);
float alignment_wrap_pi(float value);
#endif
