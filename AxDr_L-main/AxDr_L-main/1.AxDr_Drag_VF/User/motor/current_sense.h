#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H
#include <stdint.h>

/* Confirmed board: R005 (5 mOhm), INA240A1 (20 V/V).
 * VDDA is nominal 3.3 V; gain/sign still require hardware verification. */
#define CURRENT_A_PER_COUNT (3.3f / 4096.0f / 0.005f / 20.0f)
#define CURRENT_ZERO_SAMPLES 4096u
typedef enum { CURRENT_WAIT, CURRENT_CALIBRATING, CURRENT_READY, CURRENT_FAILED } current_state_t;
typedef struct {
    current_state_t state;
    uint32_t started_ms, count, sum_a, sum_b;
    uint16_t min_a, max_a, min_b, max_b;
    float offset_a, offset_b, ia, ib, ic;
    uint8_t valid;
} current_sense_t;
void current_sense_reset(current_sense_t *s, uint32_t now);
void current_sense_update(current_sense_t *s, uint32_t now, uint16_t a, uint16_t b,
                          int outputs_on, int window_valid);
#endif
