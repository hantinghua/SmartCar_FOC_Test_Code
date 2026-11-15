#ifndef VOLTAGE_TEST_H
#define VOLTAGE_TEST_H
#include "current_sense.h"

/* Independent fixed-alpha voltage experiment; never changes electrical zero. */
typedef enum { VTEST_IDLE, VTEST_RAMP, VTEST_HOLD, VTEST_DONE, VTEST_FAULT } voltage_test_state_t;
typedef enum {
    VTEST_OK, VTEST_ERR_CURRENT, VTEST_ERR_SAMPLE, VTEST_ERR_LINK,
    VTEST_ERR_ENCODER, VTEST_ERR_HARDWARE, VTEST_ERR_SPEED
} voltage_test_fault_t;
typedef struct {
    voltage_test_state_t state;
    voltage_test_fault_t fault;
    uint32_t started_ms, heartbeat_ms, ramp_ms, hold_ms;
    uint32_t valid_samples, invalid_samples;
    float target, command, limit_a, peak_a, peak_b, peak_c;
} voltage_test_t;

void voltage_test_init(voltage_test_t *t);
int voltage_test_active(const voltage_test_t *t);
int voltage_test_start(voltage_test_t *t, uint32_t now, int permille,
                       uint32_t ramp_ms, uint32_t hold_ms, uint32_t limit_ma, int direction);
void voltage_test_stop(voltage_test_t *t);
void voltage_test_fail(voltage_test_t *t, voltage_test_fault_t fault);
/* Called for every ADC pair; the only ignored invalid samples are those in
 * the first 2 ms, covering the PWM enable settling exclusion. */
void voltage_test_step(voltage_test_t *t, uint32_t now, const current_sense_t *c,
                      int encoder_ok, float rpm);
#endif
