#include "voltage_test.h"
#include <string.h>

static float magnitude(float x) { return x < 0.0f ? -x : x; }
void voltage_test_init(voltage_test_t *t) { memset(t, 0, sizeof(*t)); }
int voltage_test_active(const voltage_test_t *t)
{
    return t->state == VTEST_RAMP || t->state == VTEST_HOLD;
}
void voltage_test_stop(voltage_test_t *t)
{
    /* Keep fault/peaks until CLEAR or a successful new run. */
    t->state = t->fault == VTEST_OK ? VTEST_IDLE : VTEST_FAULT;
    t->command = 0.0f;
}
void voltage_test_fail(voltage_test_t *t, voltage_test_fault_t fault)
{
    t->state = VTEST_FAULT; t->fault = fault; t->command = 0.0f;
}
int voltage_test_start(voltage_test_t *t, uint32_t now, int permille,
                       uint32_t ramp_ms, uint32_t hold_ms, uint32_t limit_ma, int direction)
{
    if (voltage_test_active(t) || t->fault != VTEST_OK || permille < 1 || permille > 18 ||
        ramp_ms < 500u || ramp_ms > 2000u || hold_ms < 200u || hold_ms > 2000u ||
        limit_ma < 100u || limit_ma > 1500u || (direction != -1 && direction != 1)) return 0;
    voltage_test_init(t);
    t->state = VTEST_RAMP; t->started_ms = t->heartbeat_ms = now;
    t->target = (float)(permille * direction) / 1000.0f;
    t->ramp_ms = ramp_ms; t->hold_ms = hold_ms; t->limit_a = (float)limit_ma / 1000.0f;
    return 1;
}
void voltage_test_step(voltage_test_t *t, uint32_t now, const current_sense_t *c,
                      int encoder_ok, float rpm)
{
    uint32_t elapsed = now - t->started_ms;
    float ia, ib, ic;
    if (!voltage_test_active(t)) return;
    if (now - t->heartbeat_ms > 1000u) { voltage_test_fail(t, VTEST_ERR_LINK); return; }
    if (!encoder_ok || rpm != rpm) { voltage_test_fail(t, VTEST_ERR_ENCODER); return; }
    if (magnitude(rpm) > 300.0f) { voltage_test_fail(t, VTEST_ERR_SPEED); return; }
    if (c->state != CURRENT_READY) { voltage_test_fail(t, VTEST_ERR_SAMPLE); return; }
    if (!c->valid) {
        t->invalid_samples++;
        if (elapsed >= 2u) { voltage_test_fail(t, VTEST_ERR_SAMPLE); return; }
    } else {
        ia = magnitude(c->ia); ib = magnitude(c->ib); ic = magnitude(c->ic);
        if (ia != ia || ib != ib || ic != ic) { voltage_test_fail(t, VTEST_ERR_SAMPLE); return; }
        t->valid_samples++;
        if (ia > t->peak_a) t->peak_a = ia;
        if (ib > t->peak_b) t->peak_b = ib;
        if (ic > t->peak_c) t->peak_c = ic;
        if (ia > t->limit_a || ib > t->limit_a || ic > t->limit_a) {
            voltage_test_fail(t, VTEST_ERR_CURRENT); return;
        }
    }
    if (elapsed >= t->ramp_ms + t->hold_ms) {
        t->state = VTEST_DONE; t->command = 0.0f;
    } else if (elapsed >= t->ramp_ms) {
        t->state = VTEST_HOLD; t->command = t->target;
    } else t->command = t->target * ((float)elapsed / (float)t->ramp_ms);
}
