#include "current_sense.h"
#include <string.h>

void current_sense_reset(current_sense_t *s, uint32_t now)
{
    memset(s, 0, sizeof(*s));
    s->started_ms = now;
    s->min_a = s->min_b = 4095u;
}

void current_sense_update(current_sense_t *s, uint32_t now, uint16_t a, uint16_t b,
                          int outputs_on, int window_valid)
{
    s->valid = 0u;
    if (s->state == CURRENT_FAILED) return;
    if (s->state != CURRENT_READY) {
        /* Calibration NEVER enables outputs; reject contamination by PWM. */
        if (outputs_on || now - s->started_ms > 3000u) {
            s->state = CURRENT_FAILED; return;
        }
        if (now - s->started_ms < 200u) return;
        s->state = CURRENT_CALIBRATING;
        if (a < s->min_a) s->min_a = a;
        if (a > s->max_a) s->max_a = a;
        if (b < s->min_b) s->min_b = b;
        if (b > s->max_b) s->max_b = b;
        s->sum_a += a; s->sum_b += b;
        if (++s->count < CURRENT_ZERO_SAMPLES) return;
        s->offset_a = (float)s->sum_a / (float)s->count;
        s->offset_b = (float)s->sum_b / (float)s->count;
        if (s->offset_a < 1024.0f || s->offset_a > 3072.0f ||
            s->offset_b < 1024.0f || s->offset_b > 3072.0f ||
            s->max_a - s->min_a > 32u || s->max_b - s->min_b > 32u) {
            s->state = CURRENT_FAILED; return;
        }
        s->state = CURRENT_READY;
    }
    if (!window_valid || a < 16u || a > 4079u || b < 16u || b > 4079u) return;
    s->ia = ((float)a - s->offset_a) * CURRENT_A_PER_COUNT;
    s->ib = ((float)b - s->offset_b) * CURRENT_A_PER_COUNT;
    s->ic = -s->ia - s->ib;
    s->valid = 1u;
}
