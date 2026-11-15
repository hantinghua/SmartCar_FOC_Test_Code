#include "current_loop.h"
#include <string.h>

static float iabs(float x) { return x < 0.0f ? -x : x; }
static float ilimit(float x, float limit)
{
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}
static int finite_current(float x) { return x == x && x > -100.0f && x < 100.0f; }
static void output_zero(current_loop_t *t)
{
    t->reference = t->out_alpha = t->out_beta = 0.0f;
    t->integral_alpha = t->integral_beta = 0.0f;
    t->limited = 0u; t->error = -t->alpha;
}
void current_loop_init(current_loop_t *t)
{
    memset(t, 0, sizeof(*t));
    t->kp = ILOOP_KP; t->ki = ILOOP_KI;
}
int current_loop_active(const current_loop_t *t)
{
    return t->state >= ILOOP_BASELINE && t->state <= ILOOP_HOLD;
}
void current_loop_stop(current_loop_t *t)
{
    t->state = t->fault == ILOOP_OK ? ILOOP_IDLE : ILOOP_FAULT;
    output_zero(t);
}
void current_loop_fail(current_loop_t *t, current_loop_fault_t fault)
{
    t->fault = fault; t->state = ILOOP_FAULT; output_zero(t);
}
int current_loop_start(current_loop_t *t, uint32_t now, int target_ma, uint32_t hold_ms)
{
    return current_loop_start_gains(t, now, target_ma, hold_ms, ILOOP_KP, ILOOP_KI);
}
int current_loop_start_gains(current_loop_t *t, uint32_t now, int target_ma,
                             uint32_t hold_ms, float kp, float ki)
{
    if (current_loop_active(t) || t->fault != ILOOP_OK ||
        target_ma < -200 || target_ma > 200 || hold_ms < 500u || hold_ms > 2000u ||
        !(kp >= 0.0f && kp <= ILOOP_KP_MAX && ki >= 0.0f && ki <= ILOOP_KI_MAX)) return 0;
    current_loop_init(t);
    t->kp = kp; t->ki = ki;
    t->target = (float)target_ma / 1000.0f; t->hold_ms = hold_ms;
    t->started_ms = t->entered_ms = t->heartbeat_ms = now;
    t->state = ILOOP_BASELINE;
    return 1;
}
static void next_state(current_loop_t *t, current_loop_state_t state, uint32_t now)
{
    t->state = state; t->entered_ms = now;
    t->probe_samples = 0u; t->probe_sum_alpha = t->probe_sum_beta = 0.0f;
    t->out_alpha = t->out_beta = 0.0f;
    if (state == ILOOP_ALPHA_POS) t->out_alpha = ILOOP_PROBE_VOLTAGE;
    if (state == ILOOP_ALPHA_NEG) t->out_alpha = -ILOOP_PROBE_VOLTAGE;
    if (state == ILOOP_BETA_POS) t->out_beta = ILOOP_PROBE_VOLTAGE;
    if (state == ILOOP_BETA_NEG) t->out_beta = -ILOOP_PROBE_VOLTAGE;
}
static int identify_direction(current_loop_t *t)
{
    float aa = t->probe_alpha[0] - t->probe_alpha[1];
    float ba = t->probe_beta[0] - t->probe_beta[1];
    float ab = t->probe_alpha[2] - t->probe_alpha[3];
    float bb = t->probe_beta[2] - t->probe_beta[3];
    /* Opposite probes cancel constant bias. Reject weak or strongly coupled
     * feedback instead of guessing the sign or starting a current loop. */
    if (iabs(aa) < 0.030f || iabs(bb) < 0.030f ||
        iabs(ba) > 0.5f * iabs(aa) || iabs(ab) > 0.5f * iabs(bb)) return 0;
    t->polarity_alpha = aa > 0.0f ? 1 : -1;
    t->polarity_beta = bb > 0.0f ? 1 : -1;
    return 1;
}
static void pi_step(current_loop_t *t)
{
    float ea = t->reference - t->alpha, eb = -t->beta;
    float ua = t->kp * ea + t->integral_alpha;
    float ub = t->kp * eb + t->integral_beta;
    float size = iabs(ua) + iabs(ub);
    t->error = ea;
    t->limited = size > ILOOP_VOLTAGE_LIMIT;
    /* Conservative diamond limit implies sqrt(ua^2+ub^2) <= 0.5% Vbus.
     * Conditional integration prevents windup when either axis saturates. */
    if (!t->limited || ea * ua + eb * ub < 0.0f) {
        t->integral_alpha = ilimit(t->integral_alpha + t->ki * 0.00005f * ea, ILOOP_VOLTAGE_LIMIT);
        t->integral_beta = ilimit(t->integral_beta + t->ki * 0.00005f * eb, ILOOP_VOLTAGE_LIMIT);
    }
    if (t->limited) { ua *= ILOOP_VOLTAGE_LIMIT / size; ub *= ILOOP_VOLTAGE_LIMIT / size; }
    /* Sign belongs ONLY to this test's output mapping; ADC values and the
     * original voltage/alignment modes keep their existing convention. */
    t->out_alpha = (float)t->polarity_alpha * ua;
    t->out_beta = (float)t->polarity_beta * ub;
}
void current_loop_step(current_loop_t *t, uint32_t now, const current_sense_t *c,
                       int encoder_ok, float rpm)
{
    uint32_t elapsed = now - t->entered_ms;
    t->valid = c->state == CURRENT_READY && c->valid &&
        finite_current(c->ia) && finite_current(c->ib) && finite_current(c->ic);
    if (t->valid) {
        float beta = (c->ia + 2.0f * c->ib) * 0.577350269f;
        if (!t->filtered) { t->alpha = c->ia; t->beta = beta; t->filtered = 1u; }
        else {
            t->alpha += 0.025f * (c->ia - t->alpha);
            t->beta += 0.025f * (beta - t->beta);
        }
        t->error = t->reference - t->alpha;
    }
    if (!current_loop_active(t)) return;
    if (now - t->heartbeat_ms > 1000u) { current_loop_fail(t, ILOOP_ERR_LINK); return; }
    if (!encoder_ok || !finite_current(rpm / 100.0f)) { current_loop_fail(t, ILOOP_ERR_ENCODER); return; }
    if (iabs(rpm) > 300.0f) { current_loop_fail(t, ILOOP_ERR_SPEED); return; }
    if (!t->valid) {
        t->invalid_samples++;
        if (c->state != CURRENT_READY || now - t->started_ms >= 2u)
            current_loop_fail(t, ILOOP_ERR_SAMPLE);
        return;
    }
    t->valid_samples++;
    float peak = iabs(c->ia);
    if (iabs(c->ib) > peak) peak = iabs(c->ib);
    if (iabs(c->ic) > peak) peak = iabs(c->ic);
    if (peak > t->peak) t->peak = peak;
    if (peak > ILOOP_PHASE_LIMIT) { current_loop_fail(t, ILOOP_ERR_CURRENT); return; }

    if (t->state == ILOOP_BASELINE) {
        if (elapsed >= 100u) next_state(t, ILOOP_ALPHA_POS, now);
    } else if (t->state >= ILOOP_ALPHA_POS && t->state <= ILOOP_BETA_NEG) {
        /* Average RAW (offset-corrected) samples in the last 40 ms of each
         * 120 ms probe; never alter current zero offsets. */
        if (elapsed >= 80u && elapsed < 120u) {
            t->probe_sum_alpha += c->ia;
            t->probe_sum_beta += (c->ia + 2.0f * c->ib) * 0.577350269f;
            t->probe_samples++;
        }
        if (elapsed >= 120u) {
            unsigned index = (unsigned)t->state - (unsigned)ILOOP_ALPHA_POS;
            if (t->probe_samples < 400u) { current_loop_fail(t, ILOOP_ERR_SAMPLE); return; }
            t->probe_alpha[index] = t->probe_sum_alpha / (float)t->probe_samples;
            t->probe_beta[index] = t->probe_sum_beta / (float)t->probe_samples;
            if (t->state != ILOOP_BETA_NEG) next_state(t, (current_loop_state_t)(t->state + 1), now);
            else if (!identify_direction(t)) current_loop_fail(t, ILOOP_ERR_DIRECTION);
            else next_state(t, ILOOP_SETTLE, now);
        }
    } else if (t->state == ILOOP_SETTLE) {
        if (elapsed >= 100u) next_state(t, ILOOP_RAMP, now);
    } else {
        if (t->state == ILOOP_RAMP) {
            t->reference = t->target * (elapsed >= 1000u ? 1.0f : (float)elapsed / 1000.0f);
            if (elapsed >= 1000u) { next_state(t, ILOOP_HOLD, now); elapsed = 0u; }
        }
        if (t->state == ILOOP_HOLD) {
            t->reference = t->target;
            if (elapsed >= t->hold_ms) {
                t->state = ILOOP_DONE; output_zero(t); return;
            }
            if (elapsed >= t->hold_ms - 500u) {
                t->mean_samples++;
                t->mean_alpha += (t->alpha - t->mean_alpha) / (float)t->mean_samples;
                t->mean_beta += (t->beta - t->mean_beta) / (float)t->mean_samples;
                t->mean_error = t->target - t->mean_alpha;
            }
        }
        if (iabs(t->reference) > 0.030f && iabs(t->alpha) > 0.030f && t->reference * t->alpha < 0.0f) {
            if (++t->wrong_way_count >= 400u) { current_loop_fail(t, ILOOP_ERR_WRONG_WAY); return; }
        } else t->wrong_way_count = 0u;
        pi_step(t);
    }
}
