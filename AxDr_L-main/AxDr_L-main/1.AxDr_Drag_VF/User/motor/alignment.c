#include "alignment.h"
#include <math.h>
#include <string.h>
#define PI_F 3.14159265358979323846f
#define TWO_PI_F (2.0f * PI_F)

float alignment_wrap_pi(float value)
{
    while (value > PI_F) value -= TWO_PI_F;
    while (value < -PI_F) value += TWO_PI_F;
    return value;
}

int alignment_active(const alignment_t *a)
{
    return a->state >= ALIGN_RAMP && a->state <= ALIGN_RUN;
}

void alignment_init(alignment_t *a)
{
    memset(a, 0, sizeof(*a));
    a->cfg.align_u = 0.010f;
    a->cfg.run_u = 0.020f;
    a->cfg.stable_deg = 0.20f;
    a->cfg.rpm_limit = 1500.0f;
    a->cfg.ramp_ms = 1000u;
    a->cfg.hold_ms = 1000u;
    a->cfg.stable_ms = 300u;
    a->cfg.settle_timeout_ms = 3000u;
    a->cfg.pause_ms = 700u;
    a->cfg.run_ramp_ms = 1500u;
    a->cfg.run_ms = 3000u;
    a->cfg.pole_pairs = 7u; /* Reference default; verify on actual motor. */
    a->cfg.encoder_bits = 15u; /* Seekfree MENC15A reference driver. */
    a->cfg.direction = -1;
    a->cfg.auto_run = 0u; /* First test aligns only. */
}

int alignment_config_valid(const align_config_t *c)
{
    return isfinite(c->align_u) && c->align_u >= 0.001f && c->align_u <= 0.060f &&
        isfinite(c->run_u) && fabsf(c->run_u) <= 0.080f &&
        isfinite(c->stable_deg) && c->stable_deg >= 0.02f && c->stable_deg <= 2.0f &&
        isfinite(c->rpm_limit) && c->rpm_limit >= 30.0f && c->rpm_limit <= 5000.0f &&
        c->ramp_ms >= 200u && c->ramp_ms <= 5000u &&
        c->hold_ms >= 200u && c->hold_ms <= 5000u &&
        c->stable_ms >= 100u && c->stable_ms <= 2000u &&
        c->settle_timeout_ms >= c->stable_ms && c->settle_timeout_ms <= 5000u &&
        c->pause_ms <= 3000u && c->run_ramp_ms >= 200u && c->run_ramp_ms <= 5000u &&
        c->run_ms >= 100u && c->run_ms <= 10000u && c->pole_pairs >= 1u &&
        c->pole_pairs <= 30u && (c->encoder_bits == 14u || c->encoder_bits == 15u) &&
        (c->direction == 1 || c->direction == -1) && c->auto_run <= 1u;
}

void alignment_stop(alignment_t *a)
{
    a->state = ALIGN_IDLE;
    a->vd = a->vq = 0.0f;
}

void alignment_fail(alignment_t *a, align_fault_t fault)
{
    a->fault = fault;
    a->state = ALIGN_FAULT;
    a->vd = a->vq = 0.0f;
    a->zero_valid = 0u;
}

int alignment_start(alignment_t *a, uint32_t now)
{
    if (alignment_active(a) || a->fault != ALIGN_OK || !alignment_config_valid(&a->cfg)) return 0;
    a->zero_valid = 0u;
    a->span_deg = 0.0f;
    a->vd = a->vq = 0.0f;
    a->entered_ms = a->started_ms = a->heartbeat_ms = now;
    a->state = ALIGN_RAMP;
    return 1;
}

static void window_reset(alignment_t *a, uint32_t now, float angle)
{
    a->window_ms = now;
    a->window_ref = angle;
    a->window_min = a->window_max = a->window_sum = 0.0f;
    a->samples = 0u;
}

void alignment_tick(alignment_t *a, uint32_t now, float angle, int encoder_ok, float rpm)
{
    uint32_t elapsed = now - a->entered_ms;
    float t, delta;
    a->rpm = rpm;
    if (!alignment_active(a)) return;
    if (!encoder_ok || !isfinite(angle) || !isfinite(rpm)) {
        alignment_fail(a, ALIGN_ERR_ENCODER); return;
    }
    if (now - a->heartbeat_ms > 1000u) {
        alignment_fail(a, ALIGN_ERR_LINK); return;
    }
    if (fabsf(rpm) > a->cfg.rpm_limit) {
        alignment_fail(a, ALIGN_ERR_SPEED); return;
    }
    switch (a->state) {
    case ALIGN_RAMP:
        t = (float)elapsed / (float)a->cfg.ramp_ms;
        if (t > 1.0f) t = 1.0f;
        a->vd = a->cfg.align_u * t;
        if (elapsed >= a->cfg.ramp_ms) { a->state = ALIGN_HOLD; a->entered_ms = now; }
        break;
    case ALIGN_HOLD:
        a->vd = a->cfg.align_u;
        if (elapsed >= a->cfg.hold_ms) {
            a->state = ALIGN_SETTLE; a->entered_ms = now;
            window_reset(a, now, angle);
        }
        break;
    case ALIGN_SETTLE:
        delta = alignment_wrap_pi(angle - a->window_ref);
        if (delta < a->window_min) a->window_min = delta;
        if (delta > a->window_max) a->window_max = delta;
        a->span_deg = (a->window_max - a->window_min) * 180.0f / PI_F;
        a->window_sum += delta;
        a->samples++;
        if (a->span_deg > a->cfg.stable_deg) {
            window_reset(a, now, angle);
        } else if (now - a->window_ms >= a->cfg.stable_ms && a->samples >= a->cfg.stable_ms / 2u) {
            a->zero_rad = alignment_wrap_pi(a->window_ref + a->window_sum / (float)a->samples);
            a->zero_elec_deg = alignment_wrap_pi(a->zero_rad * a->cfg.pole_pairs * a->cfg.direction) * 180.0f / PI_F;
            a->zero_valid = 1u;
            a->state = ALIGN_PAUSE; a->entered_ms = now;
        }
        if (a->state == ALIGN_SETTLE && elapsed >= a->cfg.settle_timeout_ms)
            alignment_fail(a, ALIGN_ERR_UNSTABLE);
        break;
    case ALIGN_PAUSE:
        a->vd = a->cfg.align_u;
        if (elapsed >= a->cfg.pause_ms) {
            a->state = a->cfg.auto_run ? ALIGN_TRANSITION : ALIGN_DONE;
            a->entered_ms = now;
            if (!a->cfg.auto_run) a->vd = 0.0f;
        }
        break;
    case ALIGN_TRANSITION:
        t = (float)elapsed / (float)a->cfg.run_ramp_ms;
        if (t > 1.0f) t = 1.0f;
        a->vd = a->cfg.align_u * (1.0f - t);
        a->vq = a->cfg.run_u * t;
        if (elapsed >= a->cfg.run_ramp_ms) { a->state = ALIGN_RUN; a->entered_ms = now; }
        break;
    case ALIGN_RUN:
        /* Last 200 ms ramp down, then disable all bridge outputs. */
        t = (float)(a->cfg.run_ms > elapsed ? a->cfg.run_ms - elapsed : 0u) / 200.0f;
        if (t > 1.0f) t = 1.0f;
        a->vq = a->cfg.run_u * t;
        if (elapsed >= a->cfg.run_ms) { a->state = ALIGN_DONE; a->vq = 0.0f; }
        break;
    default: break;
    }
}

void alignment_voltage(const alignment_t *a, float angle, float *alpha, float *beta)
{
    float theta;
    *alpha = *beta = 0.0f;
    if (!alignment_active(a)) return;
    if (a->state < ALIGN_TRANSITION) { *alpha = a->vd; return; }
    theta = alignment_wrap_pi((angle - a->zero_rad) * a->cfg.pole_pairs * a->cfg.direction);
    if (a->state == ALIGN_TRANSITION) {
        /* Keep residual alignment field on the FIXED alpha axis, so switching
           to encoder commutation cannot rotate the existing d-axis vector. */
        *alpha = a->vd - a->vq * sinf(theta);
        *beta = a->vq * cosf(theta);
    } else {
        *alpha = -a->vq * sinf(theta);
        *beta = a->vq * cosf(theta);
    }
}
