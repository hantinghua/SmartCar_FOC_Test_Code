#include "common.h"
#include "alignment_port.h"
#include "hrtim_pwm.h"
#include "encoder.h"

alignment_t g_alignment;
current_sense_t g_current;
voltage_test_t g_voltage_test;
current_loop_t g_current_loop;
rotor_test_t g_rotor_test;
volatile uint32_t g_sample_hz;
volatile uint32_t g_isr_cnt = 0;
volatile uint16_t g_ia_raw = 0, g_ib_raw = 0;
volatile float g_foc_rpm = 0.0f;
static uint8_t hardware_ready, outputs_on;
static uint32_t last_step_ms, last_poll_ms, last_count;
static float last_angle, speed_delta;
static uint16_t speed_count;
static uint8_t angle_valid;
static uint8_t previous_bits;
static uint8_t sampling_skip;
static uint32_t rate_ms, rate_count;

int alignment_port_zero_current(void)
{
    if (alignment_port_active() || outputs_on) return 0;
    current_sense_reset(&g_current, HAL_GetTick());
    return 1;
}

int alignment_port_outputs_on(void) { return outputs_on != 0u; }
int alignment_port_active(void)
{
    return alignment_active(&g_alignment) || voltage_test_active(&g_voltage_test) || current_loop_active(&g_current_loop) || rotor_test_active(&g_rotor_test);
}

void alignment_port_stop(void)
{
    alignment_stop(&g_alignment);
    voltage_test_stop(&g_voltage_test);
    current_loop_stop(&g_current_loop);
    rotor_test_stop(&g_rotor_test);
    hrtim_pwm_outputs_disable();
    outputs_on = 0u;
}

void alignment_port_init(int hardware_ok)
{
    alignment_init(&g_alignment);
    voltage_test_init(&g_voltage_test);
    current_loop_init(&g_current_loop);
    rotor_test_init(&g_rotor_test);
    hardware_ready = hardware_ok ? 1u : 0u;
    outputs_on = 0u;
    current_sense_reset(&g_current, HAL_GetTick());
    if (!hardware_ready) alignment_fail(&g_alignment, ALIGN_ERR_HARDWARE);
}

void alignment_port_poll(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_poll_ms >= 20u) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (alignment_port_active() && g_isr_cnt == last_count) {
            if (rotor_test_active(&g_rotor_test)) rotor_test_fail(&g_rotor_test, ROTOR_ERR_HARDWARE);
            else if (current_loop_active(&g_current_loop)) current_loop_fail(&g_current_loop, ILOOP_ERR_HARDWARE);
            else if (voltage_test_active(&g_voltage_test)) voltage_test_fail(&g_voltage_test, VTEST_ERR_HARDWARE);
            else alignment_fail(&g_alignment, ALIGN_ERR_HARDWARE);
            hrtim_pwm_outputs_disable();
            outputs_on = 0u;
        }
        last_count = g_isr_cnt;
        last_poll_ms = now;
        if (now - rate_ms >= 1000u) {
            g_sample_hz = (g_isr_cnt - rate_count) * 1000u / (now - rate_ms);
            rate_ms = now; rate_count = g_isr_cnt;
            if (current_loop_active(&g_current_loop) && (g_sample_hz < 18000u || g_sample_hz > 22000u)) {
                current_loop_fail(&g_current_loop, ILOOP_ERR_SAMPLE);
                hrtim_pwm_outputs_disable(); outputs_on = 0u;
            }
        }
        __set_PRIMASK(mask);
    }
}

/* ADC triggers remain running while outputs are OFF, to observe hand rotation.
   SPI is owned exclusively by this ISR; its transfer has a cycle-based deadline. */
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint32_t now;
    float mech, delta, alpha, beta, va, vb, vc, hi, lo, common;
    int ok;
    if (hadc != &hadc1) return;
    g_isr_cnt++;
    g_ia_raw = (uint16_t)ADC1->JDR1;
    g_ib_raw = (uint16_t)ADC1->JDR2;
    now = HAL_GetTick();
    /* Trigger at count 400 (2.5 us). Two 60-cycle ADC conversions at
     * 40 MHz finish by 5.5 us. Require 7.5 us before either high side on.
     * First two sequences after enable are excluded while timing settles. */
    int window_ok = !outputs_on || (!sampling_skip &&
        HRTIM1->sTimerxRegs[HRTIM_IDX_A].CMP1xR >= 1200u &&
        HRTIM1->sTimerxRegs[HRTIM_IDX_B].CMP1xR >= 1200u);
    current_sense_update(&g_current, now, g_ia_raw, g_ib_raw, outputs_on, window_ok);
    if (sampling_skip) sampling_skip--;
    if (!hardware_ready) return;
    if (previous_bits != g_alignment.cfg.encoder_bits) {
        previous_bits = g_alignment.cfg.encoder_bits;
        angle_valid = 0u; speed_delta = 0.0f; speed_count = 0u; g_foc_rpm = 0.0f;
        encoder_set_bits(previous_bits);
    }
    mech = encoder_read_angle_rad(ENC_IDX_1);
    ok = (g_enc_last_ok != 0u);
    if (!ok) {
        angle_valid = 0u; speed_delta = 0.0f; speed_count = 0u;
        if (alignment_active(&g_alignment)) alignment_fail(&g_alignment, ALIGN_ERR_ENCODER);
    } else {
        if (angle_valid) {
            delta = alignment_wrap_pi(mech - last_angle);
            /* Reject > 18 degrees / 50us as a corrupt angle, not a valid motor speed. */
            if (fabsf(delta) > 0.31415927f && alignment_port_active()) {
                if (current_loop_active(&g_current_loop)) current_loop_fail(&g_current_loop, ILOOP_ERR_ENCODER);
                else if (voltage_test_active(&g_voltage_test)) voltage_test_fail(&g_voltage_test, VTEST_ERR_ENCODER);
                else alignment_fail(&g_alignment, ALIGN_ERR_ENCODER);
            }
            speed_delta += delta;
        }
        last_angle = mech;
        angle_valid = 1u;
        if (++speed_count >= 100u) {
            float rpm = speed_delta * (60.0f / (6.28318531f * 0.005f));
            g_foc_rpm += 0.25f * (rpm - g_foc_rpm);
            speed_delta = 0.0f; speed_count = 0u;
        }
    }
    now = HAL_GetTick();
    current_loop_step(&g_current_loop, now, &g_current, ok, g_foc_rpm);
    rotor_test_step(&g_rotor_test, now, mech, ok, g_foc_rpm, &g_current,
                    g_alignment.cfg.pole_pairs, g_alignment.cfg.direction);
    if (now != last_step_ms) {
        last_step_ms = now;
        alignment_tick(&g_alignment, now, mech, ok, g_foc_rpm);
    }
    if (voltage_test_active(&g_voltage_test))
        voltage_test_step(&g_voltage_test, now, &g_current, ok, g_foc_rpm);
    if (!alignment_port_active()) {
        if (outputs_on) { hrtim_pwm_outputs_disable(); outputs_on = 0u; }
        return;
    }
    if (g_current.state != CURRENT_READY) {
        alignment_fail(&g_alignment, ALIGN_ERR_HARDWARE);
        hrtim_pwm_outputs_disable(); outputs_on = 0u; return;
    }
    if (rotor_test_active(&g_rotor_test)) {
        rotor_test_voltage(&g_rotor_test, mech, g_alignment.cfg.pole_pairs, &alpha, &beta);
    } else if (current_loop_active(&g_current_loop)) {
        alpha = g_current_loop.out_alpha; beta = g_current_loop.out_beta;
    } else if (voltage_test_active(&g_voltage_test)) {
        alpha = g_voltage_test.command; beta = 0.0f;
    } else alignment_voltage(&g_alignment, mech, &alpha, &beta);
    va = alpha;
    vb = -0.5f * alpha + 0.86602540f * beta;
    vc = -0.5f * alpha - 0.86602540f * beta;
    hi = fmaxf(va, fmaxf(vb, vc));
    lo = fminf(va, fminf(vb, vc));
    common = (hi + lo) * 0.5f;
    hrtim_set_duty(HRTIM_IDX_A, 0.5f + va - common);
    hrtim_set_duty(HRTIM_IDX_B, 0.5f + vb - common);
    hrtim_set_duty(HRTIM_IDX_C, 0.5f + vc - common);
    if (!outputs_on) {
        hrtim_pwm_outputs_enable((1u << HRTIM_IDX_A) | (1u << HRTIM_IDX_B) | (1u << HRTIM_IDX_C));
        outputs_on = 1u;
        sampling_skip = 2u;
    }
}
