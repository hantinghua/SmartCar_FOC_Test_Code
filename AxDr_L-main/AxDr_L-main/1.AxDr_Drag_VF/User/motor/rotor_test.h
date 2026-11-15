#ifndef ROTOR_TEST_H
#define ROTOR_TEST_H
#include <stdint.h>
#include "current_sense.h"

typedef enum { ROTOR_IDLE, ROTOR_PROBE, ROTOR_PREALIGN, ROTOR_SWEEP_FWD,
               ROTOR_SWEEP_REV, ROTOR_ZERO_HOLD, ROTOR_Q_RAMP, ROTOR_Q_HOLD,
               ROTOR_Q_DOWN, ROTOR_DONE, ROTOR_FAULT } rotor_state_t;
typedef enum { ROTOR_OK, ROTOR_ERR_CURRENT, ROTOR_ERR_SAMPLE, ROTOR_ERR_LINK,
               ROTOR_ERR_ENCODER, ROTOR_ERR_HARDWARE, ROTOR_ERR_SPEED,
               ROTOR_ERR_DIRECTION, ROTOR_ERR_TRACKING, ROTOR_ERR_ZERO,
               ROTOR_ERR_NOT_CALIBRATED, ROTOR_ERR_TRAVEL } rotor_fault_t;

typedef struct {
    rotor_state_t state; rotor_fault_t fault;
    uint32_t started_ms, entered_ms, heartbeat_ms, hold_ms;
    float amplitude, id_target, iq_target, kp, ki;
    float field, field_start, mech_start, mech_unwrapped, last_mech;
    float sweep_delta, zero_mech, zero_elec, id, iq, vd, vq;
    int8_t current_sign_a, current_sign_b, encoder_direction;
    uint8_t valid, calibrated, limited, phase;
    uint32_t samples, invalid_samples;
} rotor_test_t;

void rotor_test_init(rotor_test_t *r);
int rotor_test_active(const rotor_test_t *r);
void rotor_test_stop(rotor_test_t *r);
void rotor_test_fail(rotor_test_t *r, rotor_fault_t f);
int rotor_test_start_cal(rotor_test_t *r, uint32_t now, float amplitude,
                         uint8_t pole_pairs, uint8_t encoder_bits);
int rotor_test_start_q(rotor_test_t *r, uint32_t now, int iq_ma, uint32_t hold_ms,
                       float kp, float ki);
void rotor_test_step(rotor_test_t *r, uint32_t now, float mech, int enc_ok,
                     float rpm, const current_sense_t *c, uint8_t pole_pairs,
                     int8_t configured_direction);
void rotor_test_voltage(const rotor_test_t *r, float mech, uint8_t pole_pairs,
                        float *alpha, float *beta);
float rotor_test_electrical_angle(const rotor_test_t *r, float mech, uint8_t pole_pairs);
#endif
