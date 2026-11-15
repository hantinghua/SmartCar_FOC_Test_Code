#include "rotor_test.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979323846f
#define TWO_PI_F 6.2831853071795864769f
#define DT_F 0.00005f
#define Q_V_LIMIT 0.005f
#define PHASE_LIMIT 0.5f
#define RPM_LIMIT 60.0f

static float wrap(float x) { while (x > PI_F) x -= TWO_PI_F; while (x < -PI_F) x += TWO_PI_F; return x; }
static float uwrap(float d) { return wrap(d); }
static void enter(rotor_test_t *r, rotor_state_t s, uint32_t now) { r->state=s; r->entered_ms=now; r->samples=0; r->invalid_samples=0; }

void rotor_test_init(rotor_test_t *r) { memset(r,0,sizeof(*r)); r->state=ROTOR_IDLE; r->fault=ROTOR_OK; r->kp=.015f; r->ki=.3f; r->amplitude=.005f; }
int rotor_test_active(const rotor_test_t *r) { return r->state!=ROTOR_IDLE && r->state!=ROTOR_DONE && r->state!=ROTOR_FAULT; }
void rotor_test_stop(rotor_test_t *r) { r->state=ROTOR_IDLE; r->valid=0; }
void rotor_test_fail(rotor_test_t *r, rotor_fault_t f) { r->fault=f; r->state=ROTOR_FAULT; r->valid=0; }

int rotor_test_start_cal(rotor_test_t *r, uint32_t now, float amplitude, uint8_t pp, uint8_t bits)
{
    (void)bits; if (rotor_test_active(r) || pp<1 || pp>30 || amplitude<.001f || amplitude>.018f) return 0;
    rotor_test_init(r); r->started_ms=now; r->entered_ms=now; r->amplitude=amplitude; r->phase=0; r->valid=1; r->state=ROTOR_PREALIGN; return 1;
}
int rotor_test_start_q(rotor_test_t *r, uint32_t now, int iq_ma, uint32_t hold_ms, float kp, float ki)
{
    if (rotor_test_active(r) || !r->calibrated || iq_ma < -200 || iq_ma > 200 || hold_ms<100 || hold_ms>2000 || kp<0 || kp>.06f || ki<0 || ki>1.2f) return 0;
    r->started_ms=now; r->entered_ms=now; r->hold_ms=hold_ms; r->id_target=0; r->iq_target=(float)iq_ma/1000.0f; r->kp=kp; r->ki=ki; r->vd=0; r->vq=0; r->state=ROTOR_Q_RAMP; r->valid=1; return 1;
}

void rotor_test_step(rotor_test_t *r, uint32_t now, float mech, int enc_ok, float rpm,
                     const current_sense_t *c, uint8_t pp, int8_t configured_direction)
{
    float d, elapsed, target, ca, cb, ia, ib, iq_meas, id_meas;
    if (!rotor_test_active(r)) return;
    r->heartbeat_ms=now;
    if (!enc_ok) { rotor_test_fail(r,ROTOR_ERR_ENCODER); return; }
    if (fabsf(rpm)>RPM_LIMIT) { rotor_test_fail(r,ROTOR_ERR_SPEED); return; }
    if (!c || c->state!=CURRENT_READY || !c->valid) { if (++r->invalid_samples>1000u) rotor_test_fail(r,ROTOR_ERR_CURRENT); return; }
    r->invalid_samples=0; r->samples++;
    if (r->samples==1u) r->last_mech=mech;
    d=uwrap(mech-r->last_mech); r->mech_unwrapped += d; r->last_mech=mech;
    elapsed=(float)(now-r->entered_ms);
    if (r->state==ROTOR_PREALIGN && elapsed>=1000.0f) {
        r->mech_start=mech; r->mech_unwrapped=0.0f; r->last_mech=mech; r->field_start=0; enter(r,ROTOR_SWEEP_FWD,now); return;
    }
    if (r->state==ROTOR_SWEEP_FWD) {
        target=fminf(1.0f,elapsed/4000.0f); r->field=TWO_PI_F*target;
        if (elapsed>=4000.0f) { r->sweep_delta=r->mech_unwrapped; r->mech_start=r->mech_unwrapped; r->mech_unwrapped=0; r->last_mech=mech; enter(r,ROTOR_SWEEP_REV,now); }
    } else if (r->state==ROTOR_SWEEP_REV) {
        target=fminf(1.0f,elapsed/4000.0f); r->field=TWO_PI_F*(1.0f-target);
        if (elapsed>=4000.0f) {
            d=r->mech_unwrapped; if (fabsf(r->sweep_delta)<0.05f || fabsf(d)<0.05f || r->sweep_delta*d>0.0f) { rotor_test_fail(r,ROTOR_ERR_DIRECTION); return; }
            r->encoder_direction = r->sweep_delta>0 ? 1 : -1;
            if (configured_direction==1 || configured_direction==-1) r->encoder_direction=(int8_t)(r->encoder_direction*configured_direction);
            r->zero_mech=mech; enter(r,ROTOR_ZERO_HOLD,now);
        }
    } else if (r->state==ROTOR_ZERO_HOLD) {
        r->field=0; if (elapsed>=300u) { r->zero_elec=wrap((float)r->encoder_direction*pp*r->zero_mech); r->calibrated=1; r->valid=1; r->state=ROTOR_DONE; }
    } else if (r->state==ROTOR_Q_RAMP || r->state==ROTOR_Q_HOLD || r->state==ROTOR_Q_DOWN) {
        float t;
        if (r->state==ROTOR_Q_RAMP) { t=fminf(1.0f,elapsed/500.0f); r->id_target=0; r->iq=r->iq_target*t; if (elapsed>=500.0f) enter(r,ROTOR_Q_HOLD,now); }
        else if (r->state==ROTOR_Q_HOLD) { r->iq=r->iq_target; if (elapsed>=r->hold_ms) enter(r,ROTOR_Q_DOWN,now); }
        else { t=fmaxf(0.0f,1.0f-elapsed/200.0f); r->iq=r->iq_target*t; if (elapsed>=200.0f) { r->state=ROTOR_DONE; r->vd=r->vq=0; } }
        ca=cosf(rotor_test_electrical_angle(r,mech,pp)); cb=sinf(rotor_test_electrical_angle(r,mech,pp));
        ia=c->ia; ib=(c->ia+2.0f*c->ib)*0.5773502692f; id_meas=ia*ca+ib*cb; iq_meas=-ia*cb+ib*ca; r->id=id_meas; r->iq=iq_meas;
        r->vd += r->kp*(r->id_target-id_meas) + r->ki*(r->id_target-id_meas)*DT_F;
        r->vq += r->kp*(r->iq_target-iq_meas) + r->ki*(r->iq_target-iq_meas)*DT_F;
        if (r->vd>Q_V_LIMIT) r->vd=Q_V_LIMIT; if (r->vd<-Q_V_LIMIT) r->vd=-Q_V_LIMIT;
        if (r->vq>Q_V_LIMIT) r->vq=Q_V_LIMIT; if (r->vq<-Q_V_LIMIT) r->vq=-Q_V_LIMIT;
    }
    if (fabsf(c->ia)>PHASE_LIMIT || fabsf(c->ib)>PHASE_LIMIT || fabsf(c->ic)>PHASE_LIMIT) rotor_test_fail(r,ROTOR_ERR_CURRENT);
}
float rotor_test_electrical_angle(const rotor_test_t *r, float mech, uint8_t pp) { return wrap((float)r->encoder_direction*pp*mech-r->zero_elec); }
void rotor_test_voltage(const rotor_test_t *r, float mech, uint8_t pp, float *alpha, float *beta)
{
    float th;
    if (!r || !alpha || !beta) return;
    if (r->state==ROTOR_SWEEP_FWD || r->state==ROTOR_SWEEP_REV || r->state==ROTOR_PREALIGN || r->state==ROTOR_ZERO_HOLD) th=r->field;
    else th=rotor_test_electrical_angle(r,mech,pp);
    if (r->state==ROTOR_Q_RAMP || r->state==ROTOR_Q_HOLD || r->state==ROTOR_Q_DOWN) {
        *alpha=r->vd*cosf(th)-r->vq*sinf(th); *beta=r->vd*sinf(th)+r->vq*cosf(th);
    } else { *alpha=r->amplitude*cosf(th); *beta=r->amplitude*sinf(th); }
}
