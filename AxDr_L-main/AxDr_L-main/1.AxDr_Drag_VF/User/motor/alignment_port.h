#ifndef ALIGNMENT_PORT_H
#define ALIGNMENT_PORT_H
#include "alignment.h"
#include "current_sense.h"
#include "voltage_test.h"
#include "current_loop.h"
#include "rotor_test.h"
extern current_loop_t g_current_loop;
extern voltage_test_t g_voltage_test;
extern alignment_t g_alignment;
extern current_sense_t g_current;
extern rotor_test_t g_rotor_test;
extern volatile uint32_t g_sample_hz;
extern volatile uint32_t g_isr_cnt;
extern volatile uint16_t g_ia_raw, g_ib_raw;
extern volatile float g_foc_rpm;
void alignment_port_init(int hardware_ok);
void alignment_port_poll(void);
void alignment_port_stop(void);
int alignment_port_zero_current(void);
int alignment_port_outputs_on(void);
int alignment_port_active(void);
void alignment_link_init(void);
void alignment_link_poll(void);
#endif
