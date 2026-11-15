#include "common.h"
#include "alignment_port.h"
#include "encoder.h"
#include "usart.h"
#include <limits.h>

/* ASCII frames: @sequence,VERB[,decimal integers]*CRC16\n.
 * CRC-16/CCITT-FALSE covers only text between @ and *, initial 0xffff.
 * UART DMA owns all TX buffers until its completion callback. */
#define RX_SIZE 512u
#define LINE_SIZE 240u
#define QUEUE_SIZE 8u
static uint8_t rx[RX_SIZE];
static char line[LINE_SIZE], queue[QUEUE_SIZE][LINE_SIZE], tx[LINE_SIZE];
static uint16_t rx_at, used;
static uint8_t head, tail, discard;
static volatile uint8_t uart_failed;
static uint32_t telemetry_tick, loop_telemetry_tick;
extern uint16_t adc1_buff[2];

static uint16_t crc16(const char *s, uint32_t n)
{
    uint16_t crc = 0xffffu;
    for (uint32_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)((uint8_t)s[i] << 8);
        for (uint32_t j = 0; j < 8u; ++j)
            crc = (uint16_t)((crc & 0x8000u) ? (crc << 1) ^ 0x1021u : crc << 1);
    }
    return crc;
}

static void enqueue(const char *payload)
{
    uint8_t next = (uint8_t)((head + 1u) % QUEUE_SIZE);
    if (next == tail) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (rotor_test_active(&g_rotor_test)) rotor_test_fail(&g_rotor_test, ROTOR_ERR_LINK);
        else if (current_loop_active(&g_current_loop)) current_loop_fail(&g_current_loop, ILOOP_ERR_LINK);
        else if (voltage_test_active(&g_voltage_test)) voltage_test_fail(&g_voltage_test, VTEST_ERR_LINK);
        else alignment_fail(&g_alignment, ALIGN_ERR_LINK);
        __set_PRIMASK(mask);
        return;
    }
    (void)snprintf(queue[head], LINE_SIZE, "@%s*%04X\n", payload,
                   (unsigned)crc16(payload, (uint32_t)strlen(payload)));
    head = next;
}

static void config_report(uint32_t seq)
{
    char out[220];
    align_config_t c;
    uint32_t mask = __get_PRIMASK();
    __disable_irq(); c = g_alignment.cfg; __set_PRIMASK(mask);
    (void)snprintf(out, sizeof(out), "%lu,CONFIG,%d,%d,%d,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%d,%u,%u",
        (unsigned long)seq, (int)lroundf(c.align_u * 1000.0f), (int)lroundf(c.run_u * 1000.0f),
        (int)lroundf(c.stable_deg * 1000.0f), (int)c.rpm_limit,
        (unsigned long)c.ramp_ms, (unsigned long)c.hold_ms, (unsigned long)c.stable_ms,
        (unsigned long)c.settle_timeout_ms, (unsigned long)c.pause_ms,
        (unsigned long)c.run_ramp_ms, (unsigned long)c.run_ms,
        c.pole_pairs, c.direction, c.encoder_bits, c.auto_run);
    enqueue(out);
}

static void command(char *s)
{
    char *star, *p, *end, *verb;
    long values[15], seq, crc;
    unsigned count = 0u;
    char answer[96];
    const char *error = NULL;
    uint32_t mask;
    align_config_t c;
    if (s[0] != '@' || (star = strchr(s, '*')) == NULL || strlen(star + 1) != 4u) return;
    crc = strtol(star + 1, &end, 16);
    if (*end || crc < 0 || crc > 65535 || crc16(s + 1, (uint32_t)(star - s - 1)) != (uint16_t)crc) return;
    *star = '\0';
    seq = strtol(s + 1, &end, 10);
    if (end == s + 1 || *end != ',' || seq <= 0 || seq > 65535) return;
    verb = end + 1;
    p = strchr(verb, ',');
    if (p) {
        *p++ = '\0';
        for (;;) {
            if (count >= 15u || !*p) return;
            values[count] = strtol(p, &end, 10);
            if (end == p || (*end && *end != ',') || values[count] < -100000 || values[count] > 100000) return;
            count++;
            if (!*end) break;
            p = end + 1;
        }
    }
    if (!strcmp(verb, "RCAL") && (count == 1u || count == 2u)) {
        mask = __get_PRIMASK(); __disable_irq();
        if (alignment_port_active() || alignment_port_outputs_on()) error = "BUSY";
        else if (g_current.state != CURRENT_READY || !g_current.valid) error = "CURRENT_NOT_READY";
        else if (g_sample_hz < 18000u || g_sample_hz > 22000u) error = "SAMPLE_RATE";
        else if (!g_enc_last_ok) error = "ENCODER";
        else if (!rotor_test_start_cal(&g_rotor_test, HAL_GetTick(), (float)values[0]/1000.0f,
                                       g_alignment.cfg.pole_pairs, g_alignment.cfg.encoder_bits)) error = "RANGE_OR_STATE";
        __set_PRIMASK(mask);
    } else if (!strcmp(verb, "QTEST") && count == 2u) {
        mask = __get_PRIMASK(); __disable_irq();
        if (alignment_port_active() || alignment_port_outputs_on()) error = "BUSY";
        else if (!g_rotor_test.calibrated) error = "NOT_CALIBRATED";
        else if (g_current.state != CURRENT_READY || !g_current.valid) error = "CURRENT_NOT_READY";
        else if (!rotor_test_start_q(&g_rotor_test, HAL_GetTick(), (int)values[0], (uint32_t)values[1], ILOOP_KP, ILOOP_KI)) error = "RANGE_OR_STATE";
        __set_PRIMASK(mask);
    } else if (!strcmp(verb, "CFG") && count == 15u) {
        memset(&c, 0, sizeof(c));
        c.align_u = (float)values[0] / 1000.0f;
        c.run_u = (float)values[1] / 1000.0f;
        c.stable_deg = (float)values[2] / 1000.0f;
        c.rpm_limit = (float)values[3];
        c.ramp_ms = (uint32_t)values[4]; c.hold_ms = (uint32_t)values[5];
        c.stable_ms = (uint32_t)values[6]; c.settle_timeout_ms = (uint32_t)values[7];
        c.pause_ms = (uint32_t)values[8]; c.run_ramp_ms = (uint32_t)values[9]; c.run_ms = (uint32_t)values[10];
        /* Check BEFORE narrowing to 8-bit fields. */
        if (values[11] < 1 || values[11] > 30 || (values[12] != -1 && values[12] != 1) ||
            (values[13] != 14 && values[13] != 15) || (values[14] != 0 && values[14] != 1)) error = "RANGE";
        else {
            c.pole_pairs = (uint8_t)values[11]; c.direction = (int8_t)values[12];
            c.encoder_bits = (uint8_t)values[13]; c.auto_run = (uint8_t)values[14];
            if (!alignment_config_valid(&c)) error = "RANGE";
        }
        mask = __get_PRIMASK(); __disable_irq();
        if (alignment_port_active()) error = "BUSY";
        if (!error) { g_alignment.cfg = c; g_alignment.zero_valid = 0u; g_rotor_test.calibrated = 0u; }
        __set_PRIMASK(mask);
    } else if (!strcmp(verb, "VTEST") && count == 5u) {
        mask = __get_PRIMASK(); __disable_irq();
        if (alignment_port_active() || alignment_port_outputs_on()) error = "BUSY";
        else if (g_alignment.fault != ALIGN_OK) error = "STATE";
        else if (g_current_loop.fault != ILOOP_OK) error = "LOOP_FAULT";
        else if (g_current.state != CURRENT_READY || !g_current.valid) error = "CURRENT_NOT_READY";
        else if (g_sample_hz < 18000u || g_sample_hz > 22000u) error = "SAMPLE_RATE";
        else if (!g_enc_last_ok || g_isr_cnt < 100u) error = "ENCODER";
        else if (!voltage_test_start(&g_voltage_test, HAL_GetTick(), (int)values[0],
                     (uint32_t)values[1], (uint32_t)values[2], (uint32_t)values[3], (int)values[4])) error = "RANGE_OR_STATE";
        __set_PRIMASK(mask);
    } else if (!strcmp(verb, "ITEST") && (count == 2u || count == 4u)) {
        mask = __get_PRIMASK(); __disable_irq();
        if (alignment_port_active() || alignment_port_outputs_on()) error = "BUSY";
        else if (g_alignment.fault != ALIGN_OK || g_voltage_test.fault != VTEST_OK) error = "STATE";
        else if (g_current.state != CURRENT_READY || !g_current.valid) error = "CURRENT_NOT_READY";
        else if (g_sample_hz < 18000u || g_sample_hz > 22000u) error = "SAMPLE_RATE";
        else if (!g_enc_last_ok || g_isr_cnt < 100u) error = "ENCODER";
        else if (!current_loop_start_gains(&g_current_loop, HAL_GetTick(), (int)values[0],
                     (uint32_t)values[1], count == 4u ? (float)values[2] / 10000.0f : ILOOP_KP,
                     count == 4u ? (float)values[3] / 10000.0f : ILOOP_KI)) error = "RANGE_OR_STATE";
        __set_PRIMASK(mask);
    } else if (count != 0u) error = "ARGS";
    else if (!strcmp(verb, "STOP")) {
        mask = __get_PRIMASK(); __disable_irq(); alignment_port_stop(); __set_PRIMASK(mask);
    } else if (!strcmp(verb, "CLEAR")) {
        mask = __get_PRIMASK(); __disable_irq();
        if (alignment_port_active()) error = "BUSY";
        else if (g_alignment.fault == ALIGN_ERR_HARDWARE) error = "HARDWARE";
        else {
            alignment_port_stop(); g_alignment.fault = ALIGN_OK;
            voltage_test_init(&g_voltage_test);
            current_loop_init(&g_current_loop);
        }
        __set_PRIMASK(mask);
    } else if (!strcmp(verb, "START")) {
        mask = __get_PRIMASK(); __disable_irq();
        if (voltage_test_active(&g_voltage_test) || current_loop_active(&g_current_loop)) error = "BUSY";
        else if (g_current_loop.fault != ILOOP_OK) error = "LOOP_FAULT";
        else if (g_voltage_test.fault != VTEST_OK) error = "TEST_FAULT";
        else if (g_current.state != CURRENT_READY || !g_current.valid) error = "CURRENT_NOT_READY";
        else if (g_sample_hz < 18000u || g_sample_hz > 22000u) error = "SAMPLE_RATE";
        else if (!g_enc_last_ok || g_isr_cnt < 100u) error = "ENCODER";
        else if (!alignment_start(&g_alignment, HAL_GetTick())) error = "STATE";
        __set_PRIMASK(mask);
    } else if (!strcmp(verb, "IZERO")) {
        mask = __get_PRIMASK(); __disable_irq();
        if (!alignment_port_zero_current()) error = "BUSY";
        else g_rotor_test.calibrated = 0u;
        __set_PRIMASK(mask);
    } else if (!strcmp(verb, "GET")) { config_report((uint32_t)seq); return; }
    else if (!strcmp(verb, "HELLO")) {
        (void)snprintf(answer, sizeof(answer), "%ld,HELLO,ALIGN5", seq); enqueue(answer); return;
    } else if (!strcmp(verb, "PING")) {
        mask = __get_PRIMASK(); __disable_irq();
        g_alignment.heartbeat_ms = g_voltage_test.heartbeat_ms = g_current_loop.heartbeat_ms = g_rotor_test.heartbeat_ms = HAL_GetTick();
        __set_PRIMASK(mask);
    } else error = "COMMAND";
    (void)snprintf(answer, sizeof(answer), error ? "%ld,ERR,%s" : "%ld,OK,%s", seq, error ? error : verb);
    enqueue(answer);
}

static void receive_start(void)
{
    rx_at = used = 0u; discard = 0u;
    if (HAL_UART_Receive_DMA(&huart3, rx, RX_SIZE) != HAL_OK) { uart_failed = 1u; return; }
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT | DMA_IT_TC);
    uart_failed = 0u;
}

void alignment_link_init(void) { receive_start(); }

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart3) uart_failed = 1u;
}

void alignment_link_poll(void)
{
    uint16_t available;
    uint32_t now = HAL_GetTick();
    if (uart_failed) {
        uint32_t mask = __get_PRIMASK(); __disable_irq();
        if (current_loop_active(&g_current_loop)) current_loop_fail(&g_current_loop, ILOOP_ERR_LINK);
        else if (voltage_test_active(&g_voltage_test)) voltage_test_fail(&g_voltage_test, VTEST_ERR_LINK);
        else if (alignment_active(&g_alignment)) alignment_fail(&g_alignment, ALIGN_ERR_LINK);
        __set_PRIMASK(mask);
        (void)HAL_UART_AbortReceive(&huart3);
        receive_start();
    }
    available = (uint16_t)(RX_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx));
    if (available == RX_SIZE) available = 0u;
    while (rx_at != available) {
        char ch = (char)rx[rx_at]; rx_at = (uint16_t)((rx_at + 1u) % RX_SIZE);
        if (ch == '\n') {
            if (!discard && used) { line[used] = '\0'; command(line); }
            used = 0u; discard = 0u;
        } else if (ch != '\r' && !discard) {
            if (used < LINE_SIZE - 1u) line[used++] = ch;
            else { discard = 1u; used = 0u; }
        }
    }
    if (now - telemetry_tick >= 50u && head == tail && huart3.gState == HAL_UART_STATE_READY) {
        static alignment_t a;
        static current_sense_t c;
        static voltage_test_t v;
        static current_loop_t l;
        static rotor_test_t r;
        static char out[220];
        float deg, rpm;
        uint32_t hz;
        int on;
        uint16_t raw, raw16, ia, ib;
        uint32_t mask = __get_PRIMASK(); __disable_irq();
        a = g_alignment; deg = g_enc1_deg; raw = g_enc1_raw; raw16 = g_enc1_raw16;
        ia = g_ia_raw; ib = g_ib_raw; rpm = g_foc_rpm;
        c = g_current; v = g_voltage_test; l = g_current_loop; r = g_rotor_test;
        hz = g_sample_hz; on = alignment_port_outputs_on();
        __set_PRIMASK(mask);
        telemetry_tick = now;
        /* L at 10 Hz reserves bandwidth for existing V/C/T at 20 Hz on 115200.
         * It shares THIS snapshot's timestamp; absent L rows remain blank. */
        if (now - loop_telemetry_tick >= 100u) {
            loop_telemetry_tick = now;
            (void)snprintf(out, sizeof(out), "0,L,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%d,%d,%ld,%ld,%ld,%lu,%ld,%lu,%lu",
                (unsigned long)now, (unsigned)l.state, (unsigned)l.fault,
                (long)lroundf(l.target * 1000.0f), (long)lroundf(l.reference * 1000.0f),
                (long)lroundf(l.alpha * 1000.0f), (long)lroundf(l.beta * 1000.0f), (long)lroundf(l.error * 1000.0f),
                (long)lroundf(l.out_alpha * 1000000.0f), (long)lroundf(l.out_beta * 1000000.0f),
                5000u, (unsigned)l.limited, (unsigned)l.valid, (int)l.polarity_alpha, (int)l.polarity_beta,
                (long)lroundf(l.mean_alpha * 1000.0f), (long)lroundf(l.mean_beta * 1000.0f), (long)lroundf(l.mean_error * 1000.0f),
                (unsigned long)l.mean_samples, (long)lroundf(l.peak * 1000.0f),
                (unsigned long)l.valid_samples, (unsigned long)l.invalid_samples);
            enqueue(out);
            (void)snprintf(out, sizeof(out), "0,R,%lu,%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%u,%u",
                (unsigned long)now, (unsigned)r.state, (unsigned)r.fault,
                (long)lroundf(r.field*1000.0f), (long)lroundf(r.mech_unwrapped*1000.0f),
                (long)lroundf(r.zero_mech*1000.0f), (long)lroundf(r.zero_elec*1000.0f),
                (long)lroundf(r.id*1000.0f), (long)lroundf(r.iq*1000.0f),
                (long)lroundf(r.iq_target*1000.0f), (long)lroundf(r.vd*1000000.0f),
                (long)lroundf(r.vq*1000000.0f), (int)r.encoder_direction,
                (unsigned)r.calibrated, (unsigned)r.valid);
            enqueue(out);
        }
        /* V, C, T describe the same atomic snapshot/time. Original T unchanged. */
        (void)snprintf(out, sizeof(out), "0,V,%lu,%u,%u,%ld,%ld,%lu,%lu,%ld,%ld,%ld,%ld,%lu,%lu",
            (unsigned long)now, (unsigned)v.state, (unsigned)v.fault,
            (long)lroundf(v.command * 1000.0f), (long)lroundf(v.target * 1000.0f),
            (unsigned long)v.ramp_ms, (unsigned long)v.hold_ms, (long)lroundf(v.limit_a * 1000.0f),
            (long)lroundf(v.peak_a * 1000.0f), (long)lroundf(v.peak_b * 1000.0f),
            (long)lroundf(v.peak_c * 1000.0f), (unsigned long)v.valid_samples,
            (unsigned long)v.invalid_samples);
        enqueue(out);
        (void)snprintf(out, sizeof(out), "0,C,%lu,%u,%lu,%ld,%ld,%u,%u,%ld,%ld,%ld,%u,%lu,%u",
            (unsigned long)now, (unsigned)c.state, (unsigned long)c.count,
            (long)lroundf(c.offset_a * 1000.0f), (long)lroundf(c.offset_b * 1000.0f),
            (unsigned)(c.max_a >= c.min_a ? c.max_a - c.min_a : 0u),
            (unsigned)(c.max_b >= c.min_b ? c.max_b - c.min_b : 0u),
            (long)lroundf(c.ia * 1000.0f), (long)lroundf(c.ib * 1000.0f),
            (long)lroundf(c.ic * 1000.0f), (unsigned)c.valid,
            (unsigned long)hz, (unsigned)on);
        enqueue(out);
        (void)snprintf(out, sizeof(out), "0,T,%lu,%u,%u,%d,%u,%u,%d,%d,%d,%d,%d,%d,%u,%u,%u,%u",
            (unsigned long)now, (unsigned)a.state, (unsigned)a.fault, (int)(deg * 1000.0f), raw, raw16,
            (int)(rpm * 10.0f), (int)(a.zero_rad * 57295.7795f), (int)(a.zero_elec_deg * 1000.0f),
            (int)(a.span_deg * 1000.0f), (int)(a.vd * 1000.0f), (int)(a.vq * 1000.0f),
            ia, ib, adc1_buff[0], (unsigned)a.zero_valid);
        enqueue(out);
    }
    if (head != tail && huart3.gState == HAL_UART_STATE_READY) {
        (void)strcpy(tx, queue[tail]);
        if (HAL_UART_Transmit_DMA(&huart3, (uint8_t *)tx, (uint16_t)strlen(tx)) == HAL_OK)
            tail = (uint8_t)((tail + 1u) % QUEUE_SIZE);
    }
}
