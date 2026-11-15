#ifndef SENSOR_DIAG_H
#define SENSOR_DIAG_H
/* Current hardware: test the user's previously working 14-bit interpretation.
 * Seekfree's reference uses 15 bits. Validate exactly one wrap per shaft turn. */
#define SENSOR_DIAG_ENCODER_BITS 14u
void sensor_diag_init(void);
void sensor_diag_poll(void);
#endif
