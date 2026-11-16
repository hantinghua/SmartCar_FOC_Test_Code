#ifndef BOARD_CHECK_H
#define BOARD_CHECK_H
#include "main.h"
void board_check_init(void);
void board_check_poll(void);
void board_check_tick(void);
void board_check_command(uint8_t command);
void board_check_report(uint32_t angle_centideg, uint8_t encoder_ok);
uint8_t board_check_fault(void);
uint8_t board_check_chart(void);
#endif
