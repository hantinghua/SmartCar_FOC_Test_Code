#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* MENC15A 磁编码器：SPI Mode 0，15 位角度（32768/圈）
 *   读角度：发 0x8021，回 16 位，取低 15 位
 *   读转速：发 0x8032，回 16 位，取低 15 位后按需换算
 */

/* 编码器编号：0 = 编码器1(SPI1/PA15)，1 = 编码器2(SPI4/PE11) */
#define ENC_IDX_1   0u
#define ENC_IDX_2   1u

/* 原始角度 0~32767 */
uint16_t encoder_read_raw(uint8_t idx);
void encoder_set_bits(uint8_t bits);
extern volatile uint8_t g_enc_last_ok;

/* 1 = HAL SPI 传输出错 */
extern volatile uint8_t g_enc_spi_err;

/* 调试用：最新读到的原始角度 */
extern volatile uint16_t g_enc1_raw;
extern volatile uint16_t g_enc2_raw;
extern volatile uint16_t g_enc1_raw16;

/* 调试用：机械角度（0~360 度） */
extern volatile float g_enc1_deg;
extern volatile float g_enc2_deg;

/* 机械角度（弧度 0~2π） */
float encoder_read_angle_rad(uint8_t idx);

/* 机械角度对应的 15 位原始值换算到 0~1 */
float encoder_read_angle_norm(uint8_t idx);

/* 转速原始值（带符号，15 位） */
int16_t encoder_read_speed_raw(uint8_t idx);

/* 连续读两次，判断 SPI 通信是否正常：返回 1 = 有变化（正常），0 = 一直不变 */
uint8_t encoder_check_alive(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
