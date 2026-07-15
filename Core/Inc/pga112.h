/**
  ******************************************************************************
  * @file           : pga112.h
  * @brief          : PGA112 可编程增益放大器驱动 + RMS 反馈 AGC (自动增益)
  *
  *  硬件连接 (接收端 02 / 半双工 06 一致):
  *    SPI2: PB13=SCK, PB15=MOSI (Simplex_Bidirectional_Master, 只发不收)
  *    CS  : PB12 (PG112_CS, 软件控制, 默认 High)
  *    SPI Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit.
  *
  *  PGA112 (binary gain) 增益档 (码 0..7):
  *    0->1x  1->2x  2->4x  3->8x  4->16x  5->32x  6->64x  7->128x
  *  已对照 TI PGA112 数据手册 (SBOS424C) Table 3 核对.
  *    写增益命令 WRITE : 2 字节 = { 0x2A, (G3G2G1G0<<4)|(CH3CH2CH1CH0) }
  *    退出关机 SDN_DIS: 2 字节 = { 0xE1, 0x00 }
  *
  *  AGC 策略 (RMS-based + 两端调整):
  *    - 监听态: RMS 持续偏高(>1100)降档, 持续偏低(<300)升档, 连续 4 个半缓冲延迟.
  *    - 锁帧后: 进入 PREAMBLE 后仍可调增益, 但仅逐档调(每次 ×2 或 ÷2), 不过度。
  *    - 削顶保护: 任一采样触边(≤48 或 ≥4047) 即时降一档.
  *    - RMS 计算用当前块动态 DC 均值 (不是固定 2048), 因为 OPA1642 偏置≈1.5V.
  ******************************************************************************
  */
#ifndef __PGA112_H
#define __PGA112_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* PGA112 协议常量 (Table 3/5/6) */
#define PGA_CMD_WRITE      0x2AU
#define PGA_CMD_SDN_DIS    0xE1U
#define PGA_CHANNEL        0x00U

/* 增益档码 0..7 */
#define PGA_GAIN_1X        0U
#define PGA_GAIN_2X        1U
#define PGA_GAIN_4X        2U
#define PGA_GAIN_8X        3U
#define PGA_GAIN_16X       4U
#define PGA_GAIN_32X       5U
#define PGA_GAIN_64X       6U
#define PGA_GAIN_128X      7U

#define PGA_GAIN_MIN_CODE  0U
#define PGA_GAIN_MAX_CODE  7U
#define PGA_GAIN_INIT_CODE PGA_GAIN_32X

/* AGC 门限 (RMS-based, 动态 DC = sum/N)
 *   监听态: 保守调节, 每次只调 1 档(×2), 4 个半缓冲延迟.
 *   锁帧后: 快速拉满摆幅~2.8Vpp, 弱信号一次跳 2 档(×4), 2 个半缓冲延迟. */
#define AGC_RMS_TARGET       650U   /* 目标 RMS (32x ref) */
#define AGC_RMS_HIGH        1100U   /* 偏高 → 降档 */
#define AGC_RMS_LOW          300U   /* 监听态偏低 → 升 1 档 */
#define AGC_RMS_FRAME_LOW    700U   /* 锁帧后偏低 → 大幅升档 */
#define AGC_CLIP_LOW          48U
#define AGC_CLIP_HIGH       4047U
#define AGC_HOLD_BLOCKS        4U   /* 监听态升降延迟 (100ms) */
#define AGC_HOLD_BLOCKS_FAST   2U   /* 锁帧后升降延迟 (50ms) */

void    PGA112_Init(void);
void    PGA112_SetGain(uint8_t gain_code);
uint8_t PGA112_GetGain(void);
extern volatile uint8_t g_pga_gain_live;
uint8_t PGA112_AGC_Update(const uint16_t *samples, uint16_t len, uint8_t frame_active);
void    PGA112_AGC_Reset(void);

#ifdef __cplusplus
}
#endif
#endif /* __PGA112_H */
