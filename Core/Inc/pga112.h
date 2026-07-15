/**
  ******************************************************************************
  * @file           : pga112.h
  * @brief          : PGA112 可编程增益放大器驱动 + 基于 ADC 的 AGC (自动增益)
  *
  *  硬件连接 (接收端 02 / 半双工 06 一致):
  *    SPI2: PB13=SCK, PB15=MOSI (Simplex_Bidirectional_Master, 只发不收)
  *    CS  : PB12 (PG112_CS, 软件控制, 默认 High)
  *    SPI Mode 0 (CPOL=0, CPHA=0), MSB first, 8-bit.
  *
  *  PGA112 (binary gain) 增益档 (码 0..7):
  *    0→1x  1→2x  2→4x  3→8x  4→16x  5→32x  6→64x  7→128x
  *  ✓ 已对照 TI PGA112 数据手册 (SBOS424C) Table 3 核对:
  *    写增益命令 WRITE : 2 字节 = { 0x2A, (G3G2G1G0<<4)|(CH3CH2CH1CH0) }
  *    退出关机 SDN_DIS: 2 字节 = { 0xE1, 0x00 } (D15..D0=1110000100000000)
  *    POR 后寄存器全 0 (增益=1, 通道=VCAL/CH0), 无专用 reset 命令.
  *
  *  AGC 策略 (与 v5 频谱判决互补, 只扩动态范围, 不参与判决):
  *    - 只在接收监听态 (未锁帧) 调整增益; 锁帧 (PREAMBLE/DATA) 期间冻结.
  *    - 削顶保护优先: ADC 触及量程两端 → 立即降一档.
  *    - 弱信号提升: 峰峰值持续低于目标下限 → 升一档.
  *    - 目标: 峰峰值维持在满量程 ~40%~80%, 留削顶余量.
  ******************************************************************************
  */
#ifndef __PGA112_H
#define __PGA112_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* ---- PGA112 协议常量 (如与数据手册不符, 仅需改这里) ---- */
#define PGA_CMD_WRITE      0x2AU   /* WRITE 命令字节 (Table 3) */
#define PGA_CMD_SDN_DIS    0xE1U   /* SDN_DIS 退出关机高字节 (0xE1 0x00) */
#define PGA_CHANNEL        0x00U   /* 模拟输入通道: VCAL/CH0 = 0x0 (第3脚, 手册 Table 6) */

/* 增益档码 0..7 → 1/2/4/8/16/32/64/128 倍 */
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
#define PGA_GAIN_INIT_CODE PGA_GAIN_8X   /* 上电初始增益 (用户选定 8x) */

/* ---- AGC 门限 (基于 12-bit ADC, 中值 2048) ---- */
/* 削顶: 样本进入两端 saturation 区 (距轨 <64 counts) 即判削顶 */
#define AGC_CLIP_LOW        64U
#define AGC_CLIP_HIGH     4031U    /* 4095-64 */
/* 目标峰值 (相对中值的单边幅度 counts): 满量程单边 2048.
 *   过高 (>~82%): 降档; 过低 (<~22%): 升档. 中间不动 (迟滞). */
#define AGC_AMP_HIGH      1680U    /* ~82% 单边 → 降档 */
#define AGC_AMP_LOW        450U    /* ~22% 单边 → 升档 */
/* 连续多少个"过低"判定块后才升档 (避免噪声瞬时抬档); 削顶降档为即时. */
#define AGC_LOW_HOLD         8U

/* ---- API ---- */
/* 初始化: 复位 + 设为初始增益. 需在 MX_SPI2_Init 之后调用. */
void    PGA112_Init(void);
/* 直接设增益档 (0..7). 立即经 SPI2 写入. */
void    PGA112_SetGain(uint8_t gain_code);
/* 当前增益档 */
uint8_t PGA112_GetGain(void);

/* AGC: 送入一个 ADC 半缓冲 (len 个 12-bit 采样) 做增益评估.
 * frame_active!=0 时冻结增益 (只统计不调整). 在 ADC 回调里调用.
 * 返回 1 表示本次调整了增益. */
uint8_t PGA112_AGC_Update(const uint16_t *samples, uint16_t len, uint8_t frame_active);

/* 复位 AGC 内部计数 (进入接收/RX_Start 时调用, 可选) */
void    PGA112_AGC_Reset(void);

#ifdef __cplusplus
}
#endif
#endif /* __PGA112_H */
