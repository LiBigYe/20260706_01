/**
  ******************************************************************************
  * @file           : voice_dsp.h
  * @brief          : 声语信使 v5 接收 DSP 核心 (可移植, 收发端/PC 共用)
  *
  *  以 80-sample (5ms) 块为单位驱动的接收状态机, 与 STM32 的 DMA
  *  HT/TC 半缓冲拆分完全一致 → 同一份代码即固件代码.
  *
  *  针对 DSP 复核 5 大缺陷的对策:
  *   (1) 无绝对峰峰值硬门限. 频率判决只看相对频谱置信度.
  *   (2) 一次性同步: 前导确认 + 1800Hz 同步音锁定符号栅格, 之后按
  *       固定 480-sample(30ms) 栅格自由运行, 不再依赖每符号 guard 下降沿.
  *       取样窗口是每符号 tone 的中间 10ms (block 1..2), 前后各留 5ms
  *       余量吸收 ±5ms 栅格抖动与混响拖尾.
  *   (3) 能量检测用一阶差分 sum|x[n]-x[n-1]| (等效高通), 对 DC/50Hz 免疫;
  *       Goertzel 前按窗口均值去 DC.
  *   (4) Goertzel 对每个目标频率在 ±1 bin 邻域取最大, 容忍晶振/多普勒频偏;
  *       峰值比一票否决放宽为 1.15x.
  *   (5) 输出符号数组交给 voice_fec 做 Hamming+CRC (本文件不含 FEC).
  ******************************************************************************
  */
#ifndef __VOICE_DSP_H
#define __VOICE_DSP_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "voice_proto.h"
#include "voice_fec.h"

#define VD_BLOCK        80U     /* 5ms 处理块 */
#define VD_BLOCKS_TONE   4U     /* 20ms tone = 4 块 */
#define VD_BLOCKS_GUARD  2U     /* 10ms guard = 2 块 */
#define VD_BLOCKS_SLOT   6U     /* 30ms 槽 = 6 块 */
#define VD_WIN          160U    /* Goertzel 判决窗 (中间 10ms, k 全整数) */
#define VD_WIN_OFFSET    80U    /* 窗在 tone 内的起点 (跳过第 1 块 onset) */

/* 接收状态 */
#define VD_LISTEN     0U
#define VD_PREAMBLE   1U
#define VD_DATA       2U
#define VD_DONE       3U

typedef struct {
    uint8_t  state;

    /* 能量 / 噪声自适应 (一阶差分能量) */
    uint32_t noise_floor;      /* 背景差分能量基线 */
    uint32_t last_energy;
    uint16_t hi_run;           /* 连续 HI 块 */
    uint16_t lo_run;
    uint16_t startup_quiet;    /* 上电静稳块计数 */

    /* 前导导频观察 */
    uint16_t sample_pos;       /* 自进入 PREAMBLE 起的样本计数 */
    uint16_t block_in_pre;
    uint8_t  pilot_last;       /* 上次导频 digit */
    uint8_t  pilot_trans;      /* 1500/2400 交替次数 */
    uint16_t pre_timeout;

    /* 数据栅格自由运行 */
    uint16_t slot_block;       /* 当前槽内块号 0..5 */
    uint16_t win_fill;         /* 判决窗已填样本 */
    uint16_t win_buf[VD_WIN];  /* 判决窗采样 */
    uint16_t sym_count;        /* 已解符号数 */
    uint16_t sym_expected;     /* 由 LEN 前缀求得的期望符号数 (0=未知) */
    uint8_t  symbols[VP_MAX_DATA_SYMBOLS];

    /* 结果 */
    uint8_t  payload[VP_MAX_PAYLOAD_BYTES];
    uint8_t  payload_len;
    uint8_t  crc_ok;

    /* 诊断 (可读, 不参与判决) */
    uint8_t  last_digit;
    float    last_conf;
} VoiceRx;

/* Goertzel: 对 win[0..N-1] 去 DC 后计算 4 个目标频率(±1 bin)的幅度²,
 * 返回主频 digit(0..3), 若置信度不足返回 0xFF. conf_out 可为 NULL. */
uint8_t VoiceDSP_Classify(const uint16_t *win, uint16_t N, float *conf_out);

/* 一阶差分能量 (高通, 免疫 DC/50Hz) */
uint32_t VoiceDSP_DiffEnergy(const uint16_t *blk, uint16_t n);

void     VoiceRx_Init(VoiceRx *rx);
void     VoiceRx_Start(VoiceRx *rx);
/* 送入一个 80-sample 块, 推进状态机. 返回 1 表示本块后 state==VD_DONE. */
uint8_t  VoiceRx_PushBlock(VoiceRx *rx, const uint16_t *blk, uint8_t gain_code);
/* DONE 后取结果: payload+len, crc_ok. */

#ifdef __cplusplus
}
#endif
#endif /* __VOICE_DSP_H */
