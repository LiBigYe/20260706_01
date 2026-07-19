/**
  ******************************************************************************
  * @file           : voice_dsp.h
  * @brief          : 声语信使 v5.1 接收 DSP 核心 — 多窗口 Goertzel + 固定 SNR
  *
  *  以 80-sample (5ms) 块为单位驱动的接收状态机, 与 STM32 的 DMA
  *  HT/TC 半缓冲拆分完全一致 → 同一份代码即固件代码.
  *
  *  v5.1 关键改进 (2026-07-16):
  *   (a) 多窗口 Goertzel: 取满 20ms tone (320 samples), 做 3 个重叠窗
 *       (offset 40/80/120, N=160), 累加 mag² 以平滑判决统计量.
  *   (b) 数据段使用固定的最低 SNR 门限，不让前导段的局部驻波把后续
  *       符号的判决门限抬高。
  *   (c) 软判决输出: 每符号的 4 频 mag² 存入 sym_mag2[][] 供 voice_fec 做
  *       LLR + Chase 软判决解码.
  *
  *  保留 v5 基础架构:
  *   - 前导 + 同步音一次性锁定 30ms 符号栅格
  *   - 一阶差分能量唤醒 (免疫 DC/50Hz)
  *   - 双重频域锁 (防止噪声误唤醒)
  *   - 频域擦除计数 (替换 lo_run)
  *   - ±1 bin 频偏容忍
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

#define VD_BLOCK          80U     /* 5ms 处理块 */
#define VD_BLOCKS_TONE     4U     /* 20ms tone = 4 块 */
#define VD_BLOCKS_GUARD    2U     /* 10ms guard = 2 块 */
#define VD_BLOCKS_SLOT     6U     /* 30ms 槽 = 6 块 */
#define VD_WIN            160U    /* Goertzel 判决窗 (10ms, k 全整数) */
#define VD_TONE_SAMPLES   320U    /* 完整 20ms tone (供多窗口) */
#define VD_WIN_OFFSET       0U    /* v5.1: 从 tone 起点开始填充 */

/* ── v5.1 多窗口 Goertzel ── */
#define VD_MULTI_WIN_COUNT   3U
/* 3 个窗口在 320-sample tone 内的起点: */
/* offsets defined in voice_dsp.c */

/* 接收状态 */
#define VD_LISTEN     0U
#define VD_PREAMBLE   1U
#define VD_DATA       2U
#define VD_DONE       3U

#define VD_PREAMBLE_MIN_HI   6U
#define VD_AGC_FREEZE_TRANS  4U
#define VD_PREAMBLE_MAX_MISSES 3U

/* ── v5.1 固定 SNR 门限 ── */
#define VD_SNR_MIN         2.0f   /* 绝对 SNR 下限 (6dB) */

typedef struct {
    uint8_t  state;

    /* 能量 / 噪声自适应 (一阶差分能量) */
    uint32_t noise_floor;      /* 背景差分能量基线 */
    uint32_t last_energy;
    uint16_t hi_run;           /* 连续 HI 块 */
    uint16_t lo_run;
    uint16_t startup_quiet;    /* 上电静稳块计数 */

    /* 前导导频观察 */
    uint16_t sample_pos;
    uint16_t block_in_pre;
    uint8_t  pilot_last;
    uint8_t  pilot_trans;      /* 1500/2400 交替次数 */
    uint8_t  preamble_miss;    /* 前导中连续非导频窗口数 */
    uint16_t pre_timeout;

    /* 数据栅格自由运行 */
    uint16_t slot_block;       /* 当前槽内块号 0..5 */
    uint16_t win_fill;         /* tone 窗已填样本 (0..VD_TONE_SAMPLES) */
    float    win_buf[VD_TONE_SAMPLES];  /* 完整 20ms tone (v5.1: 320 samples) */
    uint16_t sym_count;        /* 已解符号数 */
    uint16_t sym_expected;     /* LEN 前缀求得的期望符号数 (0=未知) */
    uint8_t  symbols[VP_MAX_DATA_SYMBOLS];

    /* ── v5.1 软判决: 每符号 4 频 Goertzel mag² ── */
    float    sym_mag2[VP_MAX_DATA_SYMBOLS][4];

    /* 数据段 SNR 门限。当前为 VD_SNR_MIN，字段保留供诊断与将来标定。 */
    float    data_snr_threshold;

    /* 状态跟踪 (频域判决, 不依赖时域能量) */
    uint8_t  pilot_hits;       /* 前导频域命中计数 */
    uint8_t  erase_run;        /* 数据段连续擦除计数 */

    /* 结果 */
    uint8_t  payload[VP_MAX_PAYLOAD_BYTES];
    uint8_t  payload_len;
    uint8_t  crc_ok;

    /* 诊断 (可读, 不参与判决) */
    uint8_t  last_digit;
    float    last_conf;        /* True SNR */
} VoiceRx;

/* Goertzel: 对 win[0..N-1] 去 DC 后计算 4 个目标频率(±1 bin)的幅度²,
 * 返回主频 digit(0..3), 若置信度不足返回 0xFF.
 * conf_out 可为 NULL, mag2_out[4] 可为 NULL.
 * v5.1: mag2_out 输出每个 bin 的原始 mag² (供软判决 FEC). */
uint8_t VoiceDSP_Classify(const float *win, uint16_t N,
                          float *conf_out, float *mag2_out);

/* ── v5.1 多窗口分类: 对 tone[0..tone_len-1] 做 3 窗口 Goertzel,
 * 累加 mag², 再做 True SNR + 固定门限判决.
 * snr_threshold 为当前数据段门限.
 * mag2_out[4] 接收累加后的 mag² (供软判决). */
uint8_t VoiceDSP_ClassifyMulti(const float *tone, uint16_t tone_len,
                               float snr_threshold,
                               float *conf_out, float *mag2_out);

/* 一阶差分能量 (高通, 免疫 DC/50Hz) */
uint32_t VoiceDSP_DiffEnergy(const float *blk, uint16_t n);

void     VoiceRx_Init(VoiceRx *rx);
void     VoiceRx_Start(VoiceRx *rx);
/* 送入一个 80-sample 块, 推进状态机. 返回 1 表示本块后 state==VD_DONE.
 * v5.1: 符号判决同时写入 rx->sym_mag2[][] 供软判决 FEC. */
uint8_t  VoiceRx_PushBlock(VoiceRx *rx, const uint16_t *blk);

#ifdef __cplusplus
}
#endif
#endif /* __VOICE_DSP_H */

