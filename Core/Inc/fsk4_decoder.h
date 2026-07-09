/**
  ******************************************************************************
  * @file           : fsk4_decoder.h
  * @brief          : 4-FSK Goertzel 解码器 — 声语信使接收端 (v4)
  *
  *   Goertzel 算法在 320 采样点 (20ms @ 16kHz) 窗口内检测 4 个频率:
  *     1500 Hz → digit 0
  *     1800 Hz → digit 1
  *     2100 Hz → digit 2
  *     2400 Hz → digit 3
  *
  *   使用浮点运算 (Cortex-M4 FPU 硬件加速).
  *   每个 320-sample 窗口 ≈ 4×1280 MAC 操作 ≈ 50µs @ 16MHz.
  *
  *   v4 改进: 解码器只接收包络检波提取的纯净 320-sample 载波,
  *   无保护间隔直流/纹波污染, 无跨符号 ISI, N=320 所有 k 为整数.
  *
  *   频率选择: 取 4 个频率中 magnitude² 最大的那个.
  ******************************************************************************
  */

#ifndef __FSK4_DECODER_H
#define __FSK4_DECODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========================================================================== */
/*  常量                                                                       */
/* ========================================================================== */

#define FSK4_DECODER_FREQ_COUNT  4       /* 4 个 FSK 频率 */
#define FSK4_DECODER_BLOCK_SIZE  320     /* Goertzel 窗口大小 (20ms @ 16kHz) */
#define FSK4_DECODER_SAMPLE_RATE 16000   /* ADC 采样率 Hz */

/* 4 个目标频率 (Hz) */
#define FSK4_DECODER_F0    1500
#define FSK4_DECODER_F1    1800
#define FSK4_DECODER_F2    2100
#define FSK4_DECODER_F3    2400

/* 噪声门限: magnitude² 低于此值 → 判为无信号
 *   满摆幅 3.3Vpp → ADC≈±2047 → mag² ≈ (2047×160)² ≈ 1.07×10¹¹
 *   浮动引脚噪声:   mag² 通常 < 1×10⁶
 *   10,000,000 在两者之间有 10,000:1 安全余量. */
#define FSK4_DECODER_NOISE_THRESHOLD  10000000.0f

/* 信号幅度门限: 320-sample 窗口内 peak-to-peak 低于此值 → 噪声
 *   1000 ADC counts ≈ 806 mVpp @ 3.3V/12-bit.
 *   满摆幅信号 ≈ 4095 counts, 浮动噪声 ≈ 30~150 counts.
 *   1000 足够拦截所有浮动噪声, 保留 4:1 真实信号余量. */
#define FSK4_DECODER_MIN_AMPLITUDE    1000

/* SNR 门限: 最强频率的 magnitude² 必须 > 第二强的 N 倍, 否则判噪声
 *   2.5x — 强信号下的纯音 bin 极为突出, 提高门限进一步提升抗噪. */
#define FSK4_DECODER_SNR_RATIO        2.5f

/* ========================================================================== */
/*  数据结构                                                                   */
/* ========================================================================== */

/**
  * @brief  Goertzel 解码器状态
  *
  *   对每个目标频率, 需要 q0/q1/q2 迭代状态.
  *   coeff = 2 * cos(2π * k / N), 预计算.
  */
typedef struct {
    /* ── 预计算系数 ── */
    float    coeff[FSK4_DECODER_FREQ_COUNT];  /* 每个频率的 Goertzel 系数 */
    uint32_t k[FSK4_DECODER_FREQ_COUNT];      /* 每个频率的 k = N * f / fs */

    /* ── 运行时状态 ── */
    float    q1[FSK4_DECODER_FREQ_COUNT];     /* Goertzel q1 (当前) */
    float    q2[FSK4_DECODER_FREQ_COUNT];     /* Goertzel q2 (前一) */
    uint16_t sample_count;                     /* 已处理采样数 (0..N-1) */
    uint16_t block_size;                       /* 窗口大小 (=N) */

    /* ── 最近一次检测结果 ── */
    float    magnitude_sq[FSK4_DECODER_FREQ_COUNT];  /* 各频率的 |magnitude|² */
    uint8_t  dominant_digit;                         /* 0~3 = 最强频率, 0xFF = 噪声 */
} FSK4_Decoder;

/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

/**
  * @brief  初始化解码器, 预计算 Goertzel 系数
  * @param  dec: 解码器状态指针
  * @param  block_size: Goertzel 窗口大小 (典型值 320)
  */
void FSK4_Decoder_Init(FSK4_Decoder *dec, uint16_t block_size);

/**
  * @brief  重置解码器状态 (新窗口开始)
  * @param  dec: 解码器状态指针
  *
  *   清零 q1/q2/sample_count, 准备处理下一个窗口.
  */
void FSK4_Decoder_Reset(FSK4_Decoder *dec);

/**
  * @brief  处理单个采样值 (Goertzel 迭代一步)
  * @param  dec:    解码器状态指针
  * @param  sample: ADC 采样值 (0~4095, 会被浮点化)
  *
  *   对 4 个频率同时迭代 Goertzel.
  *   当 sample_count 达到 block_size 时自动完成 magnitude 计算.
  */
void FSK4_Decoder_ProcessSample(FSK4_Decoder *dec, uint16_t sample);

/**
  * @brief  对完整的 320-sample 载波数据块运行 Goertzel 检测
  * @param  dec:     解码器状态指针
  * @param  samples: uint16 ADC 采样数组 [320] (纯载波, 无保护间隔)
  * @retval 0~3 = 最强频率对应的 digit, 0xFF = 所有频率均低于噪声门限
  *
  *   v4: 输入为包络检波提取的纯净 20ms 载波 (N=320, 整数 k).
  *   内部调用 Reset + 320×ProcessSample + 幅度比较.
  *   先做 peak-to-peak 幅度预检查再做 Goertzel.
  */
uint8_t FSK4_Decoder_DetectBlock(FSK4_Decoder *dec, const uint16_t *samples);

/**
  * @brief  获取当前窗口的各频率 magnitude² 值 (调试用)
  * @param  dec: 解码器状态指针
  * @param  mag_out: 输出数组 [4], float*4
  */
void FSK4_Decoder_GetMagnitudes(FSK4_Decoder *dec, float *mag_out);

/**
  * @brief  获取 digit 名称字符串
  * @param  digit: 0~3
  * @retval "1500Hz" / "1800Hz" / "2100Hz" / "2400Hz" / "?"
  */
const char* FSK4_Decoder_GetFreqName(uint8_t digit);

#ifdef __cplusplus
}
#endif

#endif /* __FSK4_DECODER_H */
