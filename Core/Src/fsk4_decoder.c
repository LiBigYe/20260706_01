/**
  ******************************************************************************
  * @file           : fsk4_decoder.c
  * @brief          : 4-FSK Goertzel 解码器实现 — 声语信使接收端 (v4)
  *
  *   Goertzel 算法:
  *     对每个窗口的 320 个采样值, 计算 4 个目标频率的 DFT 幅度.
  *     取幅度最大的频率作为检测结果.
  *
  *   v4: 输入为包络检波提取的纯净 20ms 载波 — 无保护间隔直流/纹波,
  *   无跨符号 ISI, N=320 所有 4 个 k 值均为整数, 零频谱泄露.
  *
  *   Goertzel 公式:
  *     k = N * f_target / f_sample  (取整)
  *     omega = 2 * PI * k / N
  *     coeff = 2 * cos(omega)
  *
  *     每采样迭代:  q0 = coeff * q1 - q2 + sample
  *                  q2 = q1
  *                  q1 = q0
  *
  *     最终幅度²:    magnitude_sq = q1² + q2² - q1 * q2 * coeff
  *
  *   N=320, f_sample=16000 (所有 k 为精确整数):
  *     ┌──────────┬─────┬────────┬───────────┐
  *     │ f (Hz)   │  k  │ omega  │ coeff     │
  *     ├──────────┼─────┼────────┼───────────┤
  *     │ 1500     │ 30  │ 0.5890 │ 1.662939  │
  *     │ 1800     │ 36  │ 0.7069 │ 1.517638  │
  *     │ 2100     │ 42  │ 0.8247 │ 1.353595  │
  *     │ 2400     │ 48  │ 0.9425 │ 1.175571  │
  *     └──────────┴─────┴────────┴───────────┘
  *
  *   用 FPU 浮点运算 (Cortex-M4 单精度硬件浮点).
  *   每窗口处理时间 ≈ 4×320×(3 MAC + 2 load/store) ≈ 50-100 µs @ 16MHz.
  ******************************************************************************
  */

#include "fsk4_decoder.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ========================================================================== */
/*  频率名称表                                                                 */
/* ========================================================================== */

static const char *freq_names[4] = {"1500Hz", "1800Hz", "2100Hz", "2400Hz"};

static const uint16_t target_freqs[FSK4_DECODER_FREQ_COUNT] = {
    FSK4_DECODER_F0,  /* 1500 Hz */
    FSK4_DECODER_F1,  /* 1800 Hz */
    FSK4_DECODER_F2,  /* 2100 Hz */
    FSK4_DECODER_F3,  /* 2400 Hz */
};

const char* FSK4_Decoder_GetFreqName(uint8_t digit)
{
    if (digit < 4) return freq_names[digit];
    return "?";
}

/* ========================================================================== */
/*  解码器核心                                                                  */
/* ========================================================================== */

/**
  * @brief  初始化解码器
  */
void FSK4_Decoder_Init(FSK4_Decoder *dec, uint16_t block_size)
{
    memset(dec, 0, sizeof(FSK4_Decoder));
    dec->block_size = block_size;

    float N = (float)block_size;
    float fs = (float)FSK4_DECODER_SAMPLE_RATE;

    for (uint8_t i = 0; i < FSK4_DECODER_FREQ_COUNT; i++) {
        float f = (float)target_freqs[i];

        /* k = round(N * f / fs) */
        dec->k[i] = (uint32_t)(N * f / fs + 0.5f);

        /* omega = 2 * PI * k / N */
        float omega = 2.0f * M_PI * (float)dec->k[i] / N;

        /* coeff = 2 * cos(omega) */
        dec->coeff[i] = 2.0f * cosf(omega);
    }
}

/**
  * @brief  重置解码器 (新窗口开始)
  */
void FSK4_Decoder_Reset(FSK4_Decoder *dec)
{
    for (uint8_t i = 0; i < FSK4_DECODER_FREQ_COUNT; i++) {
        dec->q1[i] = 0.0f;
        dec->q2[i] = 0.0f;
    }
    dec->sample_count = 0;
    dec->dominant_digit = 0xFF;
}

/**
  * @brief  处理单个采样值 (Goertzel 迭代)
  *
  *   对 4 个频率并行迭代.
  *   ADC 值范围为 0~4095, 中心约 2048 (1.65V 偏置).
  *   减去直流偏置后送入 Goertzel, 以提高频率选择性.
  */
void FSK4_Decoder_ProcessSample(FSK4_Decoder *dec, uint16_t sample)
{
    if (dec->sample_count >= dec->block_size) return;

    /* 去直流: ADC 中心 ≈ 2048 (12-bit), 转 float */
    float x = (float)((int32_t)sample - 2048);

    for (uint8_t i = 0; i < FSK4_DECODER_FREQ_COUNT; i++) {
        float q0 = dec->coeff[i] * dec->q1[i] - dec->q2[i] + x;
        dec->q2[i] = dec->q1[i];
        dec->q1[i] = q0;
    }

    dec->sample_count++;

    /* 达到 block_size 时计算最终幅度 */
    if (dec->sample_count >= dec->block_size) {
        float max_mag = 0.0f, second_mag = 0.0f;
        dec->dominant_digit = 0xFF;

        for (uint8_t i = 0; i < FSK4_DECODER_FREQ_COUNT; i++) {
            float c = dec->coeff[i];
            float q1 = dec->q1[i];
            float q2 = dec->q2[i];

            /* magnitude_sq = q1² + q2² - q1 * q2 * coeff */
            float mag_sq = q1 * q1 + q2 * q2 - q1 * q2 * c;
            dec->magnitude_sq[i] = mag_sq;

            if (mag_sq > max_mag) {
                second_mag = max_mag;
                max_mag = mag_sq;
                dec->dominant_digit = i;
            } else if (mag_sq > second_mag) {
                second_mag = mag_sq;
            }
        }

        /* 检查: (1) 低于噪声门限, 或 (2) SNR 不足 → 判噪声 */
        if (max_mag < FSK4_DECODER_NOISE_THRESHOLD) {
            dec->dominant_digit = 0xFF;
        } else if (max_mag < second_mag * FSK4_DECODER_SNR_RATIO) {
            dec->dominant_digit = 0xFF;
        }
    }
}

/**
  * @brief  对完整数据块运行 Goertzel 检测
  *
  *   便捷函数: 内部做 Reset → 逐采样 ProcessSample → 返回 digit.
  *   用于对包络检波提取的纯净 320-sample 载波数据进行一次性检测.
  *
  *   v4: 输入为 4×5ms HI 块的 320 个连续采样 (纯载波, 无保护间隔).
  *   在 Goertzel 之前先检查信号幅度: 如果 320-sample 窗口内
  *   最大-最小 < FSK4_DECODER_MIN_AMPLITUDE, 直接判为噪声.
  */
uint8_t FSK4_Decoder_DetectBlock(FSK4_Decoder *dec, const uint16_t *samples)
{
    /* ── 信号幅度预检查 ── */
    uint16_t vmin = 4095, vmax = 0;
    for (uint16_t i = 0; i < dec->block_size; i++) {
        uint16_t v = samples[i];
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    if ((vmax - vmin) < FSK4_DECODER_MIN_AMPLITUDE) {
        dec->dominant_digit = 0xFF;
        return 0xFF;
    }

    /* ── Goertzel 检测 ── */
    FSK4_Decoder_Reset(dec);

    for (uint16_t i = 0; i < dec->block_size; i++) {
        FSK4_Decoder_ProcessSample(dec, samples[i]);
    }

    return dec->dominant_digit;
}

/**
  * @brief  获取各频率的 magnitude² 值 (调试/诊断用)
  */
void FSK4_Decoder_GetMagnitudes(FSK4_Decoder *dec, float *mag_out)
{
    for (uint8_t i = 0; i < FSK4_DECODER_FREQ_COUNT; i++) {
        mag_out[i] = dec->magnitude_sq[i];
    }
}
