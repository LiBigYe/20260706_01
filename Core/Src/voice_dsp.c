/**
  ******************************************************************************
  * @file           : voice_dsp.c
  * @brief          : 声语信使 v5.1 接收 DSP 核心 — 多窗口 Goertzel + 自适应 SNR
  *
  *  时基恢复策略:
  *   - 前导 (1500/2400 交替) 唤醒 + 自适应 SNR 基线采集.
  *   - 1800Hz 同步音是唯一的精定时锚点.
  *   - 检测同步音后回扫上升沿, 锁定数据符号 30ms 栅格.
  *   - 数据阶段自由运行栅格, 用多窗口 Goertzel 累加做判决.
  *
  *  v5.1 变更 (2026-07-16):
  *   - 多窗口 Goertzel: 取满 20ms tone, 3 窗累加 mag² → ~4.8dB SNR 增益.
  *   - 自适应 SNR 门限: 前导段采集 SNR 均值, 数据段门限动态调整.
  *   - 软判决输出: 每符号 4 频 mag² 存入 sym_mag2[][].
  *   - True SNR 分类器 + 双重频域锁 + 频域擦除计数 (继承 v5.0).
  ******************************************************************************
  */
#include "voice_dsp.h"

/* v5.1 多窗口偏移量 */
static const uint16_t vd_multi_win_offsets[VD_MULTI_WIN_COUNT] = {40U, 80U, 120U};
#include <math.h>
#include <string.h>

#ifndef VD_PI
#define VD_PI 3.14159265358979323846f
#endif

/* 原始采样环形缓冲 (供同步音上升沿回扫). 512 样本 = 32ms. */
#define VD_RING 512U
static uint16_t vd_ring[VD_RING];
static uint16_t vd_ring_pos;
static uint32_t vd_total;          /* 累计样本数 (自 Start) */

/* 定时锚点 (绝对样本坐标) */
static uint32_t vd_grid_start;     /* 数据符号 0 的 tone 起点 */

/* 目标频率在 N=160 窗口下的中心 bin */
static const int vd_center_k[4] = {15, 18, 21, 24};  /* 1500/1800/2100/2400 @160 */

/* 频响补偿权重 (作用于 Goertzel mag²) */
static const float vd_freq_weight[4] = {1.33f, 1.08f, 1.00f, 1.02f};

/* ---- 唤醒能量门限 ---- */
#define VD_EN_FLOOR_INIT   400U
#define VD_EN_MARGIN        500U
#define VD_EN_MIN           500U
#define VD_STARTUP_QUIET    15U   /* ~75ms 静稳 */

/* ---- 前导/同步参数 ---- */
#define VD_PRE_TIMEOUT     140U     /* 前导内 140 块(700ms) → 放弃 */
#define VD_MIN_PILOT_TRANS   2U     /* 至少 2 次 1500/2400 交替 */
#define VD_PILOT_HITS_REQ    2U     /* 频域锁: 连续命中次数 */
#define VD_MAX_ERASE_RUN     4U     /* 数据段擦除容忍 */

/* ========================================================================== */
/*  Goertzel                                                                   */
/* ========================================================================== */
static float goertzel_mag2(const uint16_t *win, int N, int k, float dc)
{
    float coeff = 2.0f * cosf(2.0f * VD_PI * (float)k / (float)N);
    float q1 = 0.0f, q2 = 0.0f;
    for (int i = 0; i < N; i++) {
        float x = (float)win[i] - dc;
        float q0 = coeff * q1 - q2 + x;
        q2 = q1; q1 = q0;
    }
    return q1 * q1 + q2 * q2 - q1 * q2 * coeff;
}

/* ========================================================================== */
/*  单窗 Goertzel 判决 (用于前导/同步音/ACK 检测)                              */
/* ========================================================================== */
uint8_t VoiceDSP_Classify(const uint16_t *win, uint16_t N,
                          float *conf_out, float *mag2_out)
{
    /* 1. 去 DC */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < N; i++) sum += win[i];
    float dc = (float)sum / (float)N;

    /* 2. P_total = 时域交流总功率 */
    float p_total = 0.0f;
    for (uint16_t i = 0; i < N; i++) {
        float dev = (float)win[i] - dc;
        p_total += dev * dev;
    }

    if (p_total < 100.0f) {
        if (conf_out) *conf_out = 0.0f;
        if (mag2_out) { for (int i = 0; i < 4; i++) mag2_out[i] = 0.0f; }
        return 0xFFU;
    }

    /* 3. Goertzel ±1 bin, 记录 raw_mag 和 wgt_mag */
    float raw_mag[4];
    float wgt_mag[4];
    for (int i = 0; i < 4; i++) {
        float m = goertzel_mag2(win, (int)N, vd_center_k[i], dc);
        float a = goertzel_mag2(win, (int)N, vd_center_k[i] - 1, dc);
        float b = goertzel_mag2(win, (int)N, vd_center_k[i] + 1, dc);
        if (a > m) m = a;
        if (b > m) m = b;
        raw_mag[i] = m;
        wgt_mag[i] = m * vd_freq_weight[i];
    }

    /* ── v5.1: 输出原始 mag² 供软判决 FEC ── */
    if (mag2_out) {
        for (int i = 0; i < 4; i++) mag2_out[i] = raw_mag[i];
    }

    /* 4. 找最强 (用加权值) */
    int best = 0;
    float bestv = wgt_mag[0];
    for (int i = 1; i < 4; i++) {
        if (wgt_mag[i] > bestv) { bestv = wgt_mag[i]; best = i; }
    }

    /* 5. True SNR */
    float alpha = 2.0f / (float)N;
    float p_signal = raw_mag[best] * alpha;
    float p_noise = p_total - p_signal;
    if (p_noise < 1.0f) p_noise = 1.0f;
    float snr = p_signal / p_noise;
    if (conf_out) *conf_out = snr;

    /* 6. 判决 */
    if (raw_mag[best] < 200000.0f) return 0xFFU;
    if (snr < VD_SNR_MIN) return 0xFFU;

    return (uint8_t)best;
}

/* ========================================================================== */
/*  v5.1 多窗口 Goertzel 累加 + 自适应 SNR 门限                               */
/* ========================================================================== */
uint8_t VoiceDSP_ClassifyMulti(const uint16_t *tone, uint16_t tone_len,
                               float snr_threshold,
                               float *conf_out, float *mag2_out)
{
    /* 对 3 个重叠窗口各做 Goertzel, 累加每个频率的 mag² */
    float acc_mag2[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float p_total = 0.0f;
    uint8_t window_count = 0U;

    for (uint8_t w = 0; w < VD_MULTI_WIN_COUNT; w++) {
        uint16_t offset = vd_multi_win_offsets[w];
        if (offset + VD_WIN > tone_len) break;

        const uint16_t *sub = tone + offset;

        /* 去 DC */
        uint32_t sum = 0;
        for (uint16_t i = 0; i < VD_WIN; i++) sum += sub[i];
        float dc = (float)sum / (float)VD_WIN;

        for (uint16_t i = 0; i < VD_WIN; i++) {
            float dev = (float)sub[i] - dc;
            p_total += dev * dev;
        }
        window_count++;

        /* Goertzel 4 频 */
        for (int f = 0; f < 4; f++) {
            float m = goertzel_mag2(sub, (int)VD_WIN, vd_center_k[f], dc);
            float a = goertzel_mag2(sub, (int)VD_WIN, vd_center_k[f] - 1, dc);
            float b = goertzel_mag2(sub, (int)VD_WIN, vd_center_k[f] + 1, dc);
            if (a > m) m = a;
            if (b > m) m = b;
            acc_mag2[f] += m;  /* 累加, 平滑白噪声 */
        }
    }

    /* ── 输出累加 mag² (供软判决) ── */
    if (mag2_out) {
        for (int i = 0; i < 4; i++) mag2_out[i] = acc_mag2[i];
    }

    /* ── 频响补偿后找最强 ── */
    float wgt[4];
    int best = 0;
    float bestv = 0.0f;
    for (int i = 0; i < 4; i++) {
        wgt[i] = acc_mag2[i] * vd_freq_weight[i];
        if (wgt[i] > bestv) { bestv = wgt[i]; best = i; }
    }

    /* ── 绝对载波能量 ── */
    if (window_count == 0U ||
        acc_mag2[best] < 200000.0f * (float)window_count) {
        if (conf_out) *conf_out = 0.0f;
        return 0xFFU;
    }

    /* The numerator and denominator must cover the same overlapping windows. */
    float alpha = 2.0f / (float)VD_WIN;
    if (p_total < 100.0f * (float)window_count) {
        if (conf_out) *conf_out = 0.0f;
        return 0xFFU;
    }

    float p_signal = acc_mag2[best] * alpha;
    float p_noise = p_total - p_signal;
    if (p_noise < 1.0f) p_noise = 1.0f;
    float snr = p_signal / p_noise;
    if (conf_out) *conf_out = snr;

    /* ── 自适应门限 ── */
    if (snr < snr_threshold) return 0xFFU;

    return (uint8_t)best;
}

/* ========================================================================== */
/*  一阶差分能量 (高通, 免疫 DC/50Hz)                                         */
/* ========================================================================== */
uint32_t VoiceDSP_DiffEnergy(const uint16_t *blk, uint16_t n)
{
    if (n < 2U) return 0U;
    uint32_t sum = 0U;
    for (uint16_t i = 1U; i < n; i++) {
        int32_t d = (int32_t)blk[i] - (int32_t)blk[i - 1U];
        sum += (uint32_t)(d < 0 ? -d : d);
    }
    return sum;
}

/* ========================================================================== */
/*  环形缓冲                                                                   */
/* ========================================================================== */
static void ring_push(uint16_t v)
{
    vd_ring[vd_ring_pos] = v;
    vd_ring_pos = (vd_ring_pos + 1U) & (VD_RING - 1U);
}

static void ring_extract(uint32_t start_abs, uint16_t len, uint16_t *out)
{
    for (uint16_t i = 0; i < len; i++) {
        uint32_t p = (start_abs + i) & (VD_RING - 1U);
        out[i] = vd_ring[p];
    }
}

/* 同步音回扫: 在环形缓冲中从当前位置反向扫描, 找 1800Hz 上升沿起点 */
static uint32_t sync_find_onset(uint32_t now)
{
    /* 取最后 192 样本 (12ms) 做滑动窗口 Goertzel 求最大 1800Hz 响应 */
    #define SYNC_SCAN_LEN  192U
    #define SYNC_STEP        8U
    float best_m = 0.0f;
    uint32_t best_off = 0;
    for (uint32_t off = 0; off < SYNC_SCAN_LEN - VD_WIN; off += SYNC_STEP) {
        uint16_t w[VD_WIN];
        ring_extract(now - SYNC_SCAN_LEN + off, VD_WIN, w);
        uint32_t s = 0;
        for (uint16_t i = 0; i < VD_WIN; i++) s += w[i];
        float dc = (float)s / (float)VD_WIN;
        float m = goertzel_mag2(w, VD_WIN, vd_center_k[VP_SYNC_DIGIT], dc);
        if (m > best_m) { best_m = m; best_off = off; }
    }
    return now - SYNC_SCAN_LEN + best_off;
}

/* ========================================================================== */
/*  接收状态机初始化                                                           */
/* ========================================================================== */
void VoiceRx_Init(VoiceRx *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->state = VD_LISTEN;
    rx->noise_floor = VD_EN_FLOOR_INIT;
    rx->data_snr_threshold = VD_SNR_MIN;
}

void VoiceRx_Start(VoiceRx *rx)
{
    uint32_t nf = rx->noise_floor ? rx->noise_floor : VD_EN_FLOOR_INIT;
    memset(rx, 0, sizeof(*rx));
    rx->state = VD_LISTEN;
    rx->noise_floor = nf;
    rx->data_snr_threshold = VD_SNR_MIN;
    memset(vd_ring, 0, sizeof(vd_ring));
    vd_ring_pos = 0;
    vd_total = 0;
    vd_grid_start = 0;
    rx->pilot_hits = 0;
    rx->erase_run  = 0;
}

/* ========================================================================== */
/*  数据符号存储 (v5.1: 多窗口 + sym_mag2 输出)                                */
/* ========================================================================== */
static void data_store_symbol(VoiceRx *rx)
{
    float conf;
    float mag2[4];
    uint8_t d = VoiceDSP_ClassifyMulti(rx->win_buf, (uint16_t)VD_TONE_SAMPLES,
                                       rx->data_snr_threshold, &conf, mag2);
    rx->last_digit = d;
    rx->last_conf  = conf;

    /* ── v5.1: 保存 mag² 供软判决 FEC ── */
    if (rx->sym_count < VP_MAX_DATA_SYMBOLS) {
        for (int i = 0; i < 4; i++) {
            rx->sym_mag2[rx->sym_count][i] = mag2[i];
        }
    }

    /* ── 频域擦除计数 ── */
    if (d == 0xFFU) {
        rx->erase_run++;
        if (rx->erase_run >= VD_MAX_ERASE_RUN) {
            rx->state = VD_LISTEN;
            rx->hi_run = rx->lo_run = 0;
            return;
        }
    } else {
        rx->erase_run = 0;
    }

    if (rx->sym_count < VP_MAX_DATA_SYMBOLS) {
        rx->symbols[rx->sym_count++] = d;
    }

    /* 收满 LEN 前缀后求期望符号数 */
    if (rx->sym_count == VP_LEN_SYMBOLS && rx->sym_expected == 0U) {
        uint8_t plen = VoiceFEC_DecodeLen(rx->symbols);
        if (plen == 0U || plen > VP_MAX_PAYLOAD_BYTES) {
            rx->state = VD_LISTEN;
            rx->hi_run = rx->lo_run = 0;
            return;
        }
        rx->sym_expected = VoiceFEC_DataSymbolCount(plen);
    }

    /* 收满整帧 → 解码 (硬判决 fast path, 软判决由 receiver.c 在 DONE 后调用) */
    if (rx->sym_expected != 0U && rx->sym_count >= rx->sym_expected) {
        rx->crc_ok = VoiceFEC_ParseDataSymbolsSoft(rx->symbols, rx->sym_mag2, rx->sym_count,
                                                          rx->payload, &rx->payload_len);
        rx->state = VD_DONE;
    }
}

/* ========================================================================== */
/*  主入口: 送入 80-sample 块, 推进状态机                                     */
/* ========================================================================== */
uint8_t VoiceRx_PushBlock(VoiceRx *rx, const uint16_t *blk)
{
    uint32_t block_start_abs = vd_total;

    for (uint16_t i = 0; i < VD_BLOCK; i++) {
        ring_push(blk[i]);
        vd_total++;
    }

    /* 差分能量。AGC 在数据段前冻结，数据判决不依赖增益码。 */
    uint32_t energy_raw = VoiceDSP_DiffEnergy(blk, VD_BLOCK);
    uint32_t energy_norm = energy_raw;

    uint32_t thr = (rx->noise_floor > 0U ? rx->noise_floor : VD_EN_FLOOR_INIT)
                   + VD_EN_MARGIN;
    if (thr < VD_EN_MIN) thr = VD_EN_MIN;
    uint8_t hi = (energy_norm >= thr) ? 1U : 0U;

    /* ── 静稳标定 ── */
    if (rx->startup_quiet < VD_STARTUP_QUIET) {
        if (!hi) {
            rx->noise_floor = (rx->noise_floor * 7U + energy_norm) / 8U;
            rx->startup_quiet++;
        } else {
            rx->noise_floor = (rx->noise_floor * 3U + energy_norm) / 4U;
            rx->startup_quiet = VD_STARTUP_QUIET;
        }
        hi = 0U; rx->hi_run = 0U;
    } else if (rx->state == VD_LISTEN && !hi) {
        rx->noise_floor = (rx->noise_floor * 15U + energy_norm) / 16U;
    }

    if (hi) { rx->hi_run++; rx->lo_run = 0; }
    else    { rx->lo_run++; rx->hi_run = 0; }

    switch (rx->state) {

    case VD_LISTEN:
        if (rx->hi_run >= VD_PREAMBLE_MIN_HI && vd_total >= VD_WIN) {
            uint16_t lwin[VD_WIN];
            ring_extract(vd_total - VD_WIN, VD_WIN, lwin);
            float lconf;
            uint8_t ld = VoiceDSP_Classify(lwin, VD_WIN, &lconf, NULL);
            if (ld == VP_PILOT_LO || ld == VP_PILOT_HI) {
                rx->pilot_hits++;
                if (rx->pilot_hits >= VD_PILOT_HITS_REQ) {
                    /* ── v5.1: 进场时重置自适应 SNR 累加器 ── */
                    rx->state = VD_PREAMBLE;
                    rx->block_in_pre = 0;
                    rx->pilot_last = ld;
                    rx->pilot_trans = 0;
                    rx->pilot_hits = 0;
                    rx->preamble_snr_sum = 0.0f;
                    rx->preamble_snr_count = 0;
                }
            } else {
                rx->pilot_hits = 0;
            }
        } else {
            rx->pilot_hits = 0;
        }
        break;

    case VD_PREAMBLE: {
        rx->block_in_pre++;
        if (rx->block_in_pre > VD_PRE_TIMEOUT) {
            rx->state = VD_LISTEN; rx->hi_run = 0; break;
        }
        if (vd_total < VD_WIN) break;
        uint16_t win[VD_WIN];
        ring_extract(vd_total - VD_WIN, VD_WIN, win);
        float conf;
        uint8_t d = VoiceDSP_Classify(win, VD_WIN, &conf, NULL);

        if (d == VP_PILOT_LO || d == VP_PILOT_HI) {
            /* ── v5.1: 前导段累加 SNR ── */
            rx->preamble_snr_sum += conf;
            rx->preamble_snr_count++;

            if (rx->pilot_last != 0xFFU && d != rx->pilot_last) rx->pilot_trans++;
            rx->pilot_last = d;
        } else if (d == VP_SYNC_DIGIT && rx->pilot_trans >= VD_MIN_PILOT_TRANS) {
            /* ── v5.1: 锁定栅格 + 计算自适应 SNR 门限 ── */
            uint32_t tone_start = sync_find_onset(vd_total);
            vd_grid_start = tone_start + VP_SLOT_SAMPLES;
            rx->state = VD_DATA;
            rx->sym_count = 0;
            rx->sym_expected = 0;
            rx->win_fill = 0;
            rx->lo_run = 0;
            rx->erase_run = 0;

            /* 自适应 SNR 门限 = max(VD_SNR_MIN, 前导均值 × 0.5) */
            if (rx->preamble_snr_count > 0U) {
                float avg_snr = rx->preamble_snr_sum
                              / (float)rx->preamble_snr_count;
                rx->data_snr_threshold = avg_snr * 0.5f;
                if (rx->data_snr_threshold < VD_SNR_MIN)
                    rx->data_snr_threshold = VD_SNR_MIN;
            } else {
                rx->data_snr_threshold = VD_SNR_MIN;
            }
        }
        break;
    }

    case VD_DATA: {
        for (uint16_t i = 0; i < VD_BLOCK; i++) {
            uint32_t abspos = block_start_abs + i;
            if (abspos < vd_grid_start) continue;
            uint32_t rel = abspos - vd_grid_start;
            uint32_t sym = rel / VP_SLOT_SAMPLES;
            uint32_t pos = rel % VP_SLOT_SAMPLES;
            if (sym != rx->sym_count) continue;

            /* ── v5.1: 取满 20ms tone (pos 0..319) ── */
            if (pos >= VD_TONE_SAMPLES) continue;  /* guard 区间, 跳过 */
            rx->win_buf[pos] = blk[i];
            rx->win_fill = (uint16_t)(pos + 1U);
            if (rx->win_fill == VD_TONE_SAMPLES) {
                data_store_symbol(rx);
                rx->win_fill = 0;
                if (rx->state != VD_DATA) break;
            }
        }
        break;
    }

    case VD_DONE:
    default:
        break;
    }

    return (rx->state == VD_DONE) ? 1U : 0U;
}



