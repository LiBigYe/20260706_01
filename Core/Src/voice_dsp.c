/**
  ******************************************************************************
  * @file           : voice_dsp.c
  * @brief          : 声语信使 v5.1 接收 DSP 核心 — 多窗口 Goertzel + 固定 SNR
  *
  *  时基恢复策略:
  *   - 前导 (1500/2400 交替) 唤醒与频域锁定.
  *   - 1800Hz 同步音是唯一的精定时锚点.
  *   - 检测同步音后回扫上升沿, 锁定数据符号 30ms 栅格.
  *   - 数据阶段自由运行栅格, 用多窗口 Goertzel 累加做判决.
  *
  *  v5.1 变更 (2026-07-16):
  *   - 多窗口 Goertzel: 取满 20ms tone, 3 窗累加 mag² → ~4.8dB SNR 增益.
  *   - 固定最低 SNR 门限: 前导段不会抬高数据段的判决门槛.
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

/* 滤波后的浮点采样环形缓冲 (供同步音上升沿回扫). 512 样本 = 32ms. */
#define VD_RING 512U
static float vd_ring[VD_RING];
static uint16_t vd_ring_pos;
static uint32_t vd_total;          /* 累计样本数 (自 Start) */
/* RX is a single DMA-driven instance, so these workspaces avoid large ISR
 * stack frames without introducing concurrent access. */
static float vd_filtered_blk[VD_BLOCK];
static float vd_work_win[VD_WIN];

/* 定时锚点 (绝对样本坐标) */
static uint32_t vd_grid_start;     /* 数据符号 0 的 tone 起点 */

/* 目标频率在 N=160 窗口下的中心 bin */
static const int vd_center_k[4] = {15, 18, 21, 24};  /* 1500/1800/2100/2400 @160 */

/* 频响补偿权重 (作用于 Goertzel mag²) */
static const float vd_freq_weight[4] = {1.33f, 1.08f, 1.00f, 1.02f};

/* 1.1-2.8 kHz band-pass for the complete RX DSP path: one second-order
 * high-pass and three cascaded second-order low-pass sections.  This rejects
 * PWM residue and harmonic energy before it can inflate wideband SNR noise. */
typedef struct {
    float b0, b1, b2, a1, a2;
    float z1, z2;
} VdBiquad;

static VdBiquad vd_bp_high = {
    0.736145252f, -1.472290504f, 0.736145252f, -1.401415354f, 0.543165654f,
    0.0f, 0.0f
};
static VdBiquad vd_bp_low_1 = {
    0.167483800f, 0.334967600f, 0.167483800f, -0.557030997f, 0.226966197f,
    0.0f, 0.0f
};
static VdBiquad vd_bp_low_2 = {
    0.167483800f, 0.334967600f, 0.167483800f, -0.557030997f, 0.226966197f,
    0.0f, 0.0f
};
static VdBiquad vd_bp_low_3 = {
    0.167483800f, 0.334967600f, 0.167483800f, -0.557030997f, 0.226966197f,
    0.0f, 0.0f
};
static uint8_t vd_bp_primed;

static float biquad_push(VdBiquad *filter, float input)
{
    float output = filter->b0 * input + filter->z1;
    filter->z1 = filter->b1 * input - filter->a1 * output + filter->z2;
    filter->z2 = filter->b2 * input - filter->a2 * output;
    return output;
}

static void bandpass_reset(void)
{
    vd_bp_high.z1 = vd_bp_high.z2 = 0.0f;
    vd_bp_low_1.z1 = vd_bp_low_1.z2 = 0.0f;
    vd_bp_low_2.z1 = vd_bp_low_2.z2 = 0.0f;
    vd_bp_low_3.z1 = vd_bp_low_3.z2 = 0.0f;
    vd_bp_primed = 0U;
}

static float bandpass_sample(uint16_t sample)
{
    float centered = (float)sample - 2048.0f;

    /* Start the high-pass at its DC steady state.  The ADC midpoint is not
     * guaranteed to be exactly 2048, so clearing the filter must not turn its
     * static offset into a false wideband transient. */
    if (!vd_bp_primed) {
        vd_bp_high.z1 = -vd_bp_high.b0 * centered;
        vd_bp_high.z2 =  vd_bp_high.b2 * centered;
        vd_bp_primed = 1U;
    }

    float filtered = biquad_push(&vd_bp_high, centered);
    filtered = biquad_push(&vd_bp_low_1, filtered);
    filtered = biquad_push(&vd_bp_low_2, filtered);
    filtered = biquad_push(&vd_bp_low_3, filtered);
    return filtered;
}

/* ---- AGC-scaled wakeup energy gate ---- */
#define VD_EN_FLOOR_INIT   400U
#define VD_EN_MARGIN        500U
#define VD_EN_MIN           500U
#define VD_STARTUP_QUIET    15U   /* ~75ms 静稳 */
/* The carrier is accepted by frequency dominance and SNR.  Keep this only as
 * a near-silence guard; AGC still controls the analogue input range. */
#define VD_CARRIER_MAG2_MIN 100.0f
#define VD_ACTIVITY_POWER_MIN 16.0f
#define VD_FREQ_RATIO_MIN   1.35f

/* ---- 前导/同步参数 ---- */
#define VD_PRE_TIMEOUT     140U     /* 前导内 140 块(700ms) → 放弃 */
#define VD_MIN_PILOT_TRANS   2U     /* 至少 2 次 1500/2400 交替 */
#define VD_PILOT_HITS_REQ    2U     /* 频域锁: 连续命中次数 */

/* ========================================================================== */
/*  Goertzel                                                                   */
/* ========================================================================== */
static float goertzel_mag2(const float *win, int N, int k, float dc)
{
    float coeff = 2.0f * cosf(2.0f * VD_PI * (float)k / (float)N);
    float q1 = 0.0f, q2 = 0.0f;
    for (int i = 0; i < N; i++) {
        float x = win[i] - dc;
        float q0 = coeff * q1 - q2 + x;
        q2 = q1; q1 = q0;
    }
    return q1 * q1 + q2 * q2 - q1 * q2 * coeff;
}

/* ========================================================================== */
/*  单窗 Goertzel 判决 (用于前导/同步音/ACK 检测)                              */
/* ========================================================================== */
uint8_t VoiceDSP_Classify(const float *win, uint16_t N,
                          float *conf_out, float *mag2_out)
{
    /* 1. 去 DC */
    float sum = 0.0f;
    for (uint16_t i = 0; i < N; i++) sum += win[i];
    float dc = (float)sum / (float)N;

    /* 2. P_total = 时域交流总功率 */
    float p_total = 0.0f;
    for (uint16_t i = 0; i < N; i++) {
        float dev = win[i] - dc;
        p_total += dev * dev;
    }

    if (p_total < VD_ACTIVITY_POWER_MIN) {
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

    /* 4. 找最强和次强 (用加权值) */
    int best = 0;
    float bestv = wgt_mag[0];
    for (int i = 1; i < 4; i++) {
        if (wgt_mag[i] > bestv) { bestv = wgt_mag[i]; best = i; }
    }
    float secondv = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (i != best && wgt_mag[i] > secondv) secondv = wgt_mag[i];
    }
    float freq_ratio = bestv / (secondv > 1.0f ? secondv : 1.0f);

    /* 5. True SNR */
    float alpha = 2.0f / (float)N;
    float p_signal = raw_mag[best] * alpha;
    float p_noise = p_total - p_signal;
    if (p_noise < 1.0f) p_noise = 1.0f;
    float snr = p_signal / p_noise;
    if (conf_out) *conf_out = snr;

    /* 6. 判决 */
    if (raw_mag[best] < VD_CARRIER_MAG2_MIN) return 0xFFU;
    if (snr < VD_SNR_MIN && freq_ratio < VD_FREQ_RATIO_MIN) return 0xFFU;

    return (uint8_t)best;
}

/* ========================================================================== */
/*  v5.1 多窗口 Goertzel 累加 + 固定 SNR 门限                                 */
/* ========================================================================== */
uint8_t VoiceDSP_ClassifyMulti(const float *tone, uint16_t tone_len,
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

        const float *sub = tone + offset;

        /* 去 DC */
        float sum = 0.0f;
        for (uint16_t i = 0; i < VD_WIN; i++) sum += sub[i];
        float dc = (float)sum / (float)VD_WIN;

        for (uint16_t i = 0; i < VD_WIN; i++) {
            float dev = sub[i] - dc;
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

    /* ── 频响补偿后找最强和次强 ── */
    float wgt[4];
    int best = 0;
    float bestv = 0.0f;
    for (int i = 0; i < 4; i++) {
        wgt[i] = acc_mag2[i] * vd_freq_weight[i];
        if (wgt[i] > bestv) { bestv = wgt[i]; best = i; }
    }
    float secondv = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (i != best && wgt[i] > secondv) secondv = wgt[i];
    }
    float freq_ratio = bestv / (secondv > 1.0f ? secondv : 1.0f);

    /* ── 绝对载波能量 ── */
    if (window_count == 0U ||
        acc_mag2[best] < VD_CARRIER_MAG2_MIN * (float)window_count) {
        if (conf_out) *conf_out = 0.0f;
        return 0xFFU;
    }

    /* The numerator and denominator must cover the same overlapping windows. */
    float alpha = 2.0f / (float)VD_WIN;
    if (p_total < VD_ACTIVITY_POWER_MIN * (float)window_count) {
        if (conf_out) *conf_out = 0.0f;
        return 0xFFU;
    }

    float p_signal = acc_mag2[best] * alpha;
    float p_noise = p_total - p_signal;
    if (p_noise < 1.0f) p_noise = 1.0f;
    float snr = p_signal / p_noise;
    if (conf_out) *conf_out = snr;

    if (snr < snr_threshold && freq_ratio < VD_FREQ_RATIO_MIN) return 0xFFU;

    return (uint8_t)best;
}

/* ========================================================================== */
/*  一阶差分能量 (高通, 免疫 DC/50Hz)                                         */
/* ========================================================================== */
uint32_t VoiceDSP_DiffEnergy(const float *blk, uint16_t n)
{
    if (n < 2U) return 0U;
    float sum = 0.0f;
    for (uint16_t i = 1U; i < n; i++) {
        sum += fabsf(blk[i] - blk[i - 1U]);
    }
    return (uint32_t)(sum + 0.5f);
}

/* ========================================================================== */
/*  环形缓冲                                                                   */
/* ========================================================================== */
static void ring_push(float v)
{
    vd_ring[vd_ring_pos] = v;
    vd_ring_pos = (vd_ring_pos + 1U) & (VD_RING - 1U);
}

static void ring_extract(uint32_t start_abs, uint16_t len, float *out)
{
    for (uint16_t i = 0; i < len; i++) {
        uint32_t p = (start_abs + i) & (VD_RING - 1U);
        out[i] = vd_ring[p];
    }
}

/* 同步音回扫: 5ms 窗口下 1800Hz 正好位于整数 bin 9, 可稳定定位起点。 */
#define VD_SYNC_ONSET_WIN   80U
#define VD_SYNC_ONSET_STEP   8U
#define VD_SYNC_SCAN_BACK  400U

static float sync_window_mag2(uint32_t start_abs)
{
    float win[VD_SYNC_ONSET_WIN];
    ring_extract(start_abs, VD_SYNC_ONSET_WIN, win);

    float sum = 0.0f;
    for (uint16_t i = 0; i < VD_SYNC_ONSET_WIN; i++) sum += win[i];
    return goertzel_mag2(win, VD_SYNC_ONSET_WIN, 9,
                         (float)sum / (float)VD_SYNC_ONSET_WIN);
}

static uint32_t sync_find_onset(uint32_t now)
{
    uint32_t lo = (now > VD_SYNC_SCAN_BACK) ? now - VD_SYNC_SCAN_BACK : 0U;
    uint32_t hi = (now > VD_SYNC_ONSET_WIN) ? now - VD_SYNC_ONSET_WIN : 0U;
    float peak = 0.0f;

    for (uint32_t p = lo; p <= hi; p += VD_SYNC_ONSET_STEP) {
        float m = sync_window_mag2(p);
        if (m > peak) peak = m;
    }
    for (uint32_t p = lo; p <= hi; p += VD_SYNC_ONSET_STEP) {
        if (sync_window_mag2(p) >= peak * 0.8f) return p;
    }
    return lo;
}

/* ========================================================================== */
/*  接收状态机初始化                                                           */
/* ========================================================================== */
void VoiceRx_Init(VoiceRx *rx)
{
    memset(rx, 0, sizeof(*rx));
    bandpass_reset();
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
    bandpass_reset();
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

    /* Keep erasures as diagnostics, but do not abandon a frame in progress.
     * Interleaving and Chase decoding can still recover a short burst. */
    if (d == 0xFFU) {
        if (rx->erase_run != 0xFFU) rx->erase_run++;
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
        vd_filtered_blk[i] = bandpass_sample(blk[i]);
        ring_push(vd_filtered_blk[i]);
        vd_total++;
    }

    /* 差分能量。AGC 在数据段前冻结，数据判决不依赖增益码。 */
    uint32_t energy_raw = VoiceDSP_DiffEnergy(vd_filtered_blk, VD_BLOCK);
    uint32_t energy_norm = energy_raw;
    rx->last_energy = energy_raw;

    uint32_t thr = (rx->noise_floor > 0U ? rx->noise_floor : VD_EN_FLOOR_INIT)
                   + VD_EN_MARGIN;
    if (thr < VD_EN_MIN) thr = VD_EN_MIN;
    uint8_t hi = (energy_norm >= thr) ? 1U : 0U;

    /* A frame may arrive immediately after RX starts.  Let the spectral
     * pilot lock reject noise instead of learning a valid preamble as noise. */
    if (rx->startup_quiet < VD_STARTUP_QUIET) {
        if (hi) {
            rx->startup_quiet = VD_STARTUP_QUIET;
        } else {
            rx->noise_floor = (rx->noise_floor * 7U + energy_norm) / 8U;
            rx->startup_quiet++;
            hi = 0U;
            rx->hi_run = 0U;
        }
    } else if (rx->state == VD_LISTEN && !hi) {
        rx->noise_floor = (rx->noise_floor * 15U + energy_norm) / 16U;
    }

    if (hi) { rx->hi_run++; rx->lo_run = 0; }
    else    { rx->lo_run++; rx->hi_run = 0; }

    switch (rx->state) {

    case VD_LISTEN:
        if (rx->hi_run >= VD_PREAMBLE_MIN_HI && vd_total >= VD_WIN) {
            ring_extract(vd_total - VD_WIN, VD_WIN, vd_work_win);
            float lconf;
            uint8_t ld = VoiceDSP_Classify(vd_work_win, VD_WIN, &lconf, NULL);
            if (ld == VP_PILOT_LO || ld == VP_PILOT_HI) {
                rx->pilot_hits++;
                if (rx->pilot_hits >= VD_PILOT_HITS_REQ) {
                    rx->state = VD_PREAMBLE;
                    rx->block_in_pre = 0;
                    rx->pilot_last = ld;
                    rx->pilot_trans = 0;
                    rx->preamble_miss = 0U;
                    rx->pilot_hits = 0;
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
        ring_extract(vd_total - VD_WIN, VD_WIN, vd_work_win);
        uint8_t d = VoiceDSP_Classify(vd_work_win, VD_WIN, NULL, NULL);

        if (d == VP_PILOT_LO || d == VP_PILOT_HI) {
            rx->preamble_miss = 0U;
            if (rx->pilot_last != 0xFFU && d != rx->pilot_last) rx->pilot_trans++;
            rx->pilot_last = d;
        } else if (d == VP_SYNC_DIGIT && rx->pilot_trans >= VD_MIN_PILOT_TRANS) {
            /* ── v5.1: 锁定栅格 ── */
            uint32_t tone_start = sync_find_onset(vd_total);
            vd_grid_start = tone_start + VP_SLOT_SAMPLES;
            rx->state = VD_DATA;
            rx->sym_count = 0;
            rx->sym_expected = 0;
            rx->win_fill = 0;
            rx->lo_run = 0;
            rx->erase_run = 0;

            /* The preamble can be locally reinforced by room reflections.
             * Do not promote that local SNR into a packet-wide requirement. */
            rx->data_snr_threshold = VD_SNR_MIN;
        } else {
            if (rx->preamble_miss != 0xFFU) rx->preamble_miss++;
            if (rx->preamble_miss >= VD_PREAMBLE_MAX_MISSES) {
                rx->state = VD_LISTEN;
                rx->hi_run = rx->lo_run = 0U;
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
            rx->win_buf[pos] = vd_filtered_blk[i];
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



