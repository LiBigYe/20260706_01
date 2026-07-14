/**
  ******************************************************************************
  * @file           : voice_dsp.c
  * @brief          : 声语信使 v5 接收 DSP 核心实现 (可移植)
  *
  *  时基恢复策略 (关键):
  *   - 前导 (1500/2400 交替) 只用于唤醒 + 冻结噪声底 + 交替校验 (排除窄带噪声).
  *   - 1800Hz 同步音 (前导中从不出现) 是唯一的精定时锚点.
  *   - 检测到同步音后, 在原始采样环形缓冲里回扫其上升沿, 得到 sample 级
  *     的同步音起点, 由此锁定数据符号 30ms 栅格.
  *   - 数据阶段完全自由运行栅格, 只取每个 20ms tone 的中间 10ms 做判决.
  *     不依赖 guard 下降沿 → 免疫混响拖尾 (复核缺陷 2).
  ******************************************************************************
  */
#include "voice_dsp.h"
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

/* 频响补偿权重 (作用于 Goertzel 幅度²):
 * 声学链路 (扬声器+RC低通+麦克风+运放) 对 1500Hz 增益偏低, 使 digit0 的
 * 幅度天生弱于其它三音, 判决时吃亏. 给 1500Hz 的判决量(mag²)加权 1.25,
 * 即"别人 1 倍、它 1.25 倍"再一起比较. 同时影响主/次频选择与置信度.
 * 实测若仍偏弱可继续上调 vd_freq_weight[0]. */
static const float vd_freq_weight[4] = {1.25f, 1.0f, 1.0f, 1.0f};

/* ---- 唤醒能量门限 (自适应, 仅用于"唤醒提示", 真正判决靠频谱置信度) ----
 * 复核缺陷 1/3 对策: 能量门限只做低成本唤醒, 取 noise×1.5, 让弱信号也能
 * 进入 PREAMBLE; 误唤醒由后续导频交替 + 同步音置信度 (裕量数百倍) 自然拒绝. */
#define VD_EN_FLOOR_INIT   400U
#define VD_EN_MARGIN       500U
#define VD_EN_MIN          800U     /* 差分能量最小唤醒门限 (按用户要求回退 2000→800) */
#define VD_STARTUP_QUIET    15U     /* ~75ms 静稳 (缩短以降低首帧竞态窗口) */

/* ---- 前导/同步参数 (样本) ---- */
#define VD_PREAMBLE_MIN_HI   6U     /* 连续 6 HI 块 (30ms) → PREAMBLE */
#define VD_PRE_TIMEOUT     140U     /* 前导内 140 块(700ms)无同步 → 放弃 */
#define VD_MIN_PILOT_TRANS   2U     /* 至少 2 次 1500/2400 交替 */

/* 判决置信度: 主频幅度² / 次强幅度² ≥ 该比值, 否则输出擦除 0xFF.
 * 纯噪声各 bin 相近 → 比值≈1 → 擦除; 弱但干净的载波 → 比值大 → 解出.
 * 这是"无绝对幅值门限"的关键 (复核缺陷 1). */
#define VD_CONF_RATIO      1.6f

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

/* 单频幅度² (±1 bin 取最大, 容忍频偏 — 复核缺陷 4) */
static float freq_mag2(const uint16_t *win, int N, int center_k, float dc)
{
    float m = goertzel_mag2(win, N, center_k, dc);
    float a = goertzel_mag2(win, N, center_k - 1, dc);
    float b = goertzel_mag2(win, N, center_k + 1, dc);
    if (a > m) m = a;
    if (b > m) m = b;
    return m;
}

uint8_t VoiceDSP_Classify(const uint16_t *win, uint16_t N, float *conf_out)
{
    /* 去 DC: 用窗口均值 (复核缺陷 3, 全窗均值而非固定 2048) */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < N; i++) sum += win[i];
    float dc = (float)sum / (float)N;

    float mag[4];
    for (int i = 0; i < 4; i++)
        mag[i] = freq_mag2(win, (int)N, vd_center_k[i], dc) * vd_freq_weight[i];

    int best = 0; float bestv = mag[0], second = 0.0f;
    for (int i = 1; i < 4; i++) if (mag[i] > bestv) { bestv = mag[i]; best = i; }
    for (int i = 0; i < 4; i++) if (i != best && mag[i] > second) second = mag[i];

    float ratio = (second > 1.0f) ? (bestv / second) : (bestv > 1.0f ? 1000.0f : 0.0f);
    if (conf_out) *conf_out = ratio;

    if (ratio < VD_CONF_RATIO) return 0xFFU;   /* 擦除, 交给 FEC */
    return (uint8_t)best;
}

/* ========================================================================== */
/*  一阶差分能量 (高通, 免疫 DC/50Hz — 复核缺陷 3)                              */
/* ========================================================================== */
uint32_t VoiceDSP_DiffEnergy(const uint16_t *blk, uint16_t n)
{
    uint32_t e = 0;
    for (uint16_t i = 1; i < n; i++) {
        int32_t d = (int32_t)blk[i] - (int32_t)blk[i - 1];
        if (d < 0) d = -d;
        e += (uint32_t)d;
    }
    return e;
}

/* 从环形缓冲取以绝对位置 abs_start 起的 N 个样本 (调用方保证仍在缓冲内) */
static void ring_extract(uint32_t abs_start, uint16_t N, uint16_t *out)
{
    for (uint16_t i = 0; i < N; i++) {
        uint32_t a = abs_start + i;
        uint16_t idx = (uint16_t)(a % VD_RING);
        out[i] = vd_ring[idx];
    }
}

/* ========================================================================== */
/*  同步音上升沿回扫: 在最近的样本里找 1800Hz tone 起点 (绝对坐标)              */
/* ========================================================================== */
static uint32_t sync_find_onset(uint32_t detect_abs)
{
    /* 检测发生时, 最新的 160 窗判为 1800 主导 → tone 覆盖 [detect-160, detect].
     * tone 长 320 → 起点 ∈ [detect-320, detect-160]. 回扫更宽一点取上升沿. */
    uint16_t tmp[VD_WIN];
    float peak = 0.0f;
    uint32_t lo = (detect_abs > 400U) ? (detect_abs - 400U) : 0U;
    uint32_t hi = (detect_abs > 160U) ? (detect_abs - 160U) : 0U;

    /* 先求峰值 */
    for (uint32_t p = lo; p <= hi; p += 8U) {
        ring_extract(p, VD_WIN, tmp);
        uint32_t sum = 0; for (int i = 0; i < (int)VD_WIN; i++) sum += tmp[i];
        float dc = (float)sum / (float)VD_WIN;
        float m = freq_mag2(tmp, VD_WIN, vd_center_k[VP_SYNC_DIGIT], dc);
        if (m > peak) peak = m;
    }
    /* 再取最早越过 0.5×峰值的位置 = 上升沿 */
    for (uint32_t p = lo; p <= hi; p += 8U) {
        ring_extract(p, VD_WIN, tmp);
        uint32_t sum = 0; for (int i = 0; i < (int)VD_WIN; i++) sum += tmp[i];
        float dc = (float)sum / (float)VD_WIN;
        float m = freq_mag2(tmp, VD_WIN, vd_center_k[VP_SYNC_DIGIT], dc);
        if (m >= 0.5f * peak) return p;
    }
    return lo;
}

/* ========================================================================== */
/*  状态机                                                                     */
/* ========================================================================== */
void VoiceRx_Init(VoiceRx *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->state = VD_LISTEN;
    rx->noise_floor = VD_EN_FLOOR_INIT;
}

void VoiceRx_Start(VoiceRx *rx)
{
    uint32_t nf = rx->noise_floor ? rx->noise_floor : VD_EN_FLOOR_INIT;
    memset(rx, 0, sizeof(*rx));
    rx->state = VD_LISTEN;
    rx->noise_floor = nf;
    memset(vd_ring, 0, sizeof(vd_ring));
    vd_ring_pos = 0;
    vd_total = 0;
    vd_grid_start = 0;
}

/* 分类当前"已收满 160 样本的判决窗", 存入符号数组 */
static void data_store_symbol(VoiceRx *rx)
{
    float conf;
    uint8_t d = VoiceDSP_Classify(rx->win_buf, VD_WIN, &conf);
    rx->last_digit = d;
    rx->last_conf = conf;
    if (rx->sym_count < VP_MAX_DATA_SYMBOLS) {
        rx->symbols[rx->sym_count++] = d;
    }
    /* 收满 LEN 前缀后求期望符号数 */
    if (rx->sym_count == VP_LEN_SYMBOLS && rx->sym_expected == 0U) {
        uint8_t plen = VoiceFEC_DecodeLen(rx->symbols);
        if (plen == 0U || plen > VP_MAX_PAYLOAD_BYTES) {
            /* 长度非法 → 放弃, 回监听 */
            rx->state = VD_LISTEN;
            rx->hi_run = rx->lo_run = 0;
            return;
        }
        rx->sym_expected = VoiceFEC_DataSymbolCount(plen);
    }
    /* 收满整帧 → 解码 */
    if (rx->sym_expected != 0U && rx->sym_count >= rx->sym_expected) {
        rx->crc_ok = VoiceFEC_ParseDataSymbols(rx->symbols, rx->sym_count,
                                               rx->payload, &rx->payload_len);
        rx->state = VD_DONE;
    }
}

uint8_t VoiceRx_PushBlock(VoiceRx *rx, const uint16_t *blk)
{
    /* 1. 写入原始环形缓冲 */
    for (uint16_t i = 0; i < VD_BLOCK; i++) {
        vd_ring[vd_ring_pos] = blk[i];
        vd_ring_pos = (uint16_t)((vd_ring_pos + 1U) % VD_RING);
    }
    uint32_t block_start_abs = vd_total;   /* 本块首样本的绝对坐标 */
    vd_total += VD_BLOCK;

    /* 2. 能量 (差分, 高通) + 自适应唤醒门限 */
    uint32_t energy = VoiceDSP_DiffEnergy(blk, VD_BLOCK);
    rx->last_energy = energy;
    uint32_t thr = rx->noise_floor + rx->noise_floor / 2U + VD_EN_MARGIN;  /* noise×1.5 */
    if (thr < VD_EN_MIN) thr = VD_EN_MIN;
    uint8_t hi = (energy >= thr) ? 1U : 0U;

    if (rx->state == VD_LISTEN && rx->startup_quiet < VD_STARTUP_QUIET) {
        /* 上电快速底噪标定: 用原始能量统计学习, 不依赖 HI/LO 判决.
         * 关键: 只要本块能量已越过当前唤醒门限 (thr, 已含 VD_EN_MIN 下限),
         * 就判定"疑似信号到达" → 立即结束标定并让本块正常参与 HI 判决,
         * 避免首帧紧跟 RX_Start 时前导被标定窗吞掉的竞态. 用 thr 而非
         * 额外 delta, 保证与后续正常判决同一标准, 不会漏检也不会误锁. */
        if (energy >= thr && rx->startup_quiet > 0U) {
            rx->startup_quiet = VD_STARTUP_QUIET;   /* 结束标定 */
            /* hi 已按 (energy>=thr) 置位, 本块正常进入状态机 */
        } else {
            rx->noise_floor = (rx->startup_quiet == 0U)
                ? energy : (rx->noise_floor * 7U + energy) / 8U;
            if (rx->startup_quiet < VD_STARTUP_QUIET) rx->startup_quiet++;
            hi = 0U; rx->hi_run = 0U;
        }
    } else if (rx->state == VD_LISTEN && !hi) {
        rx->noise_floor = (rx->noise_floor * 15U + energy) / 16U;  /* 慢速 IIR */
    }

    if (hi) { rx->hi_run++; rx->lo_run = 0; }
    else    { rx->lo_run++; rx->hi_run = 0; }

    switch (rx->state) {

    case VD_LISTEN:
        if (rx->hi_run >= VD_PREAMBLE_MIN_HI) {
            rx->state = VD_PREAMBLE;
            rx->block_in_pre = 0;
            rx->pilot_last = 0xFFU;
            rx->pilot_trans = 0;
        }
        break;

    case VD_PREAMBLE: {
        rx->block_in_pre++;
        if (rx->block_in_pre > VD_PRE_TIMEOUT || rx->lo_run >= 20U) {
            rx->state = VD_LISTEN; rx->hi_run = 0; break;
        }
        /* 用最新 160 样本 (若已累计) 分类, 观察导频交替 + 同步音 */
        if (vd_total < VD_WIN) break;
        uint16_t win[VD_WIN];
        ring_extract(vd_total - VD_WIN, VD_WIN, win);
        float conf;
        uint8_t d = VoiceDSP_Classify(win, VD_WIN, &conf);

        if (d == VP_PILOT_LO || d == VP_PILOT_HI) {
            if (rx->pilot_last != 0xFFU && d != rx->pilot_last) rx->pilot_trans++;
            rx->pilot_last = d;
        } else if (d == VP_SYNC_DIGIT && rx->pilot_trans >= VD_MIN_PILOT_TRANS) {
            /* 同步音! 回扫上升沿, 锁定栅格 */
            uint32_t tone_start = sync_find_onset(vd_total);
            vd_grid_start = tone_start + VP_SLOT_SAMPLES;  /* 跳过同步 slot */
            rx->state = VD_DATA;
            rx->sym_count = 0;
            rx->sym_expected = 0;
            rx->win_fill = 0;
            rx->lo_run = 0;
        }
        break;
    }

    case VD_DATA: {
        /* long silence (>500ms) -> abandon */
        if (rx->lo_run >= 100U) { rx->state = VD_LISTEN; rx->hi_run = 0; break; }

        /* collect decision window on the free-running grid, sample by sample */
        for (uint16_t i = 0; i < VD_BLOCK; i++) {
            uint32_t abspos = block_start_abs + i;
            if (abspos < vd_grid_start) continue;
            uint32_t rel = abspos - vd_grid_start;
            uint32_t sym = rel / VP_SLOT_SAMPLES;
            uint32_t pos = rel % VP_SLOT_SAMPLES;
            if (sym != rx->sym_count) continue;         /* only current symbol */
            if (pos < VD_WIN_OFFSET) continue;          /* skip 5ms onset */
            uint32_t w = pos - VD_WIN_OFFSET;
            if (w < VD_WIN) {
                rx->win_buf[w] = blk[i];
                rx->win_fill = (uint16_t)(w + 1U);
                if (rx->win_fill == VD_WIN) {
                    data_store_symbol(rx);
                    rx->win_fill = 0;
                    if (rx->state != VD_DATA) break;    /* DONE/LISTEN */
                }
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
