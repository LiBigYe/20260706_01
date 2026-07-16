/**
  ******************************************************************************
  * @file           : voice_dsp.c
  * @brief          : 声语信使 v5.1 接收 DSP 核心实现 (True SNR 分类器)
  *
  *  时基恢复策略 (关键):
  *   - 前导 (1500/2400 交替) 只用于唤醒 + 冻结噪声底 + 交替校验 (排除窄带噪声).
  *   - 1800Hz 同步音 (前导中从不出现) 是唯一的精定时锚点.
  *   - 检测到同步音后, 在原始采样环形缓冲里回扫其上升沿, 得到 sample 级
  *     的同步音起点, 由此锁定数据符号 30ms 栅格.
  *   - 数据阶段完全自由运行栅格, 只取每个 20ms tone 的中间 10ms 做判决.
  *     不依赖 guard 下降沿 → 免疫混响拖尾 (复核缺陷 2).
  *
  *  v5.1 变更 (2026-07-16):
  *   - True SNR 分类器: P_signal / (P_total - P_signal) 取代 best/second.
  *     免疫宽带白噪声 (风扇/空调), 窄带干扰 (1550Hz 电机), DC 偏置.
  *   - 低能量门限 + 双重频域锁: VD_EN_MARGIN 1000→500,
  *     但须连续 2 次 Goertzel 命中导频才进入 PREAMBLE.
  *   - 频域擦除计数: 连续 4 符号 0xFF 才判定信号丢失, 取代差分能量 lo_run.
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

/* 频响补偿权重 (作用于 Goertzel mag²), 基于模拟前端传递函数 H(jω) 计算.
 *
 * 前端: OPA1642 两级 Sallen-Key 级联带通 (LPF fc=2751Hz Q=0.74 + HPF fc=1236Hz Q=0.47)
 * 四个 FSK 频率在通带内各点衰减不同 (1/|H(f)|²):
 *   1500Hz: -5.1dB → weight=1.33  1800Hz: -4.2dB → weight=1.08
 *   2100Hz: -3.9dB → weight=1.00  2400Hz: -4.0dB → weight=1.02
 *
 * weight 作用于 mag² 比较: compensated_mag² = weight[f] * |H(f)|².
 * 平衡后四个频率对相同输入幅度的响应一致, 消除"1500Hz 天生吃亏"的判决偏置. */
static const float vd_freq_weight[4] = {1.33f, 1.08f, 1.00f, 1.02f};

/* ---- 唤醒能量门限 (自适应, 仅用于"唤醒提示", 真正判决靠频谱置信度) ----
 * 复核缺陷 1/3 对策: 能量门限只做低成本唤醒. v5.1 大幅降低门限让微弱信号进门,
 * 误唤醒由后续双重 Goertzel 频域锁 (连续2次命中) 滤除. */
#define VD_EN_FLOOR_INIT   400U
#define VD_EN_MARGIN        500U  /* 核心修改: 大幅降低门限裕量 (原1000→500), 让微弱信号进门.
                                    后续由双重 Goertzel 频域锁 (连续2次命中) 滤除噪声误触发. */
#define VD_EN_MIN           500U  /* 降低绝对唤醒下限 (原800→500) */
#define VD_STARTUP_QUIET    15U   /* ~75ms 静稳 (缩短以降低首帧竞态窗口) */

/* ---- 前导/同步参数 (样本) ---- */
#define VD_PREAMBLE_MIN_HI   6U     /* 连续 6 HI 块 (30ms) → PREAMBLE */
#define VD_PRE_TIMEOUT     140U     /* 前导内 140 块(700ms)无同步 → 放弃 */
#define VD_MIN_PILOT_TRANS   2U     /* 至少 2 次 1500/2400 交替 */

/* 频域锁: 连续 Goertzel 命中导频次数 (10ms窗口, 连续2次=20ms有效载波才放行).
 * 降低差分能量唤醒门限后, 噪声可能偶尔越过门限, 但频谱平坦的噪声不可能连续
 * 两次命中同一 FSK 频点 — 这是"低门限+严验证"策略的数学基础. */
#define VD_PILOT_HITS_REQ    2U

/* 数据段频域掉线容忍: 连续 N 个符号 Goertzel 返回 0xFF (擦除) 才判定信号丢失.
 * 每个符号 30ms, N=4 即 120ms 无有效载波 → 退出. 单/双符号的瞬时擦除
 * (由突发噪声或深衰落引起) 交给 Hamming(7,4) 纠错, 不触发退网. */
#define VD_MAX_ERASE_RUN     4U

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
/*  True SNR 分类器 (v5.1 核心改进)                                             */
/* ========================================================================== */
uint8_t VoiceDSP_Classify(const uint16_t *win, uint16_t N, float *conf_out)
{
    /* 1. 去 DC: 用窗口均值 */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < N; i++) sum += win[i];
    float dc = (float)sum / (float)N;

    /* 2. 总交流功率 P_total = 时域平方和 (所有频段能量之和).
     *    P_total = Σ(x[i] - x̄)² = N × A²/2 (纯正弦波情况).
     *    后续 True SNR 需要用此值作为"信号+噪声"总能量. */
    float p_total = 0.0f;
    for (uint16_t i = 0; i < N; i++) {
        float dev = (float)win[i] - dc;
        p_total += dev * dev;
    }

    /* 极低能量阻断: 若总交流能量近乎为零 (几乎是纯 DC 死区),
     * 直接返回擦除, 避免后续浮点运算在零附近产生非法 SNR. */
    if (p_total < 100.0f) {
        if (conf_out) *conf_out = 0.0f;
        return 0xFFU;
    }

    /* 3. Goertzel 幅度² (±1 bin 取最大, 容忍频偏).
     *    同时记录 raw_mag (不含频响权重, 用于 SNR 计算) 和
     *    wgt_mag (含频响补偿, 用于 digit 选择, 消除前端频响偏差). */
    float raw_mag[4];
    float wgt_mag[4];
    for (int i = 0; i < 4; i++) {
        float m = goertzel_mag2(win, (int)N, vd_center_k[i], dc);
        float a = goertzel_mag2(win, (int)N, vd_center_k[i] - 1, dc);
        float b = goertzel_mag2(win, (int)N, vd_center_k[i] + 1, dc);
        if (a > m) m = a;
        if (b > m) m = b;
        raw_mag[i] = m;                       /* 原始 (用于 SNR 计算) */
        wgt_mag[i] = m * vd_freq_weight[i];   /* 加权 (用于 digit 选择) */
    }

    /* 4. 找最强频点 (用加权值, 消除模拟前端 1500Hz 频响劣势) */
    int best = 0;
    float bestv = wgt_mag[0];
    for (int i = 1; i < 4; i++) {
        if (wgt_mag[i] > bestv) { bestv = wgt_mag[i]; best = i; }
    }

    /* 5. True SNR: 量纲推导 — Goertzel mag² = A²×N²/4, 时域方差 = N×A²/2.
     *   p_signal = N×A²/2 = N × (4×M²/N²) / 2 = 2×M²/N.
     *   系数 α = 2/N. 对于 N=160: α = 2/160 = 0.0125.
     *   注意: 若误用 2/N², p_signal 会被缩小 160 倍 → SNR 永 < 0.01 → 全擦除! */
    float alpha = 2.0f / (float)N;
    float p_signal = raw_mag[best] * alpha;   /* 最强载波功率 (方差单位) */

    /* 噪声 = 总能量 - 载波能量, 防浮点微小负值或除零 */
    float p_noise = p_total - p_signal;
    if (p_noise < 1.0f) p_noise = 1.0f;

    float snr = p_signal / p_noise;
    if (conf_out) *conf_out = snr;  /* v5.1: 报告 True SNR, 非旧 best/second 比值 */

    /* 6. 判决门限 */
    /* (a) 绝对载波能量下限: Goertzel mag² 低于此值说明连单频能量都不存在,
     *     即使 SNR 偶然高 (纯静音 P_total≈0), 也拒绝.
     *     ±5 LSB × N=160 → (5×160/2)² = 6.4×10⁵, 留余量取 200,000. */
    #define VD_MIN_CARRIER_MAG2  200000.0f
    if (raw_mag[best] < VD_MIN_CARRIER_MAG2) return 0xFFU;

    /* (b) 真实信噪比下限: SNR=2.0 (6dB) 是抗混叠与容忍晶振频偏的甜点.
     *     部分载波能量因频偏泄漏到相邻 bin → 被 p_total 吃进当 p_noise
     *     → SNR 自然偏低. 2.0 恰好平衡"真弱信号不丢"与"噪声不误判". */
    #define VD_MIN_SNR            2.0f
    if (snr < VD_MIN_SNR) return 0xFFU;

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

/* ========================================================================== */
/*  辅助函数                                                                   */
/* ========================================================================== */

/* 从环形缓冲取以绝对位置 abs_start 起的 N 个样本 (调用方保证仍在缓冲内) */
static void ring_extract(uint32_t abs_start, uint16_t N, uint16_t *out)
{
    for (uint16_t i = 0; i < N; i++) {
        uint32_t a = abs_start + i;
        uint16_t idx = (uint16_t)(a % VD_RING);
        out[i] = vd_ring[idx];
    }
}

/* 单频幅度² (±1 bin 取最大). sync_find_onset 内部使用,
 * 不参与 True SNR 判决 (判决走 VoiceDSP_Classify). */
static float freq_mag2(const uint16_t *win, int N, int center_k, float dc)
{
    float m = goertzel_mag2(win, N, center_k, dc);
    float a = goertzel_mag2(win, N, center_k - 1, dc);
    float b = goertzel_mag2(win, N, center_k + 1, dc);
    if (a > m) m = a;
    if (b > m) m = b;
    return m;
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
    rx->pilot_hits = 0;
    rx->erase_run  = 0;
}

/* 分类当前"已收满 160 样本的判决窗", 存入符号数组 */
static void data_store_symbol(VoiceRx *rx)
{
    float conf;
    uint8_t d = VoiceDSP_Classify(rx->win_buf, VD_WIN, &conf);
    rx->last_digit = d;
    rx->last_conf = conf;

    /* ========================================================= */
    /* 核心修改: 用频域擦除计数取代时域 lo_run 判定信号丢失      */
    /* ========================================================= */
    if (d == 0xFFU) {
        rx->erase_run++;
        if (rx->erase_run >= VD_MAX_ERASE_RUN) {
            /* 连续 N 个符号 (120ms) Goertzel 无法锁定任何 FSK 频点
             * → 信号确实丢失, 退出数据段回到监听. */
            rx->state = VD_LISTEN;
            rx->hi_run = rx->lo_run = 0;
            return;
        }
    } else {
        rx->erase_run = 0;  /* 只要出一个有效符号, 掉线危机解除 */
    }
    /* ========================================================= */

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

uint8_t VoiceRx_PushBlock(VoiceRx *rx, const uint16_t *blk, uint8_t gain_code)
{
    /* 1. 写入原始环形缓冲 */
    for (uint16_t i = 0; i < VD_BLOCK; i++) {
        vd_ring[vd_ring_pos] = blk[i];
        vd_ring_pos = (uint16_t)((vd_ring_pos + 1U) % VD_RING);
    }
    uint32_t block_start_abs = vd_total;   /* 本块首样本的绝对坐标 */
    vd_total += VD_BLOCK;

    /* 2. 能量 (差分, 高通) + 自适应唤醒门限
     *    归一化到 32x 增益等效 (增益码 5): energy_norm = energy × 32 / 2^gain_code.
     *    这样 noise_floor 和 thr 都是"等效输入"量,不被 PGA 增益档牵着走. */
    uint32_t energy = VoiceDSP_DiffEnergy(blk, VD_BLOCK);
    uint32_t energy_norm;
    if (gain_code <= 5U)
        energy_norm = energy << (5U - gain_code);          /* 增益小时数值放大归一 */
    else
        energy_norm = energy >> (gain_code - 5U);          /* 增益大时数值缩小归一 */
    rx->last_energy = energy_norm;
    uint32_t thr = rx->noise_floor + rx->noise_floor / 2U + VD_EN_MARGIN;  /* noise×1.5 */
    if (thr < VD_EN_MIN) thr = VD_EN_MIN;
    uint8_t hi = (energy_norm >= thr) ? 1U : 0U;

    if (rx->state == VD_LISTEN && rx->startup_quiet < VD_STARTUP_QUIET) {
        /* 上电快速底噪标定: 用原始能量统计学习, 不依赖 HI/LO 判决.
         * 关键: 只要本块能量已越过当前唤醒门限 (thr, 已含 VD_EN_MIN 下限),
         * 就判定"疑似信号到达" → 立即结束标定并让本块正常参与 HI 判决,
         * 避免首帧紧跟 RX_Start 时前导被标定窗吞掉的竞态. 用 thr 而非
         * 额外 delta, 保证与后续正常判决同一标准, 不会漏检也不会误锁. */
        if (energy_norm >= thr && rx->startup_quiet > 0U) {
            rx->startup_quiet = VD_STARTUP_QUIET;   /* 结束标定 */
            /* hi 已按 (energy_norm>=thr) 置位, 本块正常进入状态机 */
        } else {
            rx->noise_floor = (rx->startup_quiet == 0U)
                ? energy_norm : (rx->noise_floor * 7U + energy_norm) / 8U;
            if (rx->startup_quiet < VD_STARTUP_QUIET) rx->startup_quiet++;
            hi = 0U; rx->hi_run = 0U;
        }
    } else if (rx->state == VD_LISTEN && !hi) {
        rx->noise_floor = (rx->noise_floor * 15U + energy_norm) / 16U;  /* 慢速 IIR */
    }

    if (hi) { rx->hi_run++; rx->lo_run = 0; }
    else    { rx->lo_run++; rx->hi_run = 0; }

    switch (rx->state) {

    case VD_LISTEN:
        /* 进门策略: 低能量门限 (VD_EN_MARGIN=500) 让微弱信号进门,
         * 但必须连续 2 次 Goertzel 命中同一导频才放行.
         * 频谱平坦的噪声不可能连续两次命中 1500/2400Hz —
         * 这是"低门限宽进 + 频域严查"策略不惧误唤醒的数学基础. */
        if (rx->hi_run >= VD_PREAMBLE_MIN_HI && vd_total >= VD_WIN) {
            uint16_t lwin[VD_WIN];
            ring_extract(vd_total - VD_WIN, VD_WIN, lwin);
            float lconf;
            uint8_t ld = VoiceDSP_Classify(lwin, VD_WIN, &lconf);
            if (ld == VP_PILOT_LO || ld == VP_PILOT_HI) {
                rx->pilot_hits++;
                if (rx->pilot_hits >= VD_PILOT_HITS_REQ) {
                    /* 双重频域锁: 连续2次命中, 确认前导真实存在 */
                    rx->state = VD_PREAMBLE;
                    rx->block_in_pre = 0;
                    rx->pilot_last = ld;
                    rx->pilot_trans = 0;
                    rx->pilot_hits = 0;   /* 重置供下次使用 */
                }
            } else {
                rx->pilot_hits = 0;       /* 频谱不对, 连续计数打断 */
            }
        } else {
            rx->pilot_hits = 0;           /* 能量掉落, 连续计数打断 */
        }
        break;

    case VD_PREAMBLE: {
        rx->block_in_pre++;
        /* 移除 rx->lo_run >= 20U 退出条件: 差分能量对 1500Hz 有 ~37.5% 的幅度衰减,
         * 前导中 1500Hz 导频可能被时域能量判据误杀 → 过渡到 LISTEN → 前导白等.
         * 现在仅凭超时退出, 真失锁交给 VD_DATA 的频域擦除计数处理. */
        if (rx->block_in_pre > VD_PRE_TIMEOUT) {
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
            rx->erase_run = 0;        /* 数据段起始, 擦除计数清零 */
        }
        break;
    }

    case VD_DATA: {
        /* 彻底移除 if (rx->lo_run >= 100U) 退网判定.
         * 差分能量对 1500Hz 载波天生偏低 (dE ∝ f), 数据段中低频符号
         * (digit 0=1500Hz) 的时域能量可能跌落至门限以下 → lo_run 累加
         * → 误判掉线 → LED 熄灭 / 数据帧被掐断.
         * 真正的信号丢失判定已移交给 data_store_symbol 的频域擦除计数:
         * Goertzel 连续 4 个符号无法锁定任何 FSK 频点 → 才退出. */

        for (uint16_t i = 0; i < VD_BLOCK; i++) {
            uint32_t abspos = block_start_abs + i;
            if (abspos < vd_grid_start) continue;
            uint32_t rel = abspos - vd_grid_start;
            uint32_t sym = rel / VP_SLOT_SAMPLES;
            uint32_t pos = rel % VP_SLOT_SAMPLES;
            if (sym != rx->sym_count) continue;
            if (pos < VD_WIN_OFFSET) continue;
            uint32_t w = pos - VD_WIN_OFFSET;
            if (w < VD_WIN) {
                rx->win_buf[w] = blk[i];
                rx->win_fill = (uint16_t)(w + 1U);
                if (rx->win_fill == VD_WIN) {
                    data_store_symbol(rx);
                    rx->win_fill = 0;
                    if (rx->state != VD_DATA) break;
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
