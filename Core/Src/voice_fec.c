/**
  ******************************************************************************
  * @file           : voice_fec.c
  * @brief          : 声语信使 v5.1 链路层 — Hamming(7,4) + 交织 + CRC-8 + 软判决 Chase
  *
  *  纯逻辑, 无 HAL 依赖. 收发端共用同一份实现, 保证编解码严格互逆.
  *
  *  v5.1 新增:
 *   - LLR 查表 (归一化尾数 LUT, 范围 1.0~100.0)
  *   - 软判决 Chase 解码: 每码字找 2 个最不可靠 bit, 试 4 种翻转组合,
  *     取最小软距离的候选. ~2dB 增益相对于硬判决.
  ******************************************************************************
  */
#include "voice_fec.h"
#include <string.h>

/* ========================================================================== */
/*  CRC-8 (poly 0x07, init 0x00)                                             */
/* ========================================================================== */
uint8_t VoiceFEC_Crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00U;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ========================================================================== */
/*  Hamming(7,4) — 硬判决                                                     */
/* ========================================================================== */
uint8_t VoiceFEC_HammingEncode(uint8_t nibble)
{
    uint8_t d1 = (nibble >> 0) & 1U;
    uint8_t d2 = (nibble >> 1) & 1U;
    uint8_t d3 = (nibble >> 2) & 1U;
    uint8_t d4 = (nibble >> 3) & 1U;
    uint8_t p1 = d1 ^ d2 ^ d4;
    uint8_t p2 = d1 ^ d3 ^ d4;
    uint8_t p3 = d2 ^ d3 ^ d4;
    return (uint8_t)((p1 << 6) | (p2 << 5) | (d1 << 4) |
                     (p3 << 3) | (d2 << 2) | (d3 << 1) | (d4 << 0));
}

uint8_t VoiceFEC_HammingDecode(uint8_t code7, uint8_t *corrected)
{
    uint8_t r1 = (code7 >> 6) & 1U;
    uint8_t r2 = (code7 >> 5) & 1U;
    uint8_t r3 = (code7 >> 4) & 1U;
    uint8_t r4 = (code7 >> 3) & 1U;
    uint8_t r5 = (code7 >> 2) & 1U;
    uint8_t r6 = (code7 >> 1) & 1U;
    uint8_t r7 = (code7 >> 0) & 1U;

    uint8_t s1 = r1 ^ r3 ^ r5 ^ r7;
    uint8_t s2 = r2 ^ r3 ^ r6 ^ r7;
    uint8_t s3 = r4 ^ r5 ^ r6 ^ r7;
    uint8_t syn = (uint8_t)((s3 << 2) | (s2 << 1) | s1);

    if (corrected) *corrected = 0U;
    if (syn != 0U) {
        if (corrected) *corrected = 1U;
        uint8_t bitpos = (uint8_t)(7U - syn);
        code7 ^= (uint8_t)(1U << bitpos);
        r3 = (code7 >> 4) & 1U;
        r5 = (code7 >> 2) & 1U;
        r6 = (code7 >> 1) & 1U;
        r7 = (code7 >> 0) & 1U;
    }
    return (uint8_t)((r7 << 3) | (r6 << 2) | (r5 << 1) | (r3 << 0));
}

/* ========================================================================== */
/*  v5.1 LLR 查表                                                             */
/* ========================================================================== */

/* ln(1 + i/16), i = 0..16. The mantissa is normalized to [1, 2), while
 * each factor of two adds ln(2). This keeps the LUT monotonic through 100:1
 * without calling logf() in the receive path. */
#define LLR_MANTISSA_STEPS 16U
#define LLR_RATIO_MAX      100.0f
#define LLR_LN2            0.69314718f

static const float llr_mantissa_lut[LLR_MANTISSA_STEPS + 1U] = {
    0.000000f, 0.060625f, 0.117783f, 0.171850f, 0.223144f, 0.271934f,
    0.318454f, 0.362906f, 0.405465f, 0.446287f, 0.485508f, 0.523248f,
    0.559616f, 0.594707f, 0.628609f, 0.661398f, 0.693147f
};

float VoiceFEC_ComputeLLR(float E1, float E0)
{
    /* 静音或纯噪声: 两侧能量都极低 → LLR≈0 (无信息) */
    if (E1 + E0 < 1e-4f) return 0.0f;

    float ratio;
    int sign;
    if (E1 >= E0) {
        ratio = (E0 > 1e-6f) ? (E1 / E0) : LLR_RATIO_MAX;
        sign = 1;
    } else {
        ratio = (E1 > 1e-6f) ? (E0 / E1) : LLR_RATIO_MAX;
        sign = -1;
    }

    if (ratio > LLR_RATIO_MAX) ratio = LLR_RATIO_MAX;
    if (ratio < 1.0f) ratio = 1.0f;

    uint8_t octaves = 0U;
    while (ratio >= 2.0f && octaves < 6U) {
        ratio *= 0.5f;
        octaves++;
    }
    uint8_t index = (uint8_t)((ratio - 1.0f) * (float)LLR_MANTISSA_STEPS);
    if (index > LLR_MANTISSA_STEPS) index = LLR_MANTISSA_STEPS;

    return (float)sign * ((float)octaves * LLR_LN2 + llr_mantissa_lut[index]);
}

/* ========================================================================== */
/*  内部: 比特流 / 交织 (硬判决)                                              */
/* ========================================================================== */

static uint16_t byte_to_codewords(uint8_t byte, uint8_t *cw, uint16_t idx)
{
    cw[idx++] = VoiceFEC_HammingEncode((uint8_t)((byte >> 4) & 0x0FU));
    cw[idx++] = VoiceFEC_HammingEncode((uint8_t)(byte & 0x0FU));
    return idx;
}

static uint16_t interleave_codewords(const uint8_t *cw, uint16_t ncw, uint8_t *out_bits)
{
    uint16_t o = 0;
    for (uint8_t col = 0; col < 7U; col++) {
        for (uint16_t row = 0; row < ncw; row++) {
            out_bits[o++] = (uint8_t)((cw[row] >> (6U - col)) & 1U);
        }
    }
    return o;
}

static void deinterleave_codewords(const uint8_t *in_bits, uint16_t ncw, uint8_t *cw)
{
    for (uint16_t i = 0; i < ncw; i++) cw[i] = 0U;
    uint16_t o = 0;
    for (uint8_t col = 0; col < 7U; col++) {
        for (uint16_t row = 0; row < ncw; row++) {
            if (in_bits[o++]) cw[row] |= (uint8_t)(1U << (6U - col));
        }
    }
}

/* 符号 → 2 bit (硬判) */
static uint16_t symbols_to_bits(const uint8_t *syms, uint16_t nsym, uint8_t *bits)
{
    for (uint16_t i = 0; i < nsym; i++) {
        uint8_t d = syms[i];
        if (d > 3U) d = 0U;
        bits[i * 2U + 0U] = (uint8_t)((d >> 1) & 1U);
        bits[i * 2U + 1U] = (uint8_t)(d & 1U);
    }
    return (uint16_t)(nsym * 2U);
}

static uint16_t bits_to_symbols(const uint8_t *bits, uint16_t nbit, uint8_t *syms)
{
    for (uint16_t i = 0; i < nbit / 2U; i++) {
        syms[i] = (uint8_t)((bits[i * 2U] << 1) | bits[i * 2U + 1U]);
    }
    return (uint16_t)(nbit / 2U);
}

/* ========================================================================== */
/*  v5.1 软判决: LLR 反交织 + Chase Hamming 软解码                            */
/* ========================================================================== */

/* 从符号 + mag² 计算 bit 级 LLR 数组, 同时做反交织 */
static void symbols_to_llr(const uint8_t *syms, const float mag2[][4],
                           uint16_t sym_count,
                           float *llr_out, uint16_t *llr_len)
{
    /* 每符号 2 bit → 2 LLR */
    uint16_t nb = sym_count * 2U;
    for (uint16_t i = 0; i < sym_count; i++) {
        uint8_t d = syms[i];
        float m2[4];
        if (mag2) {
            for (int j = 0; j < 4; j++) m2[j] = mag2[i][j];
        } else {
            /* 无 mag2: 用 hard-decision 近似 LLR (±1.0) */
            for (int j = 0; j < 4; j++) m2[j] = (j == d) ? 1000000.0f : 1.0f;
        }

        /* symbols_to_bits() emits MSB first, then LSB. */
        float E1_msb = m2[2] + m2[3];
        float E0_msb = m2[0] + m2[1];
        float E1_lsb = m2[1] + m2[3];
        float E0_lsb = m2[0] + m2[2];

        llr_out[i * 2U + 0U] = VoiceFEC_ComputeLLR(E1_msb, E0_msb);
        llr_out[i * 2U + 1U] = VoiceFEC_ComputeLLR(E1_lsb, E0_lsb);
    }
    *llr_len = nb;
}

/* 对 LLR 数组做反交织: LLR 值的排列与 bit 硬判决一致 */
static void deinterleave_llr(const float *llr_in, uint16_t ncw, float *llr_cw)
{
    /* ncw 个码字, 每码字 7 bit → 7×ncw 个 LLR */
    for (uint16_t c = 0; c < ncw; c++) {
        for (uint8_t b = 0; b < 7U; b++) llr_cw[c * 7U + b] = 0.0f;
    }
    uint16_t o = 0;
    for (uint8_t col = 0; col < 7U; col++) {
        for (uint16_t row = 0; row < ncw; row++) {
            llr_cw[row * 7U + (6U - col)] = llr_in[o++];
        }
    }
}

/* Chase 软 Hamming 解码一个 7-bit 码字:
 *   hard[7]:   硬判 bit (0/1)
 *   llr_abs[7]: 各 bit 的 |LLR| (置信度)
 * 返回 4-bit nibble.
 *
 *  算法: 找 2 个最不可靠 bit, 试 4 种翻转组合.
 *  每种组合做 Hamming decode, 取软距离最小的候选.
 */
static uint8_t chase_hamming_decode(const uint8_t *hard, const float *llr_abs)
{
    /* 原始硬判 → Hamming decode */
    uint8_t code7 = 0U;
    for (int i = 0; i < 7; i++) {
        if (hard[i]) code7 |= (uint8_t)(1U << (6U - i));
    }
    uint8_t corr;
    uint8_t nibble_hard = VoiceFEC_HammingDecode(code7, &corr);

    /* 找 2 个最不可靠 bit (最小 |LLR|) */
    int worst1 = -1, worst2 = -1;
    float min1 = 1e10f, min2 = 1e10f;
    for (int i = 0; i < 7; i++) {
        float a = llr_abs[i];
        if (a < min1) {
            min2 = min1; worst2 = worst1;
            min1 = a;    worst1 = i;
        } else if (a < min2) {
            min2 = a;    worst2 = i;
        }
    }

    /* 若 |LLR| 都很高 (所有 bit 都可靠), 直接返回硬判结果 */
    if (min1 > 3.0f) return nibble_hard;

    /* 尝试 4 种翻转组合, 选最小软距离 */
    uint8_t best_nibble = nibble_hard;
    float   best_dist   = 1e10f;

    for (uint8_t pat = 0; pat < 4U; pat++) {
        uint8_t test7 = code7;
        float dist = 0.0f;

        /* 翻转 bit worst1 (pat bit0=1) */
        if (pat & 1U) {
            test7 ^= (uint8_t)(1U << (6U - worst1));
            dist += llr_abs[worst1];
        }
        /* 翻转 bit worst2 (pat bit1=1) */
        if (pat & 2U && worst2 >= 0) {
            test7 ^= (uint8_t)(1U << (6U - worst2));
            dist += llr_abs[worst2];
        }

        /* Hamming decode */
        uint8_t cand_corr;
        uint8_t nibble = VoiceFEC_HammingDecode(test7, &cand_corr);

        /* 软距离 = 未翻转 bit 的 |LLR| 加权错误贡献 */
        /* 重编码候选 nibble 得到理想码字, 计算与原始 hard 的距离 */
        uint8_t ideal = VoiceFEC_HammingEncode(nibble);
        float metric = 0.0f;
        for (int i = 0; i < 7; i++) {
            uint8_t ideal_bit = (ideal >> (6U - i)) & 1U;
            if (ideal_bit != hard[i]) {
                metric += llr_abs[i];
            }
        }

        if (metric < best_dist) {
            best_dist = metric;
            best_nibble = nibble;
        }
    }

    return best_nibble;
}

/* ========================================================================== */
/*  软判决解析主函数                                                          */
/* ========================================================================== */
uint8_t VoiceFEC_ParseDataSymbolsSoft(
    const uint8_t *syms,
    const float    mag2[][4],
    uint16_t       sym_count,
    uint8_t       *out_payload,
    uint8_t       *out_len)
{
    if (sym_count < VP_LEN_SYMBOLS) return 0U;

    /* 1. LEN 解码: 三重冗余多数表决 (LEN 太短, 无需 Chase) */
    uint8_t payload_len = VoiceFEC_DecodeLen(syms);
    if (payload_len > VP_MAX_PAYLOAD_BYTES || payload_len == 0U) return 0U;

    uint16_t need = VoiceFEC_DataSymbolCount(payload_len);
    if (need > sym_count) return 0U;

    /* 2. body 符号区 (LEN 之后) */
    const uint8_t *body_syms = syms + VP_LEN_SYMBOLS;
    const float   *body_mag2 = mag2 ? &mag2[VP_LEN_SYMBOLS][0] : NULL;
    uint16_t body_nsym = (uint16_t)(VP_LEN_BYTES + payload_len + VP_CRC_BYTES) * VP_SYMS_PER_BYTE;
    uint16_t body_nbytes = (uint16_t)(payload_len + VP_CRC_BYTES);

    /* 3. 符号 → LLR */
    float llr_body[VP_CODED_MAX_BYTES * 14U];
    uint16_t llr_len;
    /* 构造本地 mag2 视图 (body 部分) */
    if (body_mag2) {
        symbols_to_llr(body_syms, (const float(*)[4])body_mag2,
                       body_nsym, llr_body, &llr_len);
    } else {
        symbols_to_llr(body_syms, NULL, body_nsym, llr_body, &llr_len);
    }

    /* 4. 反交织 LLR */
    uint16_t ncw = (uint16_t)(body_nbytes * 2U);
    float llr_cw[VP_CODED_MAX_BYTES * 14U];  /* ncw * 7 */
    deinterleave_llr(llr_body, ncw, llr_cw);

    /* 5. 每码字: 获取硬判 + |LLR|, 调用 Chase */
    /* 先获取硬判 bit (仍用 symbols_to_bits, 因 LLR 不改变硬判方向) */
    uint8_t hard_bits[VP_CODED_MAX_BYTES * 14U];
    (void)symbols_to_bits(body_syms, body_nsym, hard_bits);
    uint8_t cw_hard[VP_CODED_MAX_BYTES * 2U];
    deinterleave_codewords(hard_bits, ncw, cw_hard);

    uint8_t body_out[VP_MAX_PAYLOAD_BYTES + VP_CRC_BYTES];
    for (uint16_t i = 0; i < body_nbytes; i++) {
        /* 2 个码字 → 1 字节 */
        uint8_t nib_hi = 0, nib_lo = 0;

        for (uint8_t nib = 0; nib < 2U; nib++) {
            uint16_t ci = i * 2U + nib;
            uint8_t hard[7];
            float   abs_llr[7];
            for (int b = 0; b < 7; b++) {
                hard[b] = (cw_hard[ci] >> (6U - b)) & 1U;
                abs_llr[b] = (llr_cw[ci * 7U + b] > 0.0f)
                             ? llr_cw[ci * 7U + b]
                             : -llr_cw[ci * 7U + b];
            }
            uint8_t nibble = chase_hamming_decode(hard, abs_llr);
            if (nib == 0U) nib_hi = nibble;
            else           nib_lo = nibble;
        }
        body_out[i] = (uint8_t)((nib_hi << 4) | nib_lo);
    }

    /* 6. CRC 校验 */
    uint8_t rx_crc = body_out[payload_len];
    uint8_t crc_in[1U + VP_MAX_PAYLOAD_BYTES];
    crc_in[0] = payload_len;
    for (uint8_t i = 0; i < payload_len; i++) crc_in[1U + i] = body_out[i];
    uint8_t calc = VoiceFEC_Crc8(crc_in, (uint16_t)(1U + payload_len));
    if (calc != rx_crc) return 0U;

    /* 7. 输出 */
    for (uint8_t i = 0; i < payload_len; i++) out_payload[i] = body_out[i];
    *out_len = payload_len;
    return 1U;
}

/* ========================================================================== */
/*  硬判决编码/解码 (保留, 供 PC 测试和回退)                                  */
/* ========================================================================== */
uint16_t VoiceFEC_DataSymbolCount(uint8_t payload_len)
{
    return (uint16_t)(VP_LEN_SYMBOLS +
                      (uint16_t)(payload_len + VP_CRC_BYTES) * VP_SYMS_PER_BYTE);
}

uint16_t VoiceFEC_BuildDataSymbols(const uint8_t *payload, uint8_t payload_len,
                                   uint8_t *out_syms)
{
    if (payload_len > VP_MAX_PAYLOAD_BYTES) return 0U;

    uint8_t  cw[VP_CODED_MAX_BYTES * 2U];
    uint8_t  bits[VP_CODED_MAX_BYTES * 14U];
    uint16_t nsym = 0U;

    /* LEN 三重冗余 */
    for (uint8_t rep = 0; rep < VP_LEN_COPIES; rep++) {
        uint16_t ncw = byte_to_codewords(payload_len, cw, 0U);
        uint16_t nb  = interleave_codewords(cw, ncw, bits);
        nsym += bits_to_symbols(bits, nb, out_syms + nsym);
    }

    /* payload + CRC */
    {
        uint8_t body[VP_MAX_PAYLOAD_BYTES + VP_CRC_BYTES];
        uint16_t blen = 0U;
        for (uint8_t i = 0; i < payload_len; i++) body[blen++] = payload[i];
        uint8_t crc_in[1U + VP_MAX_PAYLOAD_BYTES];
        crc_in[0] = payload_len;
        for (uint8_t i = 0; i < payload_len; i++) crc_in[1U + i] = payload[i];
        body[blen++] = VoiceFEC_Crc8(crc_in, (uint16_t)(1U + payload_len));

        uint16_t ncw = 0U;
        for (uint16_t i = 0; i < blen; i++) ncw = byte_to_codewords(body[i], cw, ncw);
        uint16_t nb = interleave_codewords(cw, ncw, bits);
        nsym += bits_to_symbols(bits, nb, out_syms + nsym);
    }

    return nsym;
}

uint8_t VoiceFEC_DecodeLen(const uint8_t *syms)
{
    uint8_t cand[VP_LEN_COPIES];
    for (uint8_t rep = 0; rep < VP_LEN_COPIES; rep++) {
        uint8_t bits[14];
        uint8_t cw[2];
        (void)symbols_to_bits(syms + rep * (VP_LEN_BYTES * VP_SYMS_PER_BYTE),
                              VP_LEN_BYTES * VP_SYMS_PER_BYTE, bits);
        deinterleave_codewords(bits, 2U, cw);
        uint8_t corr;
        uint8_t hi = VoiceFEC_HammingDecode(cw[0], &corr);
        uint8_t lo = VoiceFEC_HammingDecode(cw[1], &corr);
        cand[rep] = (uint8_t)((hi << 4) | lo);
    }
    uint8_t out = 0U;
    for (uint8_t b = 0; b < 8; b++) {
        uint8_t ones = 0U;
        for (uint8_t rep = 0; rep < VP_LEN_COPIES; rep++)
            ones = (uint8_t)(ones + ((cand[rep] >> b) & 1U));
        if (ones * 2U > VP_LEN_COPIES) out |= (uint8_t)(1U << b);
    }
    return out;
}

uint8_t VoiceFEC_ParseDataSymbols(const uint8_t *syms, uint16_t sym_count,
                                  uint8_t *out_payload, uint8_t *out_len)
{
    /* 回退到软判决 (总是可用) */
    return VoiceFEC_ParseDataSymbolsSoft(syms, NULL, sym_count, out_payload, out_len);
}
