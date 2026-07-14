/**
  ******************************************************************************
  * @file           : voice_fec.c
  * @brief          : 声语信使 v5 链路层实现 — Hamming(7,4) + 交织 + CRC-8 + 变长帧
  *
  *  纯逻辑, 无 HAL 依赖. 收发端共用同一份实现, 保证编解码严格互逆.
  *  仅使用静态/栈缓冲, 无动态内存, 适合 MCU.
  ******************************************************************************
  */
#include "voice_fec.h"

/* ========================================================================== */
/*  CRC-8 (poly 0x07, init 0x00) — 逐位实现, 省一张 256B 表                     */
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
/*  Hamming(7,4) — 标准位置布局 pos1..7 = p1 p2 d1 p3 d2 d3 d4                 */
/*  parity: p1=d1^d2^d4, p2=d1^d3^d4, p3=d2^d3^d4                              */
/*  syndrome = 出错位置(1..7), 0=无错. 单比特纠错.                              */
/* ========================================================================== */
uint8_t VoiceFEC_HammingEncode(uint8_t nibble)
{
    uint8_t d1 = (uint8_t)((nibble >> 0) & 1U);
    uint8_t d2 = (uint8_t)((nibble >> 1) & 1U);
    uint8_t d3 = (uint8_t)((nibble >> 2) & 1U);
    uint8_t d4 = (uint8_t)((nibble >> 3) & 1U);
    uint8_t p1 = (uint8_t)(d1 ^ d2 ^ d4);
    uint8_t p2 = (uint8_t)(d1 ^ d3 ^ d4);
    uint8_t p3 = (uint8_t)(d2 ^ d3 ^ d4);
    /* bit6..bit0 = pos1..pos7 */
    return (uint8_t)((p1 << 6) | (p2 << 5) | (d1 << 4) |
                     (p3 << 3) | (d2 << 2) | (d3 << 1) | (d4 << 0));
}

uint8_t VoiceFEC_HammingDecode(uint8_t code7, uint8_t *corrected)
{
    uint8_t r1 = (uint8_t)((code7 >> 6) & 1U);
    uint8_t r2 = (uint8_t)((code7 >> 5) & 1U);
    uint8_t r3 = (uint8_t)((code7 >> 4) & 1U);
    uint8_t r4 = (uint8_t)((code7 >> 3) & 1U);
    uint8_t r5 = (uint8_t)((code7 >> 2) & 1U);
    uint8_t r6 = (uint8_t)((code7 >> 1) & 1U);
    uint8_t r7 = (uint8_t)((code7 >> 0) & 1U);

    uint8_t s1 = (uint8_t)(r1 ^ r3 ^ r5 ^ r7);  /* 覆盖 pos 1,3,5,7 */
    uint8_t s2 = (uint8_t)(r2 ^ r3 ^ r6 ^ r7);  /* 覆盖 pos 2,3,6,7 */
    uint8_t s3 = (uint8_t)(r4 ^ r5 ^ r6 ^ r7);  /* 覆盖 pos 4,5,6,7 */
    uint8_t syn = (uint8_t)((s3 << 2) | (s2 << 1) | s1);  /* 出错位置 1..7 */

    if (corrected) *corrected = 0U;
    if (syn != 0U) {
        if (corrected) *corrected = 1U;
        uint8_t bitpos = (uint8_t)(7U - syn);  /* pos1→bit6 ... pos7→bit0 */
        code7 ^= (uint8_t)(1U << bitpos);
        /* 重新提取数据位 */
        r3 = (uint8_t)((code7 >> 4) & 1U);
        r5 = (uint8_t)((code7 >> 2) & 1U);
        r6 = (uint8_t)((code7 >> 1) & 1U);
        r7 = (uint8_t)((code7 >> 0) & 1U);
    }
    /* d1=pos3=r3, d2=pos5=r5, d3=pos6=r6, d4=pos7=r7 */
    return (uint8_t)((r7 << 3) | (r6 << 2) | (r5 << 1) | (r3 << 0));
}

/* ========================================================================== */
/*  内部: 比特流 / 交织 辅助                                                    */
/* ========================================================================== */

/* 把一个字节编码为 2 个 Hamming 码字 (高 nibble 在前), 追加到 code7 数组 */
static uint16_t byte_to_codewords(uint8_t byte, uint8_t *cw, uint16_t idx)
{
    cw[idx++] = VoiceFEC_HammingEncode((uint8_t)((byte >> 4) & 0x0FU));  /* 高 nibble */
    cw[idx++] = VoiceFEC_HammingEncode((uint8_t)(byte & 0x0FU));         /* 低 nibble */
    return idx;
}

/* 块交织: ncw 个 7-bit 码字, 写作 ncw 行 × 7 列, 按列读出到 bit 流.
 * 连续输出比特属于连续码字 → 突发错误分散到不同码字, 每码字至多受损少量比特. */
static uint16_t interleave_codewords(const uint8_t *cw, uint16_t ncw, uint8_t *out_bits)
{
    uint16_t o = 0;
    for (uint8_t col = 0; col < 7U; col++) {
        for (uint16_t row = 0; row < ncw; row++) {
            /* 码字 bit6..bit0 = pos1..pos7; col=0 取 bit6 */
            out_bits[o++] = (uint8_t)((cw[row] >> (6U - col)) & 1U);
        }
    }
    return o;  /* = ncw * 7 */
}

/* 反交织: 由 bit 流恢复 ncw 个 7-bit 码字 */
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

/* 比特流 → 符号 (每 2 bit 一个符号, 高位在前) */
static uint16_t bits_to_symbols(const uint8_t *bits, uint16_t nbits, uint8_t *syms)
{
    uint16_t ns = 0;
    for (uint16_t i = 0; i + 1U < nbits; i += 2U) {
        syms[ns++] = (uint8_t)((bits[i] << 1) | bits[i + 1U]);
    }
    return ns;
}

/* 符号 → 比特流 (擦除符号 0xFF → 两个 0 比特) */
static uint16_t symbols_to_bits(const uint8_t *syms, uint16_t ns, uint8_t *bits)
{
    uint16_t nb = 0;
    for (uint16_t i = 0; i < ns; i++) {
        uint8_t s = syms[i];
        if (s > 3U) s = 0U;  /* 擦除: 当作 0, 交由 Hamming 尝试纠错 */
        bits[nb++] = (uint8_t)((s >> 1) & 1U);
        bits[nb++] = (uint8_t)(s & 1U);
    }
    return nb;
}

/* ========================================================================== */
/*  帧尺寸                                                                     */
/* ========================================================================== */
uint16_t VoiceFEC_DataSymbolCount(uint8_t payload_len)
{
    return (uint16_t)(VP_LEN_SYMBOLS +
                      (uint16_t)(payload_len + VP_CRC_BYTES) * VP_SYMS_PER_BYTE);
}

/* ========================================================================== */
/*  编码: payload → 数据区符号                                                 */
/* ========================================================================== */
uint16_t VoiceFEC_BuildDataSymbols(const uint8_t *payload, uint8_t payload_len,
                                   uint8_t *out_syms)
{
    if (payload_len > VP_MAX_PAYLOAD_BYTES) return 0U;

    uint8_t  cw[VP_CODED_MAX_BYTES * 2U];   /* 每字节 2 码字 */
    uint8_t  bits[VP_CODED_MAX_BYTES * 14U];
    uint16_t nsym = 0U;

    /* ---- 1. LEN 前缀: 三重冗余 (每份独立 7 符号 Hamming 块) ---- */
    for (uint8_t rep = 0; rep < VP_LEN_COPIES; rep++) {
        uint16_t ncw = byte_to_codewords(payload_len, cw, 0U);   /* 2 码字 */
        uint16_t nb  = interleave_codewords(cw, ncw, bits);      /* 14 bit */
        nsym += bits_to_symbols(bits, nb, out_syms + nsym);      /* 7 符号 */
    }

    /* ---- 2. payload + CRC 一起交织 ---- */
    {
        uint8_t body[VP_MAX_PAYLOAD_BYTES + VP_CRC_BYTES];
        uint16_t blen = 0U;
        for (uint8_t i = 0; i < payload_len; i++) body[blen++] = payload[i];
        /* CRC over LEN 字节 + payload, 保证长度也被保护 */
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

/* ========================================================================== */
/*  解码: 符号 → payload                                                       */
/* ========================================================================== */
uint8_t VoiceFEC_DecodeLen(const uint8_t *syms)
{
    /* 每份 7 符号 → 1 字节; 三份逐 bit 多数表决. */
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
    /* 逐 bit 多数表决 (3 份, 每 bit 取出现次数多者) */
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
    if (sym_count < VP_LEN_SYMBOLS) return 0U;

    uint8_t payload_len = VoiceFEC_DecodeLen(syms);
    if (payload_len > VP_MAX_PAYLOAD_BYTES) return 0U;

    uint16_t body_bytes = (uint16_t)(payload_len + VP_CRC_BYTES);
    uint16_t need = VoiceFEC_DataSymbolCount(payload_len);
    if (need > sym_count) return 0U;

    /* body 符号区紧跟在 LEN 之后 */
    const uint8_t *body_syms = syms + VP_LEN_SYMBOLS;
    uint16_t body_nsym = (uint16_t)(body_bytes * VP_SYMS_PER_BYTE);

    uint8_t  bits[VP_CODED_MAX_BYTES * 14U];
    uint8_t  cw[VP_CODED_MAX_BYTES * 2U];
    (void)symbols_to_bits(body_syms, body_nsym, bits);
    uint16_t ncw = (uint16_t)(body_bytes * 2U);
    deinterleave_codewords(bits, ncw, cw);

    uint8_t body[VP_MAX_PAYLOAD_BYTES + VP_CRC_BYTES];
    for (uint16_t i = 0; i < body_bytes; i++) {
        uint8_t c;
        uint8_t hi = VoiceFEC_HammingDecode(cw[i * 2U + 0U], &c);
        uint8_t lo = VoiceFEC_HammingDecode(cw[i * 2U + 1U], &c);
        body[i] = (uint8_t)((hi << 4) | lo);
    }

    uint8_t rx_crc = body[payload_len];
    uint8_t crc_in[1U + VP_MAX_PAYLOAD_BYTES];
    crc_in[0] = payload_len;
    for (uint8_t i = 0; i < payload_len; i++) crc_in[1U + i] = body[i];
    uint8_t calc = VoiceFEC_Crc8(crc_in, (uint16_t)(1U + payload_len));
    if (calc != rx_crc) return 0U;

    for (uint8_t i = 0; i < payload_len; i++) out_payload[i] = body[i];
    *out_len = payload_len;
    return 1U;
}
