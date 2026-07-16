/**
  ******************************************************************************
  * @file           : voice_fec.h
  * @brief          : 声语信使 v5.1 链路层 — Hamming(7,4) FEC + 交织 + CRC-8 + 软判决
  *
  *  收发共享的纯逻辑模块 (无 HAL 依赖, 可在 PC 上单元测试).
  *
  *  编码链 (VoiceFEC_BuildDataSymbols):
  *     bytes[] = [LEN][payload...][CRC]
  *     每字节 → 高/低 nibble → 2 个 Hamming(7,4) 码字 (各 7 bit)
  *     全部码字比特写入比特流, 再做块交织 (行写列读)
  *     每 2 个交织后的比特 → 1 个 4-FSK 符号 (0..3)
  *
  *  v5.1 软判决解码 (VoiceFEC_ParseDataSymbolsSoft):
  *     符号 + Goertzel mag² → LLR (LUT 查表) → 软 Hamming Chase 解码 →
  *     反交织 → 字节组装 → CRC 校验.
  *     约 2dB 编码增益相对于硬判决.
  *
  *  LEN 前缀: 1 字节 × 3 重冗余 × Hamming(7,4) = 21 符号.
  *  LEN 是关键单点故障, 用三重冗余多数表决保护.
  ******************************************************************************
  */
#ifndef __VOICE_FEC_H
#define __VOICE_FEC_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "voice_proto.h"

/* LEN 前缀常量 */
#define VP_LEN_COPIES   3U
#define VP_LEN_SYMBOLS  (VP_LEN_BYTES * VP_SYMS_PER_BYTE * VP_LEN_COPIES)   /* 21 */

/* ---- CRC-8 (多项式 0x07, 初值 0x00) ---- */
uint8_t VoiceFEC_Crc8(const uint8_t *data, uint16_t len);

/* ---- Hamming(7,4) 硬判决 ---- */
uint8_t VoiceFEC_HammingEncode(uint8_t nibble);
uint8_t VoiceFEC_HammingDecode(uint8_t code7, uint8_t *corrected);

/* ---- 硬判决编码/解码 API ---- */
uint16_t VoiceFEC_BuildDataSymbols(const uint8_t *payload, uint8_t payload_len,
                                   uint8_t *out_syms);
uint16_t VoiceFEC_DataSymbolCount(uint8_t payload_len);
uint8_t  VoiceFEC_DecodeLen(const uint8_t *syms);
uint8_t  VoiceFEC_ParseDataSymbols(const uint8_t *syms, uint16_t sym_count,
                                   uint8_t *out_payload, uint8_t *out_len);

/* ── v5.1 软判决 Chase 解码 ── */
/**
  * @brief  LUT 查表法 LLR 计算
  * @param  E1:  bit=1 的 Goertzel mag² 累加
  * @param  E0:  bit=0 的 Goertzel mag² 累加
  * @retval LLR = sign × ln(max(E1,E0)/min(E1,E0)), 范围约 ±4.6
  *
  *  4-FSK digit → bits 映射: 0→00, 1→01, 2→10, 3→11
  *  bit0: E1=mag[1]+mag[3], E0=mag[0]+mag[2]
  *  bit1: E1=mag[2]+mag[3], E0=mag[0]+mag[1]
  */
float VoiceFEC_ComputeLLR(float E1, float E0);

/**
  * @brief  软判决解析数据区符号 → payload (Chase 算法)
  * @param  syms:        硬判符号 (0..3 或 0xFF=擦除)
  * @param  mag2:        每符号的 4 频 Goertzel mag² [sym_count][4]
  * @param  sym_count:   符号数
  * @param  out_payload: 输出 payload
  * @param  out_len:     输出 payload 字节数
  * @retval 1 = CRC 通过; 0 = 失败
  *
  *  流程: syms+mag2 → LLR bit 流 → 反交织 → Chase Hamming 软解码 → CRC 校验.
  *  LEN 前缀仍用三重冗余多数表决 (足够鲁棒).
  */
uint8_t VoiceFEC_ParseDataSymbolsSoft(
    const uint8_t *syms,
    const float    mag2[][4],
    uint16_t       sym_count,
    uint8_t       *out_payload,
    uint8_t       *out_len);

#ifdef __cplusplus
}
#endif
#endif /* __VOICE_FEC_H */
