/**
  ******************************************************************************
  * @file           : voice_fec.h
  * @brief          : 声语信使 v5 链路层 — Hamming(7,4) FEC + 交织 + CRC-8 + 变长帧
  *
  *  收发共享的纯逻辑模块 (无 HAL 依赖, 可在 PC 上单元测试).
  *
  *  编码链 (VoiceFEC_BuildDataSymbols):
  *     bytes[] = [LEN][payload...][CRC]
  *        LEN = payload 字节数 (header 3B + text)
  *        CRC = CRC-8 over (LEN 与 payload)
  *     每字节 → 高/低 nibble → 2 个 Hamming(7,4) 码字 (各 7 bit)
  *     全部码字比特写入比特流, 再做块交织 (行写列读)
  *     每 2 个交织后的比特 → 1 个 4-FSK 符号 (0..3)
  *
  *  解码链 (VoiceFEC_ParseDataSymbols):
  *     符号 → 2bit → 比特流 → 反交织 → 每 7 bit Hamming 纠 1 bit →
  *     每 2 nibble 合成字节 → 校验 CRC-8 → 输出 payload.
  *
  *  注意: 交织块大小取决于 "总码字比特数", 而它又取决于 LEN.
  *  因此接收端必须先解出 LEN. 为此 LEN(1 字节, 2 码字, 14 bit, 7 符号) 单独
  *  作为不交织的前缀发送/接收; 其后的 payload+CRC 一起交织.
  ******************************************************************************
  */
#ifndef __VOICE_FEC_H
#define __VOICE_FEC_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "voice_proto.h"

/* LEN 前缀: 1 字节 Hamming = 7 符号, 三重冗余多数表决 → 21 符号.
 * 长度域是整帧的单点故障 (解错则整帧丢弃), 故加重保护. */
#define VP_LEN_COPIES   3U
#define VP_LEN_SYMBOLS  (VP_LEN_BYTES * VP_SYMS_PER_BYTE * VP_LEN_COPIES)   /* 21 */

/* ---- CRC-8 (多项式 0x07, 初值 0x00) ---- */
uint8_t VoiceFEC_Crc8(const uint8_t *data, uint16_t len);

/* ---- Hamming(7,4) ---- */
/* 4-bit nibble → 7-bit codeword */
uint8_t VoiceFEC_HammingEncode(uint8_t nibble);
/* 7-bit codeword → 4-bit nibble, 单比特纠错. *corrected 置 1 表示发生纠错.
 * 返回 nibble (0..15). 双比特错误无法可靠纠正 (Hamming(7,4) 只能纠 1). */
uint8_t VoiceFEC_HammingDecode(uint8_t code7, uint8_t *corrected);

/**
  * @brief  构建完整数据区符号 (LEN 前缀 + 交织后的 payload/CRC)
  * @param  payload:     待发送字节 (header + text)
  * @param  payload_len: payload 字节数 (≤ VP_MAX_PAYLOAD_BYTES)
  * @param  out_syms:    输出符号数组 (每元素 0..3), 容量 ≥ VP_MAX_DATA_SYMBOLS
  * @retval 实际符号数; 0 表示参数非法
  */
uint16_t VoiceFEC_BuildDataSymbols(const uint8_t *payload, uint8_t payload_len,
                                   uint8_t *out_syms);

/**
  * @brief  给定 payload_len, 返回完整数据区应有的符号数 (供接收端定长采样)
  */
uint16_t VoiceFEC_DataSymbolCount(uint8_t payload_len);

/**
  * @brief  仅解码 LEN 前缀 (前 VP_LEN_SYMBOLS 个符号)
  * @param  syms: 至少 VP_LEN_SYMBOLS 个符号 (0..3, 0xFF 视为擦除→按 0 处理)
  * @retval 解出的 payload_len 字节 (0..255, 调用方需再验证范围)
  */
uint8_t VoiceFEC_DecodeLen(const uint8_t *syms);

/**
  * @brief  解析数据区符号 → payload, 校验 CRC
  * @param  syms:        数据区符号 (含 LEN 前缀), 0xFF 表示擦除符号
  * @param  sym_count:   syms 中的符号数
  * @param  out_payload: 输出 payload 缓冲 (≥ VP_MAX_PAYLOAD_BYTES)
  * @param  out_len:     输出 payload 字节数
  * @retval 1 = CRC 通过; 0 = 失败 (长度非法或 CRC 不符)
  */
uint8_t VoiceFEC_ParseDataSymbols(const uint8_t *syms, uint16_t sym_count,
                                  uint8_t *out_payload, uint8_t *out_len);

#ifdef __cplusplus
}
#endif
#endif /* __VOICE_FEC_H */
