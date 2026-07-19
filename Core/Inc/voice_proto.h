/**
  ******************************************************************************
  * @file           : voice_proto.h
  * @brief          : 声语信使 v5 物理层/链路层协议常量 (收发共享)
  *
  *  v5 相对 v4 的关键变化 (针对 DSP 复核的 5 大缺陷):
  *   (1) 取消绝对峰峰值硬门限, 改用频谱置信度 (见 fsk4_decoder).
  *   (2) 不再依赖每个符号的 guard 下降沿做时钟: 前导 + 1800Hz 同步音
  *       一次性锁定符号时基, 之后按固定 30ms(480 sample) 栅格自由运行.
  *   (3) 一阶差分能量检测 (等效高通), 对 50Hz/DC 免疫; DC 去除用全 20ms 窗.
  *   (4) Goertzel 每目标频率 + 相邻 bin 容忍, 放宽峰值比一票否决.
  *   (5) 变长帧 + Hamming(7,4) FEC + 交织 + CRC-8, 取代裸 XOR.
  *
  *  时序 (与 v4 相同, 保持 16kHz / 30ms 符号栅格):
  *     符号载波 T_TONE  = 20 ms = 320 sample
  *     保护间隔 T_GUARD = 10 ms = 160 sample
  *     符号槽   T_SLOT  = 30 ms = 480 sample
  *
  *  帧结构 (v5, 变长):
  *     [前导 PREAMBLE]  1500/2400Hz 每 40ms 交替, 共 200ms  (粗能量/AGC/导频)
  *     [同步 SYNC]      1800Hz 连续 30ms 单音 (1 符号槽)     (精定时锚点)
  *     [长度 LEN]       1 字节 payload 长度 → FEC → 符号
  *     [负载 PAYLOAD]   variable: 3 字节网络头 + N 字节正文 (N = 真实字符数)
  *     [校验 CRC]       1 字节 CRC-8 → FEC → 符号
  *     [尾部隔离槽]     30ms DC 中点
  *     [结束 POSTAMBLE] 2400Hz 连续 120ms
  *
  *  FEC/交织: LEN/PAYLOAD/CRC 全部字节先做 Hamming(7,4) (每字节 → 2 码字 →
  *  14 bit), 再对整段码字比特做块交织, 最后每 2 bit 映射为 1 个 4-FSK 符号.
  *  这样单个突发 (咳嗽/关门) 打乱连续若干符号时, 交织把错误分散到不同码字,
  *  每码字仅 1 bit 翻转 → Hamming 可纠正.
  ******************************************************************************
  */
#ifndef __VOICE_PROTO_H
#define __VOICE_PROTO_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* ---- 采样/符号时序 ---- */
#define VP_SAMPLE_RATE     16000U
#define VP_TONE_SAMPLES     320U   /* 20ms 载波 */
#define VP_GUARD_SAMPLES    160U   /* 10ms 保护 */
#define VP_SLOT_SAMPLES     480U   /* 30ms 符号槽 */

/* ---- 前导/同步/结束 ---- */
#define VP_PREAMBLE_MS      200U
#define VP_PILOT_PERIOD_MS   40U   /* 前导交替周期 */
#define VP_POSTAMBLE_MS     120U
#define VP_PILOT_LO           0U   /* 1500Hz */
#define VP_PILOT_HI           3U   /* 2400Hz */
#define VP_SYNC_DIGIT         1U   /* 1800Hz 同步音 (居中, 与两个导频都不同) */

/* ---- 载荷上限 ---- */
#define VP_MAX_CHARS         48U   /* 任务书: 单条消息 ≤48 字符 */
#define VP_HEADER_BYTES       3U   /* source_id, mask_lo, mask_hi */
#define VP_LEN_BYTES          1U   /* payload 字节数 (header+text) */
#define VP_CRC_BYTES          1U
/* payload = header + text; text ≤ 48 → payload ≤ 51 字节 */
#define VP_MAX_PAYLOAD_BYTES (VP_HEADER_BYTES + VP_MAX_CHARS)   /* 51 */

/* ---- FEC 尺寸 ----
 * 每字节 → 2 个 Hamming(7,4) 码字 → 14 bit → 7 个 4-FSK 符号 (2bit/符号).
 * 受 FEC 保护的字节 = LEN(1) + payload(≤51) + CRC(1) = ≤53 字节.
 */
#define VP_CODED_MAX_BYTES  (VP_LEN_BYTES + VP_MAX_PAYLOAD_BYTES + VP_CRC_BYTES) /* 53 */
#define VP_SYMS_PER_BYTE      7U    /* 14 coded bits / 2 bits-per-symbol */
/* LEN 三重冗余(+14 符号) + (payload+CRC) 交织. 最坏 48 字符:
 *   LEN 21 + (51+1)*7 = 21 + 364 = 385 符号 → 11.55s 数据. */
#define VP_MAX_DATA_SYMBOLS (14U + VP_CODED_MAX_BYTES * VP_SYMS_PER_BYTE)  /* 385 */

/* 20s 预算校验: 200ms 前导 + 30ms 同步 + 数据×30ms + 30ms 隔离 + 120ms 结束.
 * 最坏 48 字符: coded=53B → 371 符号 → 11.13s 数据 + 0.38s ≈ 11.5s < 20s. OK.
 * 典型 10 字符: coded=(1+3+10+1)=15B → 105 符号 → 3.15s + 0.35s ≈ 3.5s. */

/* ---- 4-FSK 频率 (Hz), digit 0..3 ---- */
#define VP_F0 1500
#define VP_F1 1800
#define VP_F2 2100
#define VP_F3 2400

#ifdef __cplusplus
}
#endif
#endif /* __VOICE_PROTO_H */
