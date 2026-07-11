/**
  ******************************************************************************
  * @file           : fsk4_encoder.h
  * @brief          : 4-FSK 编码器 — 声语信使项目 (v4)
  *
  *   字符集 (74 个, 含终止符 + 标点):
  *     abcdefghijklmnopqrstuvwxyz  (0~25)
  *     ABCDEFGHIJKLMNOPQRSTUVWXYZ  (26~51)
  *     0123456789                   (52~61)
  *     .?!                          (62~64)
  *     空格                          (65)
 *     $                             (66, 内部终止符)
 *     ( )                           (67~68)
 *     + - * / =                     (69~73)
  *
  *   编码方式:
  *     字符 → 0~73 索引 → 4 位 4 进制 (每数字 2 bit)
  *     每 4 进制 digit → 1 个 FSK 符号 → 对应 4 个频率之一
  *     每字符共 4 个符号 (4^4 = 256 >= 74)
  *
  *   4 个频率: 1500 Hz ~ 2400 Hz, 步进 300 Hz
  *     Digit 0 → 1500 Hz
  *     Digit 1 → 1800 Hz
  *     Digit 2 → 2100 Hz
  *     Digit 3 → 2400 Hz
  *
  *   符号时序 (收发端共享):
  *     符号持续   T_SYMBOL = 20 ms
  *     符号间保护  T_GUARD  = 10 ms
  *     每个符号总计 30 ms
  *
  *   v4 同步设计: 10ms guard 输出 1.65V DC, 形成交流能量真空期.
  *   接收端 v4 DPLL 利用下降沿 [HI,HI,LO] 实现物理层符号定时恢复.
  *
  *   传输帧:
  *     [前导 200ms] [12 地址符号] [192 正文符号] [4 校验符号] [结束 200ms]
  *     总时长 ≈ 200 + 196×30 + 200 = 6280 ms ≈ 6.28 秒
  ******************************************************************************
  */

#ifndef __FSK4_ENCODER_H
#define __FSK4_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "network_protocol.h"

/* ========================================================================== */
/*  常量定义                                                                   */
/* ========================================================================== */

#define FSK4_CHARSET_SIZE    75    /* 字符集大小 (含 '$' 终止符 + \n) */
#define FSK4_SYMBOLS_PER_CHAR 4    /* 每字符 → 4 个 4 进制符号 (base-4) */
#define FSK4_FREQ_COUNT       4    /* 4 个频率 (0 ~ 3) */
#define FSK4_MAX_CHARS       48    /* 最大字符数 */

/* 频率定义 (Hz) */
#define FSK4_F_START         1500  /* Digit 0 = 1500 Hz */
#define FSK4_F_SPACING        300  /* 每个 digit +1 = +300 Hz */
#define FSK4_F_END           2400  /* Digit 3 = 2400 Hz */

/* DDS 参数 */
#define FSK4_DDS_SAMPLE_RATE 16000 /* 采样率 Hz (TIM3 溢出频率) */
#define FSK4_DDS_N_BITS      32    /* 相位累加器位数 */
#define FSK4_SINE_LUT_SIZE   1024  /* 正弦查找表大小 (10-bit 相位) */

/* 符号时序 (ms).
 * v4 fix: guard 10ms 确保接收端 5ms 切片总能捕获至少 1 个纯净 LO 块,
 * 消除异步时钟下 5ms guard 跨越两个接收切片造成的"时间涂抹". */
#define FSK4_T_SYMBOL        20    /* 单符号载波持续 */
#define FSK4_T_GUARD         10    /* 符号间保护间隔 (加宽防涂抹) */
#define FSK4_T_PER_SYMBOL    (FSK4_T_SYMBOL + FSK4_T_GUARD)  /* 30ms */

/* 帧时序 (ms) */
#define FSK4_T_PREAMBLE      200   /* 前导码时长 */
#define FSK4_T_POSTAMBLE     200   /* 结束标志时长 */

/* 导频 — 前导码用最低/最高频率交替 */
#define FSK4_PILOT_LO        0     /* Digit 0 = 1500 Hz */
#define FSK4_PILOT_HI        3     /* Digit 3 = 2400 Hz */
#define FSK4_PILOT_PERIOD    40    /* 每 40ms 交替一次 */

/* 校验符号数 (8-bit XOR → 4 个 base-4 符号) */
#define FSK4_CHECKSUM_SYMBOLS 4

/* ========================================================================== */
/*  数据结构                                                                   */
/* ========================================================================== */

/**
  * @brief FSK4 编码器完整状态
  */
typedef struct {
    /* ── 待发送的符号序列 ── */
    uint8_t symbols[NET_HEADER_SYMBOLS + FSK4_MAX_CHARS * FSK4_SYMBOLS_PER_CHAR + FSK4_CHECKSUM_SYMBOLS];
    uint16_t symbol_count;          /* 实际符号数 (含校验) */

    /* ── 频率索引 (0~3) → 相位增量 ── */
    uint32_t phase_inc[FSK4_FREQ_COUNT];  /* DDS phase increment per freq */

    /* ── 波形生成参数 ── */
    const uint16_t *sine_lut;       /* 指向正弦查找表 */
    uint32_t phase_acc;             /* DDS 相位累加器 (运行时更新) */
    uint8_t  current_symbol;        /* 当前发送符号索引 */
    uint16_t symbol_timer_ms;       /* 当前符号已发送时长 */

    /* ── 校验 ── */
    uint8_t  checksum;              /* XOR 校验和 */

    /* ── 统计 ── */
    uint16_t total_symbols;         /* 总符号数（含数据 + 校验） */
    uint16_t total_time_ms;         /* 预估总传输时间 */
} FSK4_Encoder;

/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

/**
  * @brief  初始化编码器状态
  * @param  enc:       编码器状态指针
  * @param  sine_lut:  正弦查找表 (1024 点 × uint16_t, 10-bit 量化)
  */
void FSK4_Init(FSK4_Encoder *enc, const uint16_t *sine_lut);

/**
  * @brief  将文本缓冲区编码为 FSK 符号序列
  * @param  enc:  编码器状态指针
  * @param  text: 输入文本 (以 '\0' 结尾, 最大 48 字符)
  * @retval 返回实际编码的符号总数 (含校验)
  *
  *  自动计算 XOR 校验和并追加 4 个校验符号。
  *  每个字符 → 索引 0~73 → 4 个 base-4 digit → 4 个符号。
  */
uint16_t FSK4_Encode(FSK4_Encoder *enc, const char *text,
                     uint8_t source_id, uint16_t target_mask);

/**
  * @brief  根据频率索引获取 DDS 相位增量
  * @param  digit: 0~3 (对应频率 1500~2400 Hz, 步进 300 Hz)
  * @retval 32 位相位增量值
  */
uint32_t FSK4_GetPhaseInc(uint8_t digit);

/**
  * @brief  根据频率索引获取对应的实际频率值 (Hz)
  * @param  digit: 0~3
  * @retval 频率 (Hz)
  */
uint16_t FSK4_GetFrequency(uint8_t digit);

/**
  * @brief  预估传输时间
  * @param  symbol_count: 符号总数 (含数据 + 校验)
  * @retval 预估总时长 (ms)，含前导 + 符号 + 保护间隔 + 结束标志
  */
uint16_t FSK4_EstimateTime(uint16_t symbol_count);

/**
  * @brief  查找字符在字符集中的索引 (0~73)
  * @param  ch: ASCII 字符
  * @retval 0~73 (有效字符), 255 (不在字符集中)
  */
uint8_t FSK4_CharToIndex(char ch);

/**
  * @brief  索引转回 ASCII 字符 (接收端解码用)
  * @param  idx: 0~73
  * @retval 对应的 ASCII 字符, '?' 表示无效索引
  */
char FSK4_IndexToChar(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __FSK4_ENCODER_H */
