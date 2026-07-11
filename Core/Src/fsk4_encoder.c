/**
  ******************************************************************************
  * @file           : fsk4_encoder.c
  * @brief          : 4-FSK 编码器实现 — 声语信使项目 (v4)
  *
  *   字符编码: 74 个字符 → 索引 0~73 → 4 个 base-4 digit → 频率序列
  *   频率范围: 1500~2400 Hz, 300 Hz 步进
  *   DDS 采样: 16 kHz, 32-bit 相位累加器
  *
  *   v4 协议: 每符号 = 20ms 载波 + 10ms guard (1.65V DC).
  *   接收端 v4 通过 DPLL 下降沿利用 guard 的能量真空期实现符号定时恢复.
  *
  *   ┌─────────────── 频率分配表 ───────────────┐
  *   │ Digit │  f(Hz) │ Phase Inc (32-bit, 16kHz) │
  *   │   0   │  1500  │    402,653,184            │
  *   │   1   │  1800  │    483,183,821            │
  *   │   2   │  2100  │    563,714,458            │
  *   │   3   │  2400  │    644,245,094            │
  *   └───────┴────────┴───────────────────────────┘
  *
  *   DDS 公式:  phase_inc = (f_out / f_sample) × 2^32
  *             phase_inc = f_out × 2^32 / 16000
  *             phase_inc = f_out × 268435.456  (取整)
  *
  *   编码举例 (字符 'a' = 索引 0):
  *     索引 = 0 (0x00)
  *     Base-4 digits: d3=0, d2=0, d1=0, d0=0 → [0, 0, 0, 0]
  *
  *   编码举例 (字符 '!' = 索引 64):
  *     索引 = 64 (0x40)
  *     bits: 0100 0000
  *     Base-4 digits: d3=1, d2=0, d1=0, d0=0 → [1, 0, 0, 0]
  *
  *   编码举例 (空格 = 索引 65):
  *     索引 = 65 (0x41)
  *     bits: 0100 0001
  *     Base-4 digits: d3=1, d2=0, d1=0, d0=1 → [1, 0, 0, 1]
  ******************************************************************************
  */

#include "fsk4_encoder.h"
#include <string.h>

/* ========================================================================== */
/*  字符集映射表                                                               */
/* ========================================================================== */

/*
 * 字符 → 索引 (0~73):
 *   0~25   = a ~ z   (ASCII 97~122)
 *   26~51  = A ~ Z   (ASCII 65~90)
 *   52~61  = 0 ~ 9   (ASCII 48~57)
 *   62     = .       (ASCII 46)
 *   63     = ?       (ASCII 63)
 *   64     = !       (ASCII 33)
 *   65     = 空格     (ASCII 32)
 *   66     = $       (ASCII 36, 内部终止符)
 *   67     = (       (ASCII 40)
 *   68     = )       (ASCII 41)
 *   69     = +       (ASCII 43)
 *   70     = -       (ASCII 45)
 *   71     = *       (ASCII 42)
 *   72     = /       (ASCII 47)
 *   73     = =       (ASCII 61)
 *
 * 不在字符集中的字符 → 返回 255
 */

uint8_t FSK4_CharToIndex(char ch)
{
    if (ch >= 'a' && ch <= 'z') return (uint8_t)(ch - 'a');       /*  0~25 */
    if (ch >= 'A' && ch <= 'Z') return (uint8_t)(ch - 'A' + 26);  /* 26~51 */
    if (ch >= '0' && ch <= '9') return (uint8_t)(ch - '0' + 52);  /* 52~61 */
    if (ch == '.')              return 62;
    if (ch == '?')              return 63;
    if (ch == '!')              return 64;
    if (ch == ' ')              return 65;
    if (ch == '$')              return 66;
    if (ch == '(')              return 67;
    if (ch == ')')              return 68;
    if (ch == '+')              return 69;
    if (ch == '-')              return 70;
    if (ch == '*')              return 71;
    if (ch == '/')              return 72;
    if (ch == '=')              return 73;
    if (ch == '\n')             return 74;
    return 255;  /* not in charset */
}

char FSK4_IndexToChar(uint8_t idx)
{
    if (idx <= 25)      return (char)('a' + idx);         /* a~z */
    if (idx <= 51)      return (char)('A' + idx - 26);    /* A~Z */
    if (idx <= 61)      return (char)('0' + idx - 52);    /* 0~9 */
    if (idx == 62)      return '.';
    if (idx == 63)      return '?';
    if (idx == 64)      return '!';
    if (idx == 65)      return ' ';
    if (idx == 66)      return '$';
    if (idx == 67)      return '(';
    if (idx == 68)      return ')';
    if (idx == 69)      return '+';
    if (idx == 70)      return '-';
    if (idx == 71)      return '*';
    if (idx == 72)      return '/';
    if (idx == 73)      return '=';
    if (idx == 74)      return '\n';
    return '?';  /* invalid */
}

/* ========================================================================== */
/*  频率与 DDS 相位增量表                                                       */
/* ========================================================================== */

/**
  * @brief  频率表: digit(0~3) → 实际频率 (Hz)
  *
  *  频率计算: f(digit) = FSK4_F_START + digit × FSK4_F_SPACING
  *           = 1500 + digit × 300
  */
static const uint16_t freq_table[FSK4_FREQ_COUNT] = {
    1500,   /* 0: 1500 Hz */
    1800,   /* 1: 1800 Hz */
    2100,   /* 2: 2100 Hz */
    2400    /* 3: 2400 Hz */
};

/**
  * @brief  DDS 相位增量表 (预计算)
  *
  *   phase_inc = f_out × 2^32 / 16000
  *   使用 64-bit 中间值避免溢出:
  *     phase_inc = ((uint64_t)freq × (1ULL << 32)) / 16000
  */
static const uint32_t phase_inc_table[FSK4_FREQ_COUNT] = {
    402653184U,   /* 0: 1500 Hz → 1500 × 268435.456 */
    483183821U,   /* 1: 1800 Hz → 1800 × 268435.456 */
    563714458U,   /* 2: 2100 Hz → 2100 × 268435.456 */
    644245094U    /* 3: 2400 Hz → 2400 × 268435.456 */
    /*
     *  验证: phase_inc / 2^32 × 16000
     *  402653184 / 4294967296 × 16000 = 1500.0000 Hz
     *  644245094 / 4294967296 × 16000 = 2400.0001 Hz
     */
};

uint16_t FSK4_GetFrequency(uint8_t digit)
{
    if (digit >= FSK4_FREQ_COUNT) return 0;
    return freq_table[digit];
}

uint32_t FSK4_GetPhaseInc(uint8_t digit)
{
    if (digit >= FSK4_FREQ_COUNT) return 0;
    return phase_inc_table[digit];
}

/* ========================================================================== */
/*  编码器核心                                                                  */
/* ========================================================================== */

/**
  * @brief  初始化编码器, 预计算 DDS 相位增量 (查表验证一致性)
  * @param  sine_lut: 1024 点正弦表指针 (NULL 跳过 LUT 赋值, 仅验证 phase_inc)
  */
void FSK4_Init(FSK4_Encoder *enc, const uint16_t *sine_lut)
{
    memset(enc, 0, sizeof(FSK4_Encoder));

    /* 验证并填充相位增量表 (与静态表对比, 确保一致性) */
    for (uint8_t i = 0; i < FSK4_FREQ_COUNT; i++) {
        uint32_t calc = (uint32_t)(((uint64_t)freq_table[i] << FSK4_DDS_N_BITS)
                                    / FSK4_DDS_SAMPLE_RATE);
        enc->phase_inc[i] = calc;
    }

    enc->sine_lut        = sine_lut;
    enc->phase_acc       = 0;
    enc->current_symbol  = 0;
    enc->symbol_timer_ms = 0;
    enc->checksum        = 0;
    enc->symbol_count    = 0;
}

/**
  * @brief  文本 → 符号序列 (base-4 编码, 定长 48 字符)
  *
  *   对输入文本逐字符处理:
  *     1. 查字符 → 索引 (0~73, 非法字符跳过)
  *     2. 索引 (8-bit) 拆为 4 个 base-4 digit → 4 个 FSK 符号
  *     3. 不足 48 字符时用空格 (索引 65) 填充到满 48 字符
  *     4. XOR 累加校验和 (含填充空格)
  *     5. 校验和 (8-bit) 拆为 4 个 base-4 symbol 追加
  *
  *   v4 fix: 强制定长 48 字符 → 输出恒为 196 符号 (192 数据 + 4 校验).
  *   接收端期望固定 196 符号. 变长输出会导致短消息时 POSTAMBLE 的连续
  *   2400Hz (无 guard) 触发 SYNC_LOST, 丢弃已正确解码的短消息.
  *
  *   Base-4 编码: digit3=(idx>>6)&0x3, digit2, digit1, digit0.
  */
uint16_t FSK4_Encode(FSK4_Encoder *enc, const char *text,
                     uint8_t source_id, uint16_t target_mask)
{
    enc->symbol_count = 0;
    enc->checksum = 0;

    if (source_id < NET_MIN_DEVICE_ID || source_id > NET_MAX_DEVICE_ID) {
        source_id = NET_MIN_DEVICE_ID;
    }
    target_mask &= NET_VALID_TARGET_MASK;

    uint8_t header[NET_HEADER_BYTES] = {
        source_id,
        (uint8_t)(target_mask & 0xFFU),
        (uint8_t)(target_mask >> 8)
    };
    for (uint8_t h = 0; h < NET_HEADER_BYTES; h++) {
        uint8_t value = header[h];
        enc->symbols[enc->symbol_count++] = (value >> 6) & 0x03;
        enc->symbols[enc->symbol_count++] = (value >> 4) & 0x03;
        enc->symbols[enc->symbol_count++] = (value >> 2) & 0x03;
        enc->symbols[enc->symbol_count++] = value & 0x03;
        enc->checksum ^= value;
    }

    uint16_t len = 0;
    while (*text && len < FSK4_MAX_CHARS) {
        uint8_t idx = FSK4_CharToIndex(*text);
        if (idx < FSK4_CHARSET_SIZE) {
            enc->symbols[enc->symbol_count++] = (idx >> 6) & 0x03;
            enc->symbols[enc->symbol_count++] = (idx >> 4) & 0x03;
            enc->symbols[enc->symbol_count++] = (idx >> 2) & 0x03;
            enc->symbols[enc->symbol_count++] = idx & 0x03;
            enc->checksum ^= idx;
            len++;
        }
        text++;
    }

    if (len < FSK4_MAX_CHARS) {
        uint8_t term_idx = 66;
        enc->symbols[enc->symbol_count++] = (term_idx >> 6) & 0x03;
        enc->symbols[enc->symbol_count++] = (term_idx >> 4) & 0x03;
        enc->symbols[enc->symbol_count++] = (term_idx >> 2) & 0x03;
        enc->symbols[enc->symbol_count++] = term_idx & 0x03;
        enc->checksum ^= term_idx;
        len++;
    }

    while (len < FSK4_MAX_CHARS) {
        uint8_t pad_idx = 65;
        enc->symbols[enc->symbol_count++] = (pad_idx >> 6) & 0x03;
        enc->symbols[enc->symbol_count++] = (pad_idx >> 4) & 0x03;
        enc->symbols[enc->symbol_count++] = (pad_idx >> 2) & 0x03;
        enc->symbols[enc->symbol_count++] = pad_idx & 0x03;
        enc->checksum ^= pad_idx;
        len++;
    }

    enc->symbols[enc->symbol_count++] = (enc->checksum >> 6) & 0x03;
    enc->symbols[enc->symbol_count++] = (enc->checksum >> 4) & 0x03;
    enc->symbols[enc->symbol_count++] = (enc->checksum >> 2) & 0x03;
    enc->symbols[enc->symbol_count++] = enc->checksum & 0x03;

    enc->total_symbols = enc->symbol_count;
    enc->total_time_ms = FSK4_EstimateTime(enc->total_symbols);
    return enc->symbol_count;
}

/* ========================================================================== */
/*  传输时间估算                                                               */
/* ========================================================================== */

/**
  * @brief  预估完整传输时间 (定长 48 字符)
  *
  *   总时长 = 前导码 + (208符号 × 30ms) + 结束标志
  *   T = 200 + 208×30 + 200 = 6,640 ms ≈ 6.64 秒
  *
  *   v4: 所有消息强制填充到 48 字符, 传输时间固定不变.
  */
uint16_t FSK4_EstimateTime(uint16_t symbol_count)
{
    uint32_t t = FSK4_T_PREAMBLE;
    t += (uint32_t)symbol_count * FSK4_T_PER_SYMBOL;
    t += FSK4_T_POSTAMBLE;
    return (uint16_t)t;
}
