/**
  ******************************************************************************
  * @file           : fsk4_encoder.c
  * @brief          : 4-FSK 编码器实现 — 声语信使项目 (v4)
  *
  *   字符编码: 75 个字符 → 索引 0~74 → 4 个 base-4 digit → 频率序列
  ******************************************************************************
  */

#include "fsk4_encoder.h"
#include <string.h>

/* ========================================================================== */
/*  字符集映射表                                                               */
/* ========================================================================== */

uint8_t FSK4_CharToIndex(char ch)
{
    if (ch >= 'a' && ch <= 'z') return (uint8_t)(ch - 'a');
    if (ch >= 'A' && ch <= 'Z') return (uint8_t)(ch - 'A' + 26);
    if (ch >= '0' && ch <= '9') return (uint8_t)(ch - '0' + 52);
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
    return 255;
}

char FSK4_IndexToChar(uint8_t idx)
{
    if (idx <= 25)      return (char)('a' + idx);
    if (idx <= 51)      return (char)('A' + idx - 26);
    if (idx <= 61)      return (char)('0' + idx - 52);
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
    return '?';
}

static const uint16_t freq_table[FSK4_FREQ_COUNT] = {
    1500, 2000, 2500, 3000
};

static const uint32_t phase_inc_table[FSK4_FREQ_COUNT] = {
    402653184U, 536870912U, 671088640U, 805306368U
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

void FSK4_Init(FSK4_Encoder *enc, const uint16_t *sine_lut)
{
    memset(enc, 0, sizeof(FSK4_Encoder));
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

uint16_t FSK4_Encode(FSK4_Encoder *enc, const char *text)
{
    enc->symbol_count = 0;
    enc->checksum     = 0;
    uint16_t len = 0;

    while (*text && len < FSK4_MAX_CHARS) {
        uint8_t idx = FSK4_CharToIndex(*text);
        if (idx < FSK4_CHARSET_SIZE) {
            enc->symbols[enc->symbol_count++] = (idx >> 6) & 0x03;
            enc->symbols[enc->symbol_count++] = (idx >> 4) & 0x03;
            enc->symbols[enc->symbol_count++] = (idx >> 2) & 0x03;
            enc->symbols[enc->symbol_count++] =  idx       & 0x03;
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
        enc->symbols[enc->symbol_count++] =  term_idx       & 0x03;
        enc->checksum ^= term_idx;
        len++;
    }

    while (len < FSK4_MAX_CHARS) {
        uint8_t pad_idx = 65;
        enc->symbols[enc->symbol_count++] = (pad_idx >> 6) & 0x03;
        enc->symbols[enc->symbol_count++] = (pad_idx >> 4) & 0x03;
        enc->symbols[enc->symbol_count++] = (pad_idx >> 2) & 0x03;
        enc->symbols[enc->symbol_count++] =  pad_idx       & 0x03;
        enc->checksum ^= pad_idx;
        len++;
    }

    enc->symbols[enc->symbol_count++] = (enc->checksum >> 6) & 0x03;
    enc->symbols[enc->symbol_count++] = (enc->checksum >> 4) & 0x03;
    enc->symbols[enc->symbol_count++] = (enc->checksum >> 2) & 0x03;
    enc->symbols[enc->symbol_count++] =  enc->checksum       & 0x03;

    enc->total_symbols = enc->symbol_count;
    enc->total_time_ms = FSK4_EstimateTime(enc->total_symbols);
    return enc->symbol_count;
}

uint16_t FSK4_EstimateTime(uint16_t symbol_count)
{
    uint32_t t = FSK4_T_PREAMBLE;
    t += (uint32_t)symbol_count * FSK4_T_PER_SYMBOL;
    t += FSK4_T_POSTAMBLE;
    return (uint16_t)t;
}
