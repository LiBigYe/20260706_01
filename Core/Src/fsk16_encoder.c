/**
  ******************************************************************************
  * @file           : fsk16_encoder.c
  * @brief          : 16-FSK 编码器实现 — 声语信使项目 (v4, 预留未使用)
  ******************************************************************************
  */

#include "fsk16_encoder.h"
#include <string.h>

uint8_t FSK16_CharToIndex(char ch)
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

char FSK16_IndexToChar(uint8_t idx)
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

static const uint16_t freq_table[FSK16_FREQ_COUNT] = {
    1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400,
    2600, 2800, 3000, 3200, 3400, 3600, 3800, 4000
};

static const uint32_t phase_inc_table[FSK16_FREQ_COUNT] = {
    268435456U, 322122547U, 375809638U, 429496730U,
    483183821U, 536870912U, 590558003U, 644245094U,
    697932186U, 751619277U, 805306368U, 858993459U,
    912680550U, 966367642U, 1020054733U, 1073741824U
};

uint16_t FSK16_GetFrequency(uint8_t hex_digit)
{
    if (hex_digit >= FSK16_FREQ_COUNT) return 0;
    return freq_table[hex_digit];
}

uint32_t FSK16_GetPhaseInc(uint8_t hex_digit)
{
    if (hex_digit >= FSK16_FREQ_COUNT) return 0;
    return phase_inc_table[hex_digit];
}

void FSK16_Init(FSK16_Encoder *enc, const uint16_t *sine_lut)
{
    memset(enc, 0, sizeof(FSK16_Encoder));
    for (uint8_t i = 0; i < FSK16_FREQ_COUNT; i++) {
        uint32_t calc = (uint32_t)(((uint64_t)freq_table[i] << FSK16_DDS_N_BITS)
                                    / FSK16_DDS_SAMPLE_RATE);
        enc->phase_inc[i] = calc;
    }
    enc->sine_lut      = sine_lut;
    enc->phase_acc     = 0;
    enc->current_symbol = 0;
    enc->symbol_timer_ms = 0;
    enc->checksum       = 0;
    enc->symbol_count   = 0;
}

uint16_t FSK16_Encode(FSK16_Encoder *enc, const char *text)
{
    enc->symbol_count = 0;
    enc->checksum     = 0;
    uint16_t len = 0;
    while (*text && len < FSK16_MAX_CHARS) {
        uint8_t idx = FSK16_CharToIndex(*text);
        if (idx < FSK16_CHARSET_SIZE) {
            enc->symbols[enc->symbol_count++] = idx >> 4;
            enc->symbols[enc->symbol_count++] = idx & 0x0F;
            enc->checksum ^= idx;
            len++;
        }
        text++;
    }
    enc->symbols[enc->symbol_count++] = enc->checksum >> 4;
    enc->symbols[enc->symbol_count++] = enc->checksum & 0x0F;
    enc->total_symbols = enc->symbol_count;
    enc->total_time_ms = FSK16_EstimateTime(enc->total_symbols);
    return enc->symbol_count;
}

uint16_t FSK16_EstimateTime(uint16_t symbol_count)
{
    uint32_t t = FSK16_T_PREAMBLE;
    t += (uint32_t)symbol_count * FSK16_T_PER_SYMBOL;
    t += FSK16_T_POSTAMBLE;
    return (uint16_t)t;
}
