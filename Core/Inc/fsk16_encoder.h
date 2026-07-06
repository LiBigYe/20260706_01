#ifndef __FSK16_ENCODER_H
#define __FSK16_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FSK16_CHARSET_SIZE    75
#define FSK16_HEX_PER_CHAR    2
#define FSK16_FREQ_COUNT      16
#define FSK16_MAX_CHARS       48

#define FSK16_F_START         1000
#define FSK16_F_SPACING       200
#define FSK16_F_END           4000

#define FSK16_DDS_SAMPLE_RATE 16000
#define FSK16_DDS_N_BITS      32
#define FSK16_SINE_LUT_SIZE   256

#define FSK16_T_SYMBOL        20
#define FSK16_T_GUARD         10
#define FSK16_T_PER_SYMBOL    (FSK16_T_SYMBOL + FSK16_T_GUARD)

#define FSK16_T_PREAMBLE      200
#define FSK16_T_POSTAMBLE     200

#define FSK16_PILOT_LO        0
#define FSK16_PILOT_HI        15
#define FSK16_PILOT_PERIOD    40

typedef struct {
    uint8_t symbols[FSK16_MAX_CHARS * FSK16_HEX_PER_CHAR + 4];
    uint16_t symbol_count;
    uint32_t phase_inc[FSK16_FREQ_COUNT];
    const uint16_t *sine_lut;
    uint32_t phase_acc;
    uint8_t  current_symbol;
    uint16_t symbol_timer_ms;
    uint8_t  checksum;
    uint16_t total_symbols;
    uint16_t total_time_ms;
} FSK16_Encoder;

void      FSK16_Init(FSK16_Encoder *enc, const uint16_t *sine_lut);
uint16_t  FSK16_Encode(FSK16_Encoder *enc, const char *text);
uint32_t  FSK16_GetPhaseInc(uint8_t hex_digit);
uint16_t  FSK16_GetFrequency(uint8_t hex_digit);
uint16_t  FSK16_EstimateTime(uint16_t symbol_count);
uint8_t   FSK16_CharToIndex(char ch);
char      FSK16_IndexToChar(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __FSK16_ENCODER_H */
