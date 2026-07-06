#ifndef __FSK4_ENCODER_H
#define __FSK4_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FSK4_CHARSET_SIZE    75
#define FSK4_SYMBOLS_PER_CHAR 4
#define FSK4_FREQ_COUNT       4
#define FSK4_MAX_CHARS       48

#define FSK4_F_START         1500
#define FSK4_F_SPACING        500
#define FSK4_F_END           3000

#define FSK4_DDS_SAMPLE_RATE 16000
#define FSK4_DDS_N_BITS      32
#define FSK4_SINE_LUT_SIZE   1024

#define FSK4_T_SYMBOL        20
#define FSK4_T_GUARD         10
#define FSK4_T_PER_SYMBOL    (FSK4_T_SYMBOL + FSK4_T_GUARD)

#define FSK4_T_PREAMBLE      200
#define FSK4_T_POSTAMBLE     200

#define FSK4_PILOT_LO        0
#define FSK4_PILOT_HI        3
#define FSK4_PILOT_PERIOD    40

#define FSK4_CHECKSUM_SYMBOLS 4

typedef struct {
    uint8_t symbols[FSK4_MAX_CHARS * FSK4_SYMBOLS_PER_CHAR + FSK4_CHECKSUM_SYMBOLS];
    uint16_t symbol_count;
    uint32_t phase_inc[FSK4_FREQ_COUNT];
    const uint16_t *sine_lut;
    uint32_t phase_acc;
    uint8_t  current_symbol;
    uint16_t symbol_timer_ms;
    uint8_t  checksum;
    uint16_t total_symbols;
    uint16_t total_time_ms;
} FSK4_Encoder;

void      FSK4_Init(FSK4_Encoder *enc, const uint16_t *sine_lut);
uint16_t  FSK4_Encode(FSK4_Encoder *enc, const char *text);
uint32_t  FSK4_GetPhaseInc(uint8_t digit);
uint16_t  FSK4_GetFrequency(uint8_t digit);
uint16_t  FSK4_EstimateTime(uint16_t symbol_count);
uint8_t   FSK4_CharToIndex(char ch);
char      FSK4_IndexToChar(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __FSK4_ENCODER_H */
