#ifndef __PGA112_H
#define __PGA112_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define PGA_CMD_WRITE      0x2AU
#define PGA_CMD_SDN_DIS    0xE1U
#define PGA_CHANNEL        0x00U

#define PGA_GAIN_1X        0U
#define PGA_GAIN_2X        1U
#define PGA_GAIN_4X        2U
#define PGA_GAIN_8X        3U
#define PGA_GAIN_16X       4U
#define PGA_GAIN_32X       5U
#define PGA_GAIN_64X       6U
#define PGA_GAIN_128X      7U

#define PGA_GAIN_MIN_CODE  0U
#define PGA_GAIN_MAX_CODE  7U
#define PGA_GAIN_INIT_CODE PGA_GAIN_32X

#define AGC_RMS_HIGH         550U
#define AGC_RMS_LOW          250U
#define AGC_CLIP_LOW          48U
#define AGC_CLIP_HIGH       4047U
#define AGC_HOLD_BLOCKS        4U

#define AGC_VPP_LOW         1500U
#define AGC_VPP_HIGH        2800U
#define AGC_VPP_MAX         3300U

void    PGA112_Init(void);
void    PGA112_SetGain(uint8_t gain_code);
uint8_t PGA112_GetGain(void);
extern volatile uint8_t g_pga_gain_live;
uint8_t PGA112_AGC_Update(const uint16_t *samples, uint16_t len, uint8_t frame_active);
void    PGA112_AGC_Reset(void);

#ifdef __cplusplus
}
#endif
#endif
