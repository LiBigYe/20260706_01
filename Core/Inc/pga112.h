#ifndef __PGA112_H
#define __PGA112_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include "stm32f4xx_hal.h"

#define PGA_CMD_WRITE      0x2AU
#define PGA_CMD_SDN_DIS    0xC2U
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
#define PGA_GAIN_INIT_CODE PGA_GAIN_16X
#define PGA_GAIN_INVALID   0xFFU

/* All AGC thresholds are ADC counts, so the control law has one scale. */
#define AGC_CLIP_LOW          48U
#define AGC_CLIP_HIGH       4047U
#define AGC_CLIP_SAMPLES       3U
#define AGC_VPP_LOW_COUNTS  1120U  /* About 0.9 Vpp at 3.3 V VDDA. */
#define AGC_VPP_HIGH_COUNTS 2234U  /* About 1.8 Vpp at 3.3 V VDDA. */
#define AGC_HOLD_BLOCKS        4U  /* Four 5 ms blocks. */
#define AGC_ACQUIRE_MAX_CODE PGA_GAIN_64X

#define PGA112_AGC_DISABLED 0U
#define PGA112_AGC_ACQUIRE  1U
#define PGA112_AGC_LOCKED   2U

HAL_StatusTypeDef PGA112_Init(void);
HAL_StatusTypeDef PGA112_SetGain(uint8_t gain_code);
uint8_t PGA112_RequestGain(uint8_t gain_code);
void    PGA112_Service(void);
void    PGA112_CancelPending(void);
uint8_t PGA112_IsPending(void);
uint8_t PGA112_GetGain(void);
extern volatile uint8_t g_pga_gain_live;
HAL_StatusTypeDef PGA112_GetLastStatus(void);
uint32_t PGA112_GetErrorCount(void);
uint16_t PGA112_GetLastVpp(void);
uint8_t PGA112_AGC_Update(const uint16_t *samples, uint16_t len, uint8_t mode);
void    PGA112_AGC_Reset(void);

#ifdef __cplusplus
}
#endif
#endif
