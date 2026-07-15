#include "pga112.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

volatile uint8_t  g_pga_gain_live = PGA_GAIN_INIT_CODE;
static uint16_t agc_hi_hold = 0U;
static uint16_t agc_lo_hold = 0U;

static void pga_write2(uint8_t b0, uint8_t b1)
{
    uint8_t tx[2] = { b0, b1 };
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_RESET);
    (void)HAL_SPI_Transmit(&hspi2, tx, 2, 1U);
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_SET);
}

void PGA112_SetGain(uint8_t gain_code)
{
    if (gain_code > PGA_GAIN_MAX_CODE) gain_code = PGA_GAIN_MAX_CODE;
    g_pga_gain_live = gain_code;
    uint8_t data = (uint8_t)((gain_code << 4) | (PGA_CHANNEL & 0x0FU));
    pga_write2(PGA_CMD_WRITE, data);
}

uint8_t PGA112_GetGain(void) { return g_pga_gain_live; }

void PGA112_Init(void)
{
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_SET);
    pga_write2(PGA_CMD_SDN_DIS, 0x00U);
    PGA112_SetGain(PGA_GAIN_INIT_CODE);
    agc_hi_hold = 0U;
    agc_lo_hold = 0U;
}

void PGA112_AGC_Reset(void)
{
    agc_hi_hold = 0U;
    agc_lo_hold = 0U;
}

uint8_t PGA112_AGC_Update(const uint16_t *samples, uint16_t len, uint8_t frame_active)
{
    if (len == 0U) return 0U;

    uint8_t clipped = 0U;
    uint32_t sum = 0U;
    uint16_t vmin = 4095U, vmax = 0U;
    uint16_t i;
    for (i = 0; i < len; i++) {
        uint16_t s = samples[i];
        if (s <= AGC_CLIP_LOW || s >= AGC_CLIP_HIGH) clipped = 1U;
        sum += s;
        if (s < vmin) vmin = s;
        if (s > vmax) vmax = s;
    }
    uint16_t mean = (uint16_t)(sum / len);
    uint16_t vpp_raw = (uint16_t)(vmax - vmin);

    uint32_t sq_sum = 0U;
    for (i = 0; i < len; i++) {
        int32_t d = (int32_t)samples[i] - (int32_t)mean;
        sq_sum += (uint32_t)(d * d);
    }
    uint32_t rms2 = sq_sum / len;
    uint32_t rms2_high = (uint32_t)AGC_RMS_HIGH * AGC_RMS_HIGH;
    uint32_t rms2_low  = (uint32_t)AGC_RMS_LOW  * AGC_RMS_LOW;

    if (clipped) {
        agc_hi_hold = 0U; agc_lo_hold = 0U;
        if (g_pga_gain_live > PGA_GAIN_MIN_CODE) {
            PGA112_SetGain((uint8_t)(g_pga_gain_live - 1U));
            return 1U;
        }
        return 0U;
    }

    if (frame_active && vpp_raw > 0U) {
        uint32_t vpp_mv = (uint32_t)vpp_raw * 3300U / 4095U;
        if (vpp_mv < AGC_VPP_LOW) {
            agc_hi_hold = 0U;
            if (agc_lo_hold < 0xFFFFU) agc_lo_hold++;
            if (agc_lo_hold >= AGC_HOLD_BLOCKS) {
                agc_lo_hold = 0U;
                if (g_pga_gain_live < PGA_GAIN_MAX_CODE) {
                    PGA112_SetGain((uint8_t)(g_pga_gain_live + 1U));
                    return 1U;
                }
            }
            return 0U;
        }
    }

    if (rms2 >= rms2_high) {
        agc_lo_hold = 0U;
        if (agc_hi_hold < 0xFFFFU) agc_hi_hold++;
        if (agc_hi_hold >= AGC_HOLD_BLOCKS) {
            agc_hi_hold = 0U;
            if (g_pga_gain_live > PGA_GAIN_MIN_CODE) {
                PGA112_SetGain((uint8_t)(g_pga_gain_live - 1U));
                return 1U;
            }
        }
        return 0U;
    }

    if (rms2 <= rms2_low) {
        agc_hi_hold = 0U;
        if (agc_lo_hold < 0xFFFFU) agc_lo_hold++;
        if (agc_lo_hold >= AGC_HOLD_BLOCKS) {
            agc_lo_hold = 0U;
            if (g_pga_gain_live < PGA_GAIN_MAX_CODE) {
                PGA112_SetGain((uint8_t)(g_pga_gain_live + 1U));
                return 1U;
            }
        }
        return 0U;
    }

    agc_hi_hold = 0U;
    agc_lo_hold = 0U;
    return 0U;
}
