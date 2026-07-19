#include "pga112.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

volatile uint8_t  g_pga_gain_live = PGA_GAIN_INVALID;
static volatile uint8_t pga_gain_requested = PGA_GAIN_INIT_CODE;
static volatile uint8_t pga_write_pending = 0U;
static volatile uint8_t pga_updates_frozen = 0U;
static volatile uint16_t pga_last_vpp = 0U;
static volatile HAL_StatusTypeDef pga_last_status = HAL_OK;
static volatile uint32_t pga_error_count = 0U;
static uint32_t pga_next_retry_tick = 0U;
static uint8_t agc_up_hold = 0U;
static uint8_t agc_down_hold = 0U;

static HAL_StatusTypeDef pga_write2(uint8_t b0, uint8_t b1)
{
    uint8_t tx[2] = { b0, b1 };
    HAL_StatusTypeDef status;

    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_RESET);
    status = HAL_SPI_Transmit(&hspi2, tx, 2, 1U);
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_SET);

    return status;
}

static uint8_t pga_clamp_gain(uint8_t gain_code)
{
    if (gain_code > PGA_GAIN_MAX_CODE) gain_code = PGA_GAIN_MAX_CODE;
    return gain_code;
}

static HAL_StatusTypeDef pga_write_gain(uint8_t gain_code)
{
    uint8_t data = (uint8_t)((gain_code << 4) | (PGA_CHANNEL & 0x0FU));
    HAL_StatusTypeDef status = pga_write2(PGA_CMD_WRITE, data);

    pga_last_status = status;
    if (status == HAL_OK) {
        g_pga_gain_live = gain_code;
    } else {
        pga_error_count++;
    }
    return status;
}

HAL_StatusTypeDef PGA112_SetGain(uint8_t gain_code)
{
    gain_code = pga_clamp_gain(gain_code);
    pga_gain_requested = gain_code;
    HAL_StatusTypeDef status = pga_write_gain(gain_code);
    pga_write_pending = (status == HAL_OK) ? 0U : 1U;
    pga_next_retry_tick = (status == HAL_OK) ? 0U : (HAL_GetTick() + 10U);
    return status;
}

uint8_t PGA112_RequestGain(uint8_t gain_code)
{
    gain_code = pga_clamp_gain(gain_code);
    if (pga_updates_frozen != 0U) return 0U;
    if (pga_write_pending == 0U && g_pga_gain_live == gain_code) {
        return 0U;
    }

    if (pga_write_pending != 0U && pga_gain_requested == gain_code) {
        return 0U;
    }

    pga_gain_requested = gain_code;
    pga_write_pending = 1U;
    pga_next_retry_tick = 0U;
    return 1U;
}

void PGA112_Service(void)
{
    if (pga_write_pending == 0U) return;
    if ((int32_t)(HAL_GetTick() - pga_next_retry_tick) < 0) return;

    /* The DMA callback changes the lock when it enters DATA. Mask only that
     * callback while shifting two SPI bytes so an accepted write always
     * completes before the state transition that freezes the gain. */
    uint32_t dma_irq_was_enabled = NVIC_GetEnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    if (pga_updates_frozen != 0U || pga_write_pending == 0U) {
        if (dma_irq_was_enabled != 0U) HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
        return;
    }

    uint8_t gain_code = pga_gain_requested;
    HAL_StatusTypeDef status = pga_write_gain(gain_code);
    if (status == HAL_OK) {
        pga_next_retry_tick = 0U;
        pga_write_pending = 0U;
    } else {
        pga_next_retry_tick = HAL_GetTick() + 10U;
    }
    if (dma_irq_was_enabled != 0U) HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void PGA112_CancelPending(void)
{
    if (g_pga_gain_live <= PGA_GAIN_MAX_CODE) {
        pga_gain_requested = g_pga_gain_live;
        pga_write_pending = 0U;
        pga_next_retry_tick = 0U;
    }
}

void PGA112_SetUpdatesFrozen(uint8_t frozen)
{
    pga_updates_frozen = frozen ? 1U : 0U;
    if (pga_updates_frozen != 0U) PGA112_CancelPending();
}

uint8_t PGA112_GetGain(void) { return g_pga_gain_live; }
uint8_t PGA112_IsPending(void) { return pga_write_pending; }
HAL_StatusTypeDef PGA112_GetLastStatus(void) { return pga_last_status; }
uint32_t PGA112_GetErrorCount(void) { return pga_error_count; }
uint16_t PGA112_GetLastVpp(void) { return pga_last_vpp; }

HAL_StatusTypeDef PGA112_Init(void)
{
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_SET);
    pga_last_status = pga_write2(PGA_CMD_SDN_DIS, 0x00U);
    if (pga_last_status != HAL_OK) {
        pga_error_count++;
        pga_gain_requested = PGA_GAIN_INIT_CODE;
        pga_write_pending = 1U;
        pga_next_retry_tick = HAL_GetTick() + 10U;
        PGA112_AGC_Reset();
        return pga_last_status;
    }

    PGA112_AGC_Reset();
    return PGA112_SetGain(PGA_GAIN_INIT_CODE);
}

void PGA112_AGC_Reset(void)
{
    agc_up_hold = 0U;
    agc_down_hold = 0U;
}

uint8_t PGA112_AGC_Update(const uint16_t *samples, uint16_t len, uint8_t mode)
{
    if (len == 0U) return 0U;

    uint8_t clip_samples = 0U;
    uint16_t vmin = 4095U, vmax = 0U;
    for (uint16_t i = 0U; i < len; i++) {
        uint16_t s = samples[i];
        if ((s <= AGC_CLIP_LOW || s >= AGC_CLIP_HIGH) &&
            clip_samples < AGC_CLIP_SAMPLES) {
            clip_samples++;
        }
        if (s < vmin) vmin = s;
        if (s > vmax) vmax = s;
    }
    uint16_t vpp_raw = (uint16_t)(vmax - vmin);
    pga_last_vpp = vpp_raw;

    /* Real clipping creates multiple near-rail samples in one 5 ms block. */
    if (clip_samples >= AGC_CLIP_SAMPLES) {
        PGA112_AGC_Reset();
        if (g_pga_gain_live > PGA_GAIN_MIN_CODE) {
            return PGA112_RequestGain((uint8_t)(g_pga_gain_live - 1U));
        }
        return 0U;
    }

    if (mode == PGA112_AGC_DISABLED ||
        g_pga_gain_live > PGA_GAIN_MAX_CODE ||
        PGA112_IsPending()) return 0U;

    if (vpp_raw < AGC_VPP_LOW_COUNTS) {
        agc_down_hold = 0U;
        if (agc_up_hold < AGC_HOLD_BLOCKS) agc_up_hold++;
        if (agc_up_hold >= AGC_HOLD_BLOCKS) {
            uint8_t max_gain = (mode == PGA112_AGC_ACQUIRE) ?
                               AGC_ACQUIRE_MAX_CODE : PGA_GAIN_MAX_CODE;
            agc_up_hold = 0U;
            if (g_pga_gain_live < max_gain) {
                return PGA112_RequestGain((uint8_t)(g_pga_gain_live + 1U));
            }
        }
        return 0U;
    }

    if (mode == PGA112_AGC_LOCKED && vpp_raw > AGC_VPP_HIGH_COUNTS) {
        agc_up_hold = 0U;
        if (agc_down_hold < AGC_HOLD_BLOCKS) agc_down_hold++;
        if (agc_down_hold >= AGC_HOLD_BLOCKS) {
            agc_down_hold = 0U;
            if (g_pga_gain_live > PGA_GAIN_MIN_CODE) {
                return PGA112_RequestGain((uint8_t)(g_pga_gain_live - 1U));
            }
        }
        return 0U;
    }

    PGA112_AGC_Reset();
    return 0U;
}
