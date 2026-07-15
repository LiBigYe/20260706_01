/**
  ******************************************************************************
  * @file           : pga112.c
  * @brief          : PGA112 驱动 + ADC 反馈 AGC 实现
  *
  *  经 SPI2 (PB13 SCK / PB15 MOSI, 单向) + PB12 CS 盲写增益命令.
  *  AGC 只在接收监听态调整, 锁帧期间冻结, 与 v5 频谱判决互补.
  ******************************************************************************
  */
#include "pga112.h"
#include "main.h"

extern SPI_HandleTypeDef hspi2;

static uint8_t  pga_gain = PGA_GAIN_INIT_CODE;
static uint16_t agc_low_count = 0;   /* 连续"信号过低"块计数 */

/* ---- 底层: 拉低 CS → 发 2 字节 → 拉高 CS ---- */
static void pga_write2(uint8_t b0, uint8_t b1)
{
    uint8_t tx[2] = { b0, b1 };
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_RESET);
    /* 单向 (1LINE) master 发送; 超时 10ms 足够 (2 字节 @1.5MHz ≈ 11µs) */
    (void)HAL_SPI_Transmit(&hspi2, tx, 2, 10U);
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_SET);
}

void PGA112_SetGain(uint8_t gain_code)
{
    if (gain_code > PGA_GAIN_MAX_CODE) gain_code = PGA_GAIN_MAX_CODE;
    pga_gain = gain_code;
    /* 低字节: 高 4 位=增益码, 低 4 位=通道码 */
    uint8_t data = (uint8_t)((gain_code << 4) | (PGA_CHANNEL & 0x0FU));
    pga_write2(PGA_CMD_WRITE, data);
}

uint8_t PGA112_GetGain(void) { return pga_gain; }

void PGA112_Init(void)
{
    /* CS 空闲高 */
    HAL_GPIO_WritePin(PG112_CS_GPIO_Port, PG112_CS_Pin, GPIO_PIN_SET);
    /* 退出关机模式 (SDN_DIS), 确保数字接口使能; POR 后寄存器本为全 0 */
    pga_write2(PGA_CMD_SDN_DIS, 0x00U);
    /* 设初始增益 */
    PGA112_SetGain(PGA_GAIN_INIT_CODE);
    agc_low_count = 0;
}

void PGA112_AGC_Reset(void)
{
    agc_low_count = 0;
}

uint8_t PGA112_AGC_Update(const uint16_t *samples, uint16_t len, uint8_t frame_active)
{
    if (len == 0U) return 0U;

    /* 统计: 是否削顶 + 相对中值(2048)的最大单边幅度 */
    uint8_t  clipped = 0U;
    uint16_t max_amp = 0U;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t s = samples[i];
        if (s <= AGC_CLIP_LOW || s >= AGC_CLIP_HIGH) clipped = 1U;
        int32_t d = (int32_t)s - 2048;
        if (d < 0) d = -d;
        if ((uint16_t)d > max_amp) max_amp = (uint16_t)d;
    }

    /* 锁帧期间冻结增益: 只统计, 不动 (避免帧中途幅度跳变干扰同步) */
    if (frame_active) { agc_low_count = 0U; return 0U; }

    /* 1) 削顶保护 (最高优先, 即时降档) */
    if (clipped) {
        agc_low_count = 0U;
        if (pga_gain > PGA_GAIN_MIN_CODE) {
            PGA112_SetGain((uint8_t)(pga_gain - 1U));
            return 1U;
        }
        return 0U;
    }

    /* 2) 幅度过高 (接近削顶) → 降一档 */
    if (max_amp >= AGC_AMP_HIGH) {
        agc_low_count = 0U;
        if (pga_gain > PGA_GAIN_MIN_CODE) {
            PGA112_SetGain((uint8_t)(pga_gain - 1U));
            return 1U;
        }
        return 0U;
    }

    /* 3) 幅度持续过低 → 升一档 (带迟滞, 连续 N 块才升, 防噪声抬档) */
    if (max_amp <= AGC_AMP_LOW) {
        if (agc_low_count < 0xFFFFU) agc_low_count++;
        if (agc_low_count >= AGC_LOW_HOLD) {
            agc_low_count = 0U;
            if (pga_gain < PGA_GAIN_MAX_CODE) {
                PGA112_SetGain((uint8_t)(pga_gain + 1U));
                return 1U;
            }
        }
        return 0U;
    }

    /* 4) 处于目标窗口 [LOW, HIGH] 之间 → 不动 (迟滞区) */
    agc_low_count = 0U;
    return 0U;
}
