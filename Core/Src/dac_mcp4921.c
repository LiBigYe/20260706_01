/**
  ******************************************************************************
  * @file           : dac_mcp4921.c
  * @brief          : MCP4921 12-bit SPI DAC DDS output implementation.
  *
  * SPI3 uses PB3/SCK and PB5/MOSI at 6.25 MHz. PA15 controls CS manually.
  * MCP4921 LDAC must be held low so every completed 16-bit write updates VOUT.
  ******************************************************************************
  */

#include "dac_mcp4921.h"
#include "fsk4_encoder.h"

#define MCP4921_COMMAND_ACTIVE_1X      0x3000U
#define MCP4921_COMMAND_SHUTDOWN       0x0000U
#define MCP4921_MIDSCALE               2048U
#define MCP4921_SPI_WAIT_LIMIT         1024U

static SPI_HandleTypeDef *dac_hspi;
static uint32_t dac_phase_accumulator;
static uint32_t dac_phase_increment;
static uint8_t dac_active;
static volatile uint8_t dac_fault;

static const uint16_t dac_sine_lut[256] = {
    2048, 2098, 2148, 2199, 2249, 2299, 2348, 2398,
    2447, 2497, 2545, 2594, 2642, 2690, 2738, 2785,
    2831, 2878, 2923, 2968, 3013, 3057, 3100, 3143,
    3185, 3227, 3267, 3307, 3347, 3385, 3423, 3459,
    3495, 3531, 3565, 3598, 3630, 3662, 3692, 3722,
    3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045,
    4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
    4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065,
    4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777,
    3750, 3722, 3692, 3662, 3630, 3598, 3565, 3531,
    3495, 3459, 3423, 3385, 3347, 3307, 3267, 3227,
    3185, 3143, 3100, 3057, 3013, 2968, 2923, 2878,
    2831, 2785, 2738, 2690, 2642, 2594, 2545, 2497,
    2447, 2398, 2348, 2299, 2249, 2199, 2148, 2098,
    2048, 1998, 1948, 1897, 1847, 1797, 1748, 1698,
    1649, 1599, 1551, 1502, 1454, 1406, 1358, 1311,
    1265, 1218, 1173, 1128, 1083, 1039,  996,  953,
     911,  869,  829,  789,  749,  711,  673,  637,
     601,  565,  531,  498,  466,  434,  404,  374,
     346,  319,  292,  267,  243,  220,  198,  177,
     157,  138,  121,  104,   89,   75,   62,   51,
      40,   31,   23,   16,   11,    7,    3,    2,
       1,    2,    3,    7,   11,   16,   23,   31,
      40,   51,   62,   75,   89,  104,  121,  138,
     157,  177,  198,  220,  243,  267,  292,  319,
     346,  374,  404,  434,  466,  498,  531,  565,
     601,  637,  673,  711,  749,  789,  829,  869,
     911,  953,  996, 1039, 1083, 1128, 1173, 1218,
    1265, 1311, 1358, 1406, 1454, 1502, 1551, 1599,
    1649, 1698, 1748, 1797, 1847, 1897, 1948, 1998,
};

static uint8_t DAC_MCP4921_WaitForFlag(uint32_t flag)
{
    uint32_t wait_count = MCP4921_SPI_WAIT_LIMIT;

    while ((dac_hspi->Instance->SR & flag) == 0U) {
        if (wait_count-- == 0U) {
            dac_fault = 1U;
            return 0U;
        }
    }

    return 1U;
}

static void DAC_MCP4921_WriteCommand(uint16_t command)
{
    volatile uint8_t discard;

    if (dac_hspi == NULL) {
        dac_fault = 1U;
        return;
    }

    HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_RESET);

    if (!DAC_MCP4921_WaitForFlag(SPI_SR_TXE)) {
        HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
        return;
    }
    *(__IO uint8_t *)&dac_hspi->Instance->DR = (uint8_t)(command >> 8);

    if (!DAC_MCP4921_WaitForFlag(SPI_SR_RXNE)) {
        HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
        return;
    }
    discard = *(__IO uint8_t *)&dac_hspi->Instance->DR;
    (void)discard;

    if (!DAC_MCP4921_WaitForFlag(SPI_SR_TXE)) {
        HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
        return;
    }
    *(__IO uint8_t *)&dac_hspi->Instance->DR = (uint8_t)command;

    if (!DAC_MCP4921_WaitForFlag(SPI_SR_RXNE)) {
        HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
        return;
    }
    discard = *(__IO uint8_t *)&dac_hspi->Instance->DR;
    (void)discard;

    if (!DAC_MCP4921_WaitForFlag(SPI_SR_TXE)) {
        HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
        return;
    }

    {
        uint32_t wait_count = MCP4921_SPI_WAIT_LIMIT;
        while ((dac_hspi->Instance->SR & SPI_SR_BSY) != 0U) {
            if (wait_count-- == 0U) {
                dac_fault = 1U;
                HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
                return;
            }
        }
    }

    HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
}

static void DAC_MCP4921_WriteValue(uint16_t value)
{
    DAC_MCP4921_WriteCommand(MCP4921_COMMAND_ACTIVE_1X | (value & 0x0FFFU));
}

void DAC_MCP4921_Init(SPI_HandleTypeDef *hspi)
{
    dac_hspi = hspi;
    dac_phase_accumulator = 0U;
    dac_phase_increment = FSK4_GetPhaseInc(FSK4_PILOT_LO);
    dac_active = 0U;
    dac_fault = 0U;

    HAL_GPIO_WritePin(DAC_CS_GPIO_Port, DAC_CS_Pin, GPIO_PIN_SET);
    DAC_MCP4921_WriteCommand(MCP4921_COMMAND_SHUTDOWN);
}

void DAC_MCP4921_SetFreq(uint8_t digit)
{
    if (digit < FSK4_FREQ_COUNT) {
        dac_phase_increment = FSK4_GetPhaseInc(digit);
    }
}

void DAC_MCP4921_Tick(void)
{
    uint8_t index;

    if (!dac_active || dac_fault) {
        return;
    }

    dac_phase_accumulator += dac_phase_increment;
    index = (uint8_t)(dac_phase_accumulator >> 24);
    DAC_MCP4921_WriteValue(dac_sine_lut[index]);
}

void DAC_MCP4921_OutputMidscale(void)
{
    dac_phase_increment = 0U;
    dac_phase_accumulator = 0U;

    if (dac_active && !dac_fault) {
        DAC_MCP4921_WriteValue(MCP4921_MIDSCALE);
    }
}

void DAC_MCP4921_Start(void)
{
    dac_active = 1U;
    DAC_MCP4921_OutputMidscale();
}

void DAC_MCP4921_Shutdown(void)
{
    if (dac_hspi != NULL && !dac_fault) {
        DAC_MCP4921_WriteCommand(MCP4921_COMMAND_SHUTDOWN);
    }
    dac_active = 0U;
}

uint8_t DAC_MCP4921_HasFault(void)
{
    return dac_fault;
}
