/**
  ******************************************************************************
  * @file           : dac_mcp4921.h
  * @brief          : MCP4921 SPI DAC DDS output for the voice messenger.
  ******************************************************************************
  */

#ifndef __DAC_MCP4921_H
#define __DAC_MCP4921_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void DAC_MCP4921_Init(SPI_HandleTypeDef *hspi);
void DAC_MCP4921_SetFreq(uint8_t digit);
void DAC_MCP4921_Tick(void);
void DAC_MCP4921_OutputMidscale(void);
void DAC_MCP4921_Start(void);
void DAC_MCP4921_Shutdown(void);
uint8_t DAC_MCP4921_HasFault(void);

#ifdef __cplusplus
}
#endif

#endif /* __DAC_MCP4921_H */
