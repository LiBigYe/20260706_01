/**
  ******************************************************************************
  * @file           : oled.h
  * @brief          : SSD1306 OLED 128x64 I2C Driver (Hardware I2C via I2C1)
  *                   PB6=SCL, PB7=SDA (AF4), 400kHz, address 0x3C
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* OLED screen parameters --------------------------------------------------*/
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_ADDR       (0x3C << 1)  /* 8-bit I2C address (0x78) */

/* Font parameters: 6x8 pixels (5x7 glyph + 1 spacing column) */
#define FONT_WIDTH      6
#define FONT_HEIGHT     8
#define CHARS_PER_LINE  (OLED_WIDTH / FONT_WIDTH)  /* 21 */
#define TOTAL_PAGES     (OLED_HEIGHT / FONT_HEIGHT) /* 8  */

/* Public functions --------------------------------------------------------*/
void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_Fill(uint8_t color);
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void OLED_ShowChar(uint8_t x, uint8_t y, char ch);
void OLED_ShowCharInvert(uint8_t x, uint8_t y, char ch);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowStringInvert(uint8_t x, uint8_t y, const char *str);
void OLED_Refresh(void);
void OLED_ShowStartupScreen(void);
void OLED_SetDisplay(uint8_t enabled);
uint8_t OLED_IsDisplayEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
