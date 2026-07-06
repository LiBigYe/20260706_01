/**
  ******************************************************************************
  * @file           : oled.c
  * @brief          : SSD1306 OLED 128x64 I2C Driver (Hardware I2C via I2C1)
  *                   - 5x7 ASCII font (chars 32~126), 6-byte width with spacing
  *                   - Full 1024-byte framebuffer for flicker-free rendering
  *                   - I2C1: PB6=SCL, PB7=SDA, 400kHz Fast Mode
  ******************************************************************************
  */

#include "oled.h"
#include <string.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------- */
/*                   Hardware I2C Transport Layer                             */
/* -------------------------------------------------------------------------- */

static I2C_HandleTypeDef *oled_i2c;   /* Set by OLED_Init() */

/* SSD1306 control byte prefixes */
#define CTRL_CMD     0x00   /* Following byte is a command */
#define CTRL_DATA    0x40   /* Following bytes are display data */

/* Private variables -------------------------------------------------------*/
/* Framebuffer: 8 pages x 128 columns (1024 bytes) */
static uint8_t framebuffer[TOTAL_PAGES][OLED_WIDTH];

/* -------------------------------------------------------------------------- */
/*                      6x8 ASCII Font Table (chars 32~126)                   */
/*           5x7 glyph + 1 blank column = 6 bytes per char                   */
/* -------------------------------------------------------------------------- */
static const uint8_t font6x8[95][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* (space) */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) */
    {0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; */
    {0x00,0x08,0x14,0x22,0x41,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = */
    {0x41,0x22,0x14,0x08,0x00,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E */
    {0x7F,0x09,0x09,0x01,0x01,0x00}, /* F */
    {0x3E,0x41,0x41,0x51,0x32,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L */
    {0x7F,0x02,0x04,0x02,0x7F,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V */
    {0x7F,0x20,0x18,0x20,0x7F,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X */
    {0x03,0x04,0x78,0x04,0x03,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z */
    {0x00,0x00,0x7F,0x41,0x41,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* \ */
    {0x41,0x41,0x7F,0x00,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f */
    {0x08,0x14,0x54,0x54,0x3C,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j */
    {0x00,0x7F,0x10,0x28,0x44,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } */
    {0x08,0x08,0x2A,0x1C,0x08,0x00}, /* ~ */
};

/* -------------------------------------------------------------------------- */
/*                  Hardware I2C Low-level Communication                     */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Send a command byte to SSD1306 via HAL I2C
  * @param  cmd: command byte
  */
static void OLED_WriteCmd(uint8_t cmd)
{
    uint8_t buf[2] = {CTRL_CMD, cmd};
    HAL_I2C_Master_Transmit(oled_i2c, OLED_ADDR, buf, 2, 10);
}

/**
  * @brief  Send multiple data bytes to SSD1306 in one I2C transaction
  *         Prepends 0x40 (data control byte) before the payload.
  * @param  data: pointer to pixel data
  * @param  len:  number of data bytes
  */
static void OLED_WriteDataBulk(const uint8_t *data, uint16_t len)
{
    static uint8_t buf[129];  /* [0]=0x40 + up to 128 bytes payload */
    buf[0] = CTRL_DATA;
    memcpy(buf + 1, data, len);
    HAL_I2C_Master_Transmit(oled_i2c, OLED_ADDR, buf, len + 1, 100);
}

/* -------------------------------------------------------------------------- */
/*                       Public API Functions                                 */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Initialize the SSD1306 OLED display
  *         Init sequence for 128x64, hardware I2C1 interface.
  * @param  hi2c: pointer to HAL I2C handle (already initialized by CubeMX)
  */
void OLED_Init(I2C_HandleTypeDef *hi2c)
{
    oled_i2c = hi2c;  /* Store handle for subsequent OLED_WriteCmd/Data calls */

    OLED_WriteCmd(0xAE); /* Display OFF */

    /* Set display clock divide ratio / oscillator frequency */
    OLED_WriteCmd(0xD5);
    OLED_WriteCmd(0x80);

    /* Set multiplex ratio: 64 rows */
    OLED_WriteCmd(0xA8);
    OLED_WriteCmd(0x3F);

    /* Set display offset = 0 */
    OLED_WriteCmd(0xD3);
    OLED_WriteCmd(0x00);

    /* Set display start line = 0 */
    OLED_WriteCmd(0x40);

    /* Enable charge pump */
    OLED_WriteCmd(0x8D);
    OLED_WriteCmd(0x14);

    /* Set memory addressing mode = Horizontal */
    OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x00);

    /* Segment remap: column 127 mapped to SEG0 */
    OLED_WriteCmd(0xA1);

    /* COM scan direction: remapped (N-1 to 0) */
    OLED_WriteCmd(0xC8);

    /* Set COM pins hardware configuration */
    OLED_WriteCmd(0xDA);
    OLED_WriteCmd(0x12);

    /* Set contrast */
    OLED_WriteCmd(0x81);
    OLED_WriteCmd(0xCF);

    /* Set pre-charge period */
    OLED_WriteCmd(0xD9);
    OLED_WriteCmd(0xF1);

    /* Set VCOMH deselect level */
    OLED_WriteCmd(0xDB);
    OLED_WriteCmd(0x40);

    /* Display all on resume (normal mode) */
    OLED_WriteCmd(0xA4);

    /* Normal display (non-inverted) */
    OLED_WriteCmd(0xA6);

    /* Deactivate scroll */
    OLED_WriteCmd(0x2E);

    /* Clear framebuffer and display */
    OLED_Clear();
    OLED_Refresh();

    /* Display ON */
    OLED_WriteCmd(0xAF);
}

/**
  * @brief  Clear the framebuffer (all pixels off)
  */
void OLED_Clear(void)
{
    memset(framebuffer, 0x00, sizeof(framebuffer));
}

/**
  * @brief  Fill the entire framebuffer with a color
  * @param  color: 0 = off, 1 = on
  */
void OLED_Fill(uint8_t color)
{
    memset(framebuffer, color ? 0xFF : 0x00, sizeof(framebuffer));
}

/**
  * @brief  Set or clear a single pixel in the framebuffer
  * @param  x:     column position (0~127)
  * @param  y:     row position (0~63)
  * @param  color: 0 = clear, non-zero = set
  */
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;
    uint8_t page = y / 8;
    uint8_t bit  = y % 8;
    if (color) {
        framebuffer[page][x] |=  (1 << bit);
    } else {
        framebuffer[page][x] &= ~(1 << bit);
    }
}

/**
  * @brief  Draw a character at (x, y) pixel position in normal mode
  * @param  x:   left pixel coordinate
  * @param  y:   top pixel coordinate
  * @param  ch:  ASCII character to display (32~126)
  */
void OLED_ShowChar(uint8_t x, uint8_t y, char ch)
{
    if (ch < 32 || ch > 126) ch = ' ';
    uint8_t idx = ch - 32;
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t data = font6x8[idx][i];
        for (uint8_t j = 0; j < 8; j++) {
            if (data & (1 << j)) {
                OLED_DrawPixel(x + i, y + j, 1);
            }
        }
    }
}

/**
  * @brief  Draw a character at (x, y) in inverted mode (cursor highlight)
  * @param  x:   left pixel coordinate
  * @param  y:   top pixel coordinate
  * @param  ch:  ASCII character to display (32~126)
  */
void OLED_ShowCharInvert(uint8_t x, uint8_t y, char ch)
{
    if (ch < 32 || ch > 126) ch = ' ';
    uint8_t idx = ch - 32;

    /* First fill the 6x8 area with white */
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 8; j++) {
            OLED_DrawPixel(x + i, y + j, 1);
        }
    }
    /* Then clear pixels where the font has data (invert effect) */
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t data = font6x8[idx][i];
        for (uint8_t j = 0; j < 8; j++) {
            if (data & (1 << j)) {
                OLED_DrawPixel(x + i, y + j, 0);
            }
        }
    }
}

/**
  * @brief  Draw a null-terminated string at (x, y) pixel position
  * @param  x:   left pixel coordinate
  * @param  y:   top pixel coordinate
  * @param  str: pointer to string
  */
void OLED_ShowString(uint8_t x, uint8_t y, const char *str)
{
    while (*str) {
        /* Check if next char would go off-screen */
        if (x + FONT_WIDTH > OLED_WIDTH) {
            x = 0;
            y += FONT_HEIGHT;
        }
        if (y + FONT_HEIGHT > OLED_HEIGHT) break;
        OLED_ShowChar(x, y, *str);
        x += FONT_WIDTH;
        str++;
    }
}

/**
  * @brief  Draw a null-terminated string in inverted mode
  * @param  x:   left pixel coordinate
  * @param  y:   top pixel coordinate
  * @param  str: pointer to string
  */
void OLED_ShowStringInvert(uint8_t x, uint8_t y, const char *str)
{
    while (*str) {
        if (x + FONT_WIDTH > OLED_WIDTH) {
            x = 0;
            y += FONT_HEIGHT;
        }
        if (y + FONT_HEIGHT > OLED_HEIGHT) break;
        OLED_ShowCharInvert(x, y, *str);
        x += FONT_WIDTH;
        str++;
    }
}

/**
  * @brief  Refresh the OLED display by sending framebuffer to SSD1306
  *         Uses horizontal addressing mode: sends all 8 pages
  */
void OLED_Refresh(void)
{
    /* Set column address range: 0~127 */
    OLED_WriteCmd(0x21);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x7F);

    /* Set page address range: 0~7 */
    OLED_WriteCmd(0x22);
    OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x07);

    /* Send each page */
    for (uint8_t page = 0; page < TOTAL_PAGES; page++) {
        OLED_WriteDataBulk(framebuffer[page], OLED_WIDTH);
    }
}
