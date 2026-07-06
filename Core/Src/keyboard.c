/**
  ******************************************************************************
  * @file           : keyboard.c
  * @brief          : 4x4 Matrix Keyboard Driver — 声语信使项目
  *
  *   Hardware layout (PA1=ROW0顶, PA4=ROW3底, PA5=COL0左, PB3=COL3右):
  *                Col0    Col1    Col2    Col3
  *              (PA5)   (PA6)   (PA7)   (PB3)
  *   Row0 (PA1,顶): 1       2       3     开/关
  *   Row1 (PA2):    4       5       6     发送
  *   Row2 (PA3):    7       8       9     英/数
  *   Row3 (PA4,底): <-      0       ->    删除
  *
  *   Scanning: drive each row LOW in sequence, read column inputs.
  *   A column reads LOW => that key is pressed.
  *   Pull-up resistors on columns ensure idle = HIGH.
  *
  *   Note: COL0~2 on GPIOA, COL3 on GPIOB → use col_ports[] array.
  ******************************************************************************
  */

#include "keyboard.h"

/* Pin definitions (from CubeMX, defined in main.h) ------------------------*/
#define ROW_PORT    KB_ROW0_GPIO_Port

#define ROW0_PIN    KB_ROW0_Pin
#define ROW1_PIN    KB_ROW1_Pin
#define ROW2_PIN    KB_ROW2_Pin
#define ROW3_PIN    KB_ROW3_Pin

#define COL0_PIN    KB_COL0_Pin
#define COL1_PIN    KB_COL1_Pin
#define COL2_PIN    KB_COL2_Pin
#define COL3_PIN    KB_COL3_Pin

static const uint16_t    row_pins[4] = {ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN};
static const uint16_t    col_pins[4] = {COL0_PIN, COL1_PIN, COL2_PIN, COL3_PIN};
static GPIO_TypeDef * const col_ports[4] = {
    KB_COL0_GPIO_Port, KB_COL1_GPIO_Port, KB_COL2_GPIO_Port, KB_COL3_GPIO_Port
};

/* Debounce timing ---------------------------------------------------------*/
#define DEBOUNCE_MS       30    /* Press debounce: must be stable 30ms */
#define HOLD_REPEAT_MS    200   /* Long press repeat start */
#define REPEAT_RATE_MS    150   /* Repeat rate after hold */

/* Key name table ----------------------------------------------------------*/
static const char *key_names[16] = {
    "1",    "2",    "3",    "4",
    "5",    "6",    "7",    "8",
    "9",    "0",    "<-",   "->",
    "Del",  "Mode", "Send", "Power"
};

/* State variables ---------------------------------------------------------*/
static uint8_t  prev_key    = KEY_NONE;
static uint8_t  stable_key  = KEY_NONE;
static uint32_t press_tick  = 0;
static uint32_t repeat_tick = 0;
static uint8_t  repeat_phase = 0;  /* 0 = initial press, 1 = auto-repeat */

/* Key mapping table: [row][col] -> KEY_ code -----------------------------*/
/*
 *   Row                 Col0       Col1       Col2       Col3
 *   Row0 (PA1, 顶):     KEY_1      KEY_2      KEY_3      KEY_POWER
 *   Row1 (PA2):         KEY_4      KEY_5      KEY_6      KEY_SEND
 *   Row2 (PA3):         KEY_7      KEY_8      KEY_9      KEY_MODE
 *   Row3 (PA4, 底):     KEY_LEFT   KEY_0      KEY_RIGHT  KEY_DELETE
 */
static const uint8_t key_map[4][4] = {
    {KEY_1,      KEY_2,     KEY_3,      KEY_POWER },
    {KEY_4,      KEY_5,     KEY_6,      KEY_SEND  },
    {KEY_7,      KEY_8,     KEY_9,      KEY_MODE  },
    {KEY_LEFT,   KEY_0,     KEY_RIGHT,  KEY_DELETE},
};

/* -------------------------------------------------------------------------- */
/*                            Initialization                                  */
/* -------------------------------------------------------------------------- */

void Keyboard_Init(void)
{
    HAL_GPIO_WritePin(ROW_PORT,
        ROW0_PIN | ROW1_PIN | ROW2_PIN | ROW3_PIN, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/*                          Key Scanning                                      */
/* -------------------------------------------------------------------------- */

static uint8_t ScanMatrix(void)
{
    for (uint8_t row = 0; row < 4; row++) {
        HAL_GPIO_WritePin(ROW_PORT, row_pins[row], GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < 200; d++);

        for (uint8_t col = 0; col < 4; col++) {
            if (HAL_GPIO_ReadPin(col_ports[col], col_pins[col]) == GPIO_PIN_RESET) {
                HAL_GPIO_WritePin(ROW_PORT, row_pins[row], GPIO_PIN_SET);
                return key_map[row][col];
            }
        }

        HAL_GPIO_WritePin(ROW_PORT, row_pins[row], GPIO_PIN_SET);
    }
    return KEY_NONE;
}

/**
  * @brief  Public scan function with debounce and auto-repeat
  */
uint8_t Keyboard_Scan(void)
{
    HAL_Delay(20);

    uint8_t current_key = ScanMatrix();
    uint32_t now = HAL_GetTick();

    if (current_key == KEY_NONE) {
        prev_key   = KEY_NONE;
        stable_key = KEY_NONE;
        repeat_phase = 0;
        return KEY_NONE;
    }

    if (current_key == prev_key) {
        if (stable_key == KEY_NONE) {
            if ((now - press_tick) >= DEBOUNCE_MS) {
                stable_key = current_key;
                press_tick = now;
                repeat_tick = now;
                repeat_phase = 0;
                return stable_key;
            }
        } else {
            if (repeat_phase == 0) {
                if ((now - press_tick) >= HOLD_REPEAT_MS) {
                    repeat_phase = 1;
                    repeat_tick = now;
                    return stable_key;
                }
            } else {
                if ((now - repeat_tick) >= REPEAT_RATE_MS) {
                    repeat_tick = now;
                    return stable_key;
                }
            }
        }
    } else {
        prev_key   = current_key;
        press_tick = now;
        stable_key = KEY_NONE;
        repeat_phase = 0;
    }

    return KEY_NONE;
}

const char* Keyboard_GetKeyName(uint8_t key)
{
    if (key < 16) return key_names[key];
    return "?";
}
