/**
  ******************************************************************************
  * @file           : keyboard.c
  * @brief          : 4x4 Matrix Keyboard Driver — 声语信使项目 (半双工)
  *
  *   Hardware layout (PA0=ROW0顶, PA3=ROW3底, PA9=COL0左, PA12=COL3右):
  *                Col0    Col1    Col2    Col3
  *              (PA9)   (PA10)  (PA11)  (PA12)
  *   Row0 (PA0,顶): 1       2       3     开/关
  *   Row1 (PA1):    4       5       6     发送
  *   Row2 (PA2):    7       8       9     英/数
  *   Row3 (PA3,底): <-      0       ->    删除
  *
  *   Scanning: drive each row LOW in sequence, read column inputs.
  *   A column reads LOW => that key is pressed.
  *   Pull-up resistors on columns ensure idle = HIGH.
  ******************************************************************************
  */

#include "keyboard.h"

/* Pin definitions (from CubeMX, defined in main.h) ------------------------*/
#define ROW_PORT    GPIOA
#define COL_PORT    GPIOA

#define ROW0_PIN    KB_ROW0_Pin
#define ROW1_PIN    KB_ROW1_Pin
#define ROW2_PIN    KB_ROW2_Pin
#define ROW3_PIN    KB_ROW3_Pin

#define COL0_PIN    KB_COL0_Pin
#define COL1_PIN    KB_COL1_Pin
#define COL2_PIN    KB_COL2_Pin
#define COL3_PIN    KB_COL3_Pin

static const uint16_t row_pins[4] = {ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN};
static const uint16_t col_pins[4] = {COL0_PIN, COL1_PIN, COL2_PIN, COL3_PIN};

/* Debounce timing ---------------------------------------------------------*/
#define DEBOUNCE_MS       30    /* Press debounce: must be stable 30ms */
#define HOLD_REPEAT_MS    200   /* Long press repeat start */
#define REPEAT_RATE_MS    150   /* Repeat rate after hold */

/* Key name table ----------------------------------------------------------*/
static const char *key_names[16] = {
    "1",    "2",    "3",    "4",
    "5",    "6",    "7",    "8",
    "9",    "0",    "<-",   "->",
    "删除", "英/数","发送", "开/关"
};

/* State variables ---------------------------------------------------------*/
static uint8_t  prev_key    = KEY_NONE;
static uint8_t  stable_key  = KEY_NONE;
static uint32_t press_tick  = 0;
static uint32_t repeat_tick = 0;
static uint8_t  repeat_phase = 0;  /* 0 = initial press, 1 = auto-repeat */

/* Key mapping table: [row][col] -> KEY_ code -----------------------------*/
/*
 *   Row (GPIO)               Col0(PA9) Col1(PA10) Col2(PA11) Col3(PA12)
 *   Row0 (PA0, 顶):          KEY_1     KEY_2     KEY_3     KEY_POWER
 *   Row1 (PA1):              KEY_4     KEY_5     KEY_6     KEY_SEND
 *   Row2 (PA2):              KEY_7     KEY_8     KEY_9     KEY_FN
 *   Row3 (PA3, 底):          KEY_LEFT  KEY_0     KEY_RIGHT KEY_DELETE
 */
static const uint8_t key_map[4][4] = {
    {KEY_1,      KEY_2,     KEY_3,      KEY_POWER },
    {KEY_4,      KEY_5,     KEY_6,      KEY_SEND  },
    {KEY_7,      KEY_8,     KEY_9,      KEY_FN  },
    {KEY_LEFT,   KEY_0,     KEY_RIGHT,  KEY_DELETE},
};

/* -------------------------------------------------------------------------- */
/*                            Initialization                                  */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Initialize keyboard GPIO pins
  *         GPIO configuration (mode, pull, speed) is handled by CubeMX in
  *         MX_GPIO_Init().  Here we only ensure rows are idle HIGH.
  */
void Keyboard_Init(void)
{
    /* Ensure all row outputs are HIGH (keys idle, no pull-down path) */
    HAL_GPIO_WritePin(ROW_PORT,
        ROW0_PIN | ROW1_PIN | ROW2_PIN | ROW3_PIN, GPIO_PIN_SET);
}

/* -------------------------------------------------------------------------- */
/*                          Key Scanning                                      */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Scan the 4x4 matrix once
  * @retval Key code (KEY_1~KEY_POWER) or KEY_NONE if no key pressed
  */
static uint8_t ScanMatrix(void)
{
    for (uint8_t row = 0; row < 4; row++) {
        /* Drive current row LOW */
        HAL_GPIO_WritePin(ROW_PORT, row_pins[row], GPIO_PIN_RESET);

        /* Small delay for signal settling */
        for (volatile uint32_t d = 0; d < 200; d++);

        /* Read each column */
        for (uint8_t col = 0; col < 4; col++) {
            if (HAL_GPIO_ReadPin(COL_PORT, col_pins[col]) == GPIO_PIN_RESET) {
                /* Key pressed! Restore row before returning */
                HAL_GPIO_WritePin(ROW_PORT, row_pins[row], GPIO_PIN_SET);
                return key_map[row][col];
            }
        }

        /* Restore row to HIGH */
        HAL_GPIO_WritePin(ROW_PORT, row_pins[row], GPIO_PIN_SET);
    }
    return KEY_NONE;
}

/**
  * @brief  Public scan function with debounce and auto-repeat
  *
  *         Call this periodically in main loop.
  *         Uses debounce: key must be stable for DEBOUNCE_MS.
  *         After HOLD_REPEAT_MS, auto-repeat kicks in at REPEAT_RATE_MS.
  *
  * @retval Key code or KEY_NONE
  */
uint8_t Keyboard_Scan(void)
{
    HAL_Delay(20);  /* Throttle scan to ~50Hz */

    uint8_t current_key = ScanMatrix();
    uint32_t now = HAL_GetTick();

    /* No key currently detected */
    if (current_key == KEY_NONE) {
        prev_key   = KEY_NONE;
        stable_key = KEY_NONE;
        repeat_phase = 0;
        return KEY_NONE;
    }

    /* Debounce: same key as last scan? */
    if (current_key == prev_key) {
        /* Same key, check if debounce time passed */
        if (stable_key == KEY_NONE) {
            if ((now - press_tick) >= DEBOUNCE_MS) {
                /* Key is stable, register press */
                stable_key = current_key;
                press_tick = now;
                repeat_tick = now;
                repeat_phase = 0;
                return stable_key;
            }
        } else {
            /* Auto-repeat only for cursor/delete keys. Control and number keys
             * are edge-only so address selection cannot toggle repeatedly. */
            uint8_t repeatable = (stable_key == KEY_LEFT ||
                                  stable_key == KEY_RIGHT ||
                                  stable_key == KEY_DELETE) ? 1U : 0U;
            if (repeatable) {
                if (repeat_phase == 0) {
                    if ((now - press_tick) >= HOLD_REPEAT_MS) {
                        repeat_phase = 1;
                        repeat_tick = now;
                        return stable_key;
                    }
                } else if ((now - repeat_tick) >= REPEAT_RATE_MS) {
                    repeat_tick = now;
                    return stable_key;
                }
            }
        }
    } else {
        /* Different key detected, start debounce for new key */
        prev_key   = current_key;
        press_tick = now;
        stable_key = KEY_NONE;
        repeat_phase = 0;
    }

    return KEY_NONE;
}

/**
  * @brief  Get the display name of a key
  * @param  key: Key code (0~15)
  * @retval String describing the key
  */
uint8_t Keyboard_IsPressed(void)
{
    return stable_key != KEY_NONE ? 1U : 0U;
}

const char* Keyboard_GetKeyName(uint8_t key)
{
    if (key < 16) {
        return key_names[key];
    }
    return "?";
}

