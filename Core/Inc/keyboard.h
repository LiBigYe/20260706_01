/**
  ******************************************************************************
  * @file           : keyboard.h
  * @brief          : 4x4 Matrix Keyboard Driver — 声语信使项目
  *                   Rows: PA0~PA3 (output push-pull)
  *                   Cols: PA4~PA7 (input pull-up)
  *
  *   物理布局 (PA0=顶, PA3=底, PA4=左, PA7=右):
  *        Col0    Col1    Col2    Col3
  *      (PA4)   (PA5)   (PA6)   (PA7)
  *   Row0(PA0,顶): 1       2       3     开/关
  *   Row1(PA1):    4       5       6     发送
  *   Row2(PA2):    7       8       9     英/数
  *   Row3(PA3,底): ←      0       →     删除
  ******************************************************************************
  */

#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Key codes (returned by Keyboard_Scan) -----------------------------------*/
#define KEY_NONE    0xFF   /* No key pressed */

/* Number keys (T9 input) */
#define KEY_1       0
#define KEY_2       1
#define KEY_3       2
#define KEY_4       3
#define KEY_5       4
#define KEY_6       5
#define KEY_7       6
#define KEY_8       7
#define KEY_9       8
#define KEY_0       9      /* Space in English mode, 0 in number mode */

/* Control keys */
#define KEY_LEFT    10     /* ← Cursor left */
#define KEY_RIGHT   11     /* → Cursor right */
#define KEY_DELETE  12     /* Delete (backspace) */
#define KEY_MODE    13     /* 英/数模式切换: abc → ABC → 123 */
#define KEY_SEND    14     /* 发送 (placeholder for future use) */
#define KEY_POWER   15     /* 开/关 (placeholder for future use) */

/* Function prototypes -----------------------------------------------------*/
void      Keyboard_Init(void);
uint8_t   Keyboard_Scan(void);
const char* Keyboard_GetKeyName(uint8_t key);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_H */
