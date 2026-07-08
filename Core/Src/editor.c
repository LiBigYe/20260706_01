/**
  ******************************************************************************
  * @file           : editor.c
  * @brief          : 短信息编辑器 — 声语信使项目
  ******************************************************************************
  */

#include "editor.h"
#include "oled.h"
#include "keyboard.h"
#include <string.h>
#include <stdio.h>

static const char *t9_map[MODE_COUNT][10] = {
    { "0\n","1",  "2(", "3)", "4+",  "5-", "6", "7*",  "8/", "9="  },
    { " \n", ".?!", "abc(", "def)", "ghi", "jkl", "mno", "pqrs", "tuv", "wxyz" },
    { " \n", ".?!", "ABC(", "DEF)", "GHI", "JKL", "MNO", "PQRS", "TUV", "WXYZ" },
};

static const char *mode_label[MODE_COUNT] = { "123", "abc", "ABC" };

typedef struct {
    char     buf[EDITOR_MAX_CHARS + 1];
    uint8_t  len;
    uint8_t  cur;
    uint8_t  mode;
    uint8_t  t9_key;
    uint32_t t9_tick;
    uint8_t  t9_tap;
    uint32_t blink_tick;
    uint8_t  blink_on;
    uint8_t  dirty;
    uint8_t  send_requested;
    char     tx_status[14];
} Editor;

static Editor ed;

static uint8_t CycleLen(uint8_t key) {
    if (key > 9) return 1;
    return (uint8_t)strlen(t9_map[ed.mode][key]);
}

static char GetChar(uint8_t key, uint8_t tap) {
    if (key > 9) return '?';
    const char *s = t9_map[ed.mode][key];
    uint8_t n = (uint8_t)strlen(s);
    if (n == 0) return '?';
    return s[(tap - 1) % n];
}

static void BufInsert(char ch) {
    if (ed.len >= EDITOR_MAX_CHARS) return;
    for (int8_t i = (int8_t)(ed.len); i > (int8_t)(ed.cur); i--)
        ed.buf[i] = ed.buf[i - 1];
    ed.buf[ed.cur] = ch;
    ed.len++;
    ed.cur++;
    ed.dirty = 1;
}

static void BufBackspace(void) {
    if (ed.len == 0 || ed.cur == 0) return;
    for (uint8_t i = ed.cur - 1; i < ed.len - 1; i++)
        ed.buf[i] = ed.buf[i + 1];
    ed.cur--;
    ed.len--;
    ed.buf[ed.len] = '\0';
    ed.dirty = 1;
}

static void BufReplaceLast(char ch) {
    if (ed.cur > 0 && ed.len > 0) {
        ed.buf[ed.cur - 1] = ch;
        ed.dirty = 1;
    }
}

static void HandleT9(uint8_t key) {
    uint8_t n = CycleLen(key);
    if (n == 0) return;
    uint32_t now = HAL_GetTick();
    if (key == ed.t9_key && (now - ed.t9_tick) < T9_TIMEOUT_MS) {
        if (ed.cur > 0) {
            ed.t9_tap++;
            if (ed.t9_tap > n) ed.t9_tap = 1;
            BufReplaceLast(GetChar(key, ed.t9_tap));
        }
    } else {
        if (ed.len < EDITOR_MAX_CHARS) {
            ed.t9_tap = 1;
            BufInsert(GetChar(key, ed.t9_tap));
            ed.t9_key = key;
        }
    }
    ed.t9_tick = now;
}

static void NextMode(void) {
    ed.mode = (ed.mode + 1) % MODE_COUNT;
    ed.t9_key = 0xFF;
    ed.t9_tap = 0;
    ed.dirty = 1;
}

void Editor_Init(void) {
    memset(&ed, 0, sizeof(ed));
    ed.t9_key = 0xFF;
    ed.blink_on = 1;
    ed.blink_tick = HAL_GetTick();
    ed.dirty = 1;
}

void Editor_HandleKey(uint8_t key) {
    switch (key) {
        case KEY_1: HandleT9(1); break;
        case KEY_2: HandleT9(2); break;
        case KEY_3: HandleT9(3); break;
        case KEY_4: HandleT9(4); break;
        case KEY_5: HandleT9(5); break;
        case KEY_6: HandleT9(6); break;
        case KEY_7: HandleT9(7); break;
        case KEY_8: HandleT9(8); break;
        case KEY_9: HandleT9(9); break;
        case KEY_0: HandleT9(0); break;
        case KEY_DELETE: BufBackspace(); break;
        case KEY_LEFT: if (ed.cur > 0) { ed.cur--; ed.dirty = 1; } break;
        case KEY_RIGHT: if (ed.cur < ed.len) { ed.cur++; ed.dirty = 1; } break;
        case KEY_FN: NextMode(); break;
        case KEY_SEND: if (ed.len > 0) ed.send_requested = 1; break;
        default: break;
    }
}

void Editor_Tick(void) {
    uint32_t now = HAL_GetTick();
    if ((now - ed.blink_tick) >= CURSOR_BLINK_MS) {
        ed.blink_tick = now;
        ed.blink_on = !ed.blink_on;
        ed.dirty = 1;
    }
    if (ed.t9_key != 0xFF && (now - ed.t9_tick) >= T9_TIMEOUT_MS) {
        ed.t9_key = 0xFF;
        ed.t9_tap = 0;
    }
}

#define COLS 21
#define VISIBLE_ROWS 7

void Editor_UpdateDisplay(void) {
    static uint32_t lr = 0;
    if (!ed.dirty) return;
    ed.dirty = 0;
    uint32_t now = HAL_GetTick();
    if ((now - lr) < 25) return;
    lr = now;

    OLED_Clear();

    /* Build visual lines */
    uint8_t ls[50], ll[50], tl = 0, cl = 0;
    uint16_t pos = 0;
    if (ed.len == 0) {
        ls[0] = 0; ll[0] = 0; tl = 1; cl = 0;
    } else {
        while (pos < ed.len) {
            ls[tl] = (uint8_t)pos;
            uint8_t l = 0;
            while (pos + l < ed.len && ed.buf[pos + l] != '\n' && l < COLS) {
                if (pos + l == ed.cur) cl = tl;
                l++;
            }
            if (pos + l < ed.len && ed.buf[pos + l] == '\n' && pos + l == ed.cur) cl = tl;
            ll[tl] = l;
            tl++;
            pos += l;
            if (pos < ed.len && ed.buf[pos] == '\n') pos++;
        }
        if (ed.len > 0 && ed.buf[ed.len - 1] == '\n') {
            ls[tl] = (uint8_t)ed.len; ll[tl] = 0; tl++;
        }
        for (uint8_t i = 0; i < tl; i++) {
            uint8_t le = ls[i] + ll[i];
            if (ed.cur >= ls[i] && ed.cur <= le) { cl = i; break; }
        }
        if (ed.cur == ed.len) cl = tl - 1;
    }

    /* Scroll */
    uint8_t st = 0;
    if (tl > VISIBLE_ROWS) {
        st = tl - VISIBLE_ROWS;
        if (cl < st) st = cl;
        if (cl >= st + VISIBLE_ROWS) st = cl - VISIBLE_ROWS + 1;
    }

    /* Render */
    for (uint8_t row = 0; row < VISIBLE_ROWS; row++) {
        uint8_t li = st + row;
        if (li >= tl) break;
        uint8_t start = ls[li], llen = ll[li], py = row * 8;
        for (uint8_t col = 0; col <= COLS; col++) {
            uint8_t px = col * 6;
            if (col < llen) {
                char ch = ed.buf[start + col];
                if (start + col == ed.cur && ed.blink_on)
                    OLED_ShowCharInvert(px, py, ch);
                else
                    OLED_ShowChar(px, py, ch);
            } else if (start + col == ed.cur && ed.blink_on) {
                for (uint8_t i = 0; i < 5; i++) OLED_DrawPixel(px + i + 1, py + 7, 1);
                break;
            } else if (col >= llen) {
                break;
            }
        }
    }

    /* Status bar */
    char line[32];
    uint8_t rem = EDITOR_MAX_CHARS - ed.len;
    if (ed.len < 10 || rem < 10)
        snprintf(line, sizeof(line), "%d/%d [%s]", ed.len, rem, mode_label[ed.mode]);
    else
        snprintf(line, sizeof(line), "%d/%d[%s]", ed.len, rem, mode_label[ed.mode]);
    OLED_ShowString(0, 7 * 8, line);
    if (ed.tx_status[0]) {
        uint8_t sl = (uint8_t)strlen(ed.tx_status);
        OLED_ShowString(128 - sl * 6, 7 * 8, ed.tx_status);
    }
    OLED_Refresh();
}

const char* Editor_GetBuffer(void) { return ed.buf; }
uint8_t Editor_GetLength(void) { return ed.len; }
uint8_t Editor_IsCursorAtEnd(void) { return (ed.cur >= ed.len) ? 1 : 0; }
uint8_t Editor_IsSendRequested(void) { return ed.send_requested; }
void Editor_ClearSendRequest(void) { ed.send_requested = 0; }

void Editor_SetTxStatus(const char *status) {
    if (status) {
        strncpy(ed.tx_status, status, sizeof(ed.tx_status) - 1);
        ed.tx_status[sizeof(ed.tx_status) - 1] = '\0';
    } else {
        ed.tx_status[0] = '\0';
    }
    ed.dirty = 1;
}
