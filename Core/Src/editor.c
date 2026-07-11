/**
  ******************************************************************************
  * @file           : editor.c
  * @brief          : 短信息编辑器 — 声语信使项目
  *
  *  功能：
  *     - T9多击输入（九宫格）
  *     - 48字符缓冲区，光标插入/删除
  *     - 3种输入模式：abc / ABC / 123（英/数模式切换，移除旧 Sym 模式）
  *     - OLED显示：文本区 + 分隔线 + 状态栏 + 提示栏
  *     - 光标闪烁
  *
  *  键盘映射（参照 按键功能定义.txt）：
  *   key 1: .?!          key 2: abc        key 3: def
  *   key 4: ghi          key 5: jkl        key 6: mno
  *   key 7: pqrs         key 8: tuv        key 9: wxyz
  *   key 0: 空格 (英文) / 0 (数字)
  *   ← / → : 光标移动    Del: 删除(Backspace)
  *   Mode: 英/数模式切换 (abc→ABC→123→abc)
  *   Send / Power: 预留后续功能
  ******************************************************************************
  */

#include "editor.h"
#include "oled.h"
#include "keyboard.h"
#include <string.h>

/* ========================================================================== */
/*                           T9 字符映射表                                     */
/* ========================================================================== */

/* 每个字符串 = 该键在当前模式下的所有字符，按循环顺序排列
 * 修改点：
 *   - key 1: 简化 ".?!" (原 ".,!?'\"@/:;" → 按按键功能定义精简)
 *   - key 0: 英文模式 " " (空格)，数字模式 "0"
 *   - 移除 MODE_SYMBOL (4→3 模式)
 */
static const char *t9_map[MODE_COUNT][10] = {
    /* MODE_NUMBER */ { "0\n","1",  "2(", "3)", "4+",  "5-", "6", "7*",  "8/", "9="  },
    /* MODE_LOWER  */ { " \n", ".?!", "abc(", "def)", "ghi", "jkl", "mno", "pqrs", "tuv", "wxyz" },
    /* MODE_UPPER  */ { " \n", ".?!", "ABC(", "DEF)", "GHI", "JKL", "MNO", "PQRS", "TUV", "WXYZ" },
};

static const char *mode_label[MODE_COUNT] = { "123", "abc", "ABC" };

/* ========================================================================== */
/*                           编辑器状态                                        */
/* ========================================================================== */

typedef struct {
    char     buf[EDITOR_MAX_CHARS + 1];   /* 缓冲区 */
    uint8_t  len;                          /* 当前长度 */
    uint8_t  cur;                          /* 光标位置 (0..len) */
    uint8_t  mode;                         /* 输入模式 */

    /* T9 多击状态 */
    uint8_t  t9_key;                       /* 上一个按下的T9键(0~9) */
    uint32_t t9_tick;                      /* 上次按键时间 */
    uint8_t  t9_tap;                       /* 连续按键次数(从1开始) */

    /* 显示状态 */
    uint32_t blink_tick;                   /* 光标闪烁定时 */
    uint8_t  blink_on;                     /* 光标是否可见 */
    uint8_t  dirty;                        /* 需要刷新显示 */

    uint8_t  send_requested;               /* 发送键已按下 (main.c 轮询) */
    char     tx_status[14];                /* Tx 状态文本 (右对齐) */
} Editor;

static Editor ed;

/* ========================================================================== */
/*                        字符查找辅助函数                                     */
/* ========================================================================== */

/** 获取某键在当前位置的字符总数 */
static uint8_t CycleLen(uint8_t key)
{
    if (key > 9) return 1;
    return (uint8_t)strlen(t9_map[ed.mode][key]);
}

/** 获取某键在 tap 次按压时应显示的字符 (tap 从1开始) */
static char GetChar(uint8_t key, uint8_t tap)
{
    if (key > 9) return '?';
    const char *s = t9_map[ed.mode][key];
    uint8_t n = (uint8_t)strlen(s);
    if (n == 0) return '?';
    uint8_t idx = (tap - 1) % n;
    return s[idx];
}

/* ========================================================================== */
/*                          缓冲区操作                                         */
/* ========================================================================== */

static void BufInsert(char ch)
{
    if (ed.len >= EDITOR_MAX_CHARS) return;
    /* 从右向左移位 */
    for (int8_t i = (int8_t)(ed.len); i > (int8_t)(ed.cur); i--)
        ed.buf[i] = ed.buf[i - 1];
    ed.buf[ed.cur] = ch;
    ed.len++;
    ed.cur++;
    ed.dirty = 1;
}

static void BufBackspace(void)
{
    if (ed.len == 0 || ed.cur == 0) return;
    for (uint8_t i = ed.cur - 1; i < ed.len - 1; i++)
        ed.buf[i] = ed.buf[i + 1];
    ed.cur--;
    ed.len--;
    ed.buf[ed.len] = '\0';
    ed.dirty = 1;
}

static void BufReplaceLast(char ch)
{
    /* 替换光标前一个字符 (用于T9同键切换) */
    if (ed.cur > 0 && ed.len > 0) {
        ed.buf[ed.cur - 1] = ch;
        ed.dirty = 1;
    }
}

/* ========================================================================== */
/*                           T9 多击输入                                      */
/* ========================================================================== */

static void HandleT9(uint8_t key)
{
    uint8_t n = CycleLen(key);
    if (n == 0) return;

    uint32_t now = HAL_GetTick();

    if (key == ed.t9_key && (now - ed.t9_tick) < T9_TIMEOUT_MS) {
        /* 同键连续按 → 切换到下一个字符 (仅当光标前有字符可替换) */
        if (ed.cur > 0) {
            ed.t9_tap++;
            if (ed.t9_tap > n) ed.t9_tap = 1;
            BufReplaceLast(GetChar(key, ed.t9_tap));
        }
    } else {
        /* 新键或超时 → 插入新字符 (缓冲区满则直接拒绝) */
        if (ed.len < EDITOR_MAX_CHARS) {
            ed.t9_tap = 1;
            BufInsert(GetChar(key, ed.t9_tap));
            ed.t9_key = key;
        }
    }
    ed.t9_tick = now;
}

/* ========================================================================== */
/*                          模式切换                                           */
/* ========================================================================== */

static void NextMode(void)
{
    ed.mode = (ed.mode + 1) % MODE_COUNT;
    /* 切换模式时重置T9状态 */
    ed.t9_key = 0xFF;
    ed.t9_tap = 0;
    ed.dirty = 1;
}

/* ========================================================================== */
/*                         公开API                                             */
/* ========================================================================== */

void Editor_Init(void)
{
    memset(&ed, 0, sizeof(ed));
    ed.t9_key = 0xFF;
    ed.blink_on = 1;
    ed.blink_tick = HAL_GetTick();
    ed.dirty = 1;
}

/* -------------------------------------------------------------------------- */
/*  按键分发                                                                  */
/* -------------------------------------------------------------------------- */
void Editor_HandleKey(uint8_t key)
{
    switch (key) {
        /* ---- T9 数字键 ---- */
        case KEY_1:  HandleT9(1);  break;
        case KEY_2:  HandleT9(2);  break;
        case KEY_3:  HandleT9(3);  break;
        case KEY_4:  HandleT9(4);  break;
        case KEY_5:  HandleT9(5);  break;
        case KEY_6:  HandleT9(6);  break;
        case KEY_7:  HandleT9(7);  break;
        case KEY_8:  HandleT9(8);  break;
        case KEY_9:  HandleT9(9);  break;
        case KEY_0:  HandleT9(0);  break;   /* 空格(英文) / 0(数字) */

        /* ---- 控制键 ---- */
        case KEY_DELETE:  BufBackspace();   break;   /* 删除光标前字符 */
        case KEY_LEFT:
            if (ed.cur > 0) { ed.cur--; ed.dirty = 1; }
            break;
        case KEY_RIGHT:
            if (ed.cur < ed.len) { ed.cur++; ed.dirty = 1; }
            break;
        case KEY_FN:  NextMode();          break;   /* 英/数模式切换 */

        /* 预留键 */
        case KEY_SEND:
            if (ed.len > 0) ed.send_requested = 1;
            break;
        case KEY_POWER:  /* TODO: 开关机/休眠 */  break;

        default: break;
    }
}

/* -------------------------------------------------------------------------- */
/*  定时器（每50ms调一次）                                                     */
/* -------------------------------------------------------------------------- */
void Editor_Tick(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - ed.blink_tick) >= CURSOR_BLINK_MS) {
        ed.blink_tick = now;
        ed.blink_on = !ed.blink_on;
        ed.dirty = 1;
    }

    /* T9超时 → 重置 */
    if (ed.t9_key != 0xFF && (now - ed.t9_tick) >= T9_TIMEOUT_MS) {
        ed.t9_key = 0xFF;
        ed.t9_tap = 0;
    }
}

/* ========================================================================== */
/*                           OLED 画面渲染                                     */
/* ========================================================================== */

#define COLS   (OLED_WIDTH / FONT_WIDTH)    /* 21 */
#define ROWS   8                              /* 8行 ×8px */
#define VISIBLE_ROWS  7                       /* 文本区: 第0~6行 (预留第7行给状态栏) */

void Editor_UpdateDisplay(void)
{
    static uint32_t last_refresh = 0;

    if (!ed.dirty) return;
    ed.dirty = 0;

    uint32_t now = HAL_GetTick();
    if ((now - last_refresh) < 25) return;
    last_refresh = now;

    OLED_Clear();

    /* ── 第1遍: 构建每行起止, 计算总行数和光标所在行 ── */
    uint8_t  line_start[50];
    uint8_t  line_len[50];
    uint8_t  total_lines = 0;
    uint8_t  cursor_line = 0;
    uint16_t pos = 0;

    /* 构建所有视觉行 */
    if (ed.len == 0) {
        line_start[0] = 0;
        line_len[0]   = 0;
        total_lines   = 1;
        cursor_line   = 0;
    } else {
        while (pos < ed.len) {
            line_start[total_lines] = (uint8_t)pos;
            uint8_t l = 0;
            while (pos + l < ed.len && ed.buf[pos + l] != '\n' && l < COLS) {
                if (pos + l == ed.cur) cursor_line = total_lines;
                l++;
            }
            /* 光标在换行符上 → 属当前行行尾 */
            if (pos + l < ed.len && ed.buf[pos + l] == '\n' && pos + l == ed.cur)
                cursor_line = total_lines;

            line_len[total_lines] = l;
            total_lines++;
            pos += l;
            if (pos < ed.len && ed.buf[pos] == '\n') pos++; /* 跳过换行符 */
        }
        /* 缓冲末尾是 \n → 追加一个空行 */
        if (ed.len > 0 && ed.buf[ed.len - 1] == '\n') {
            line_start[total_lines] = (uint8_t)ed.len;
            line_len[total_lines]   = 0;
            total_lines++;
        }
        /* 后处理: 根据视觉行范围精确确定 cursor_line */
        cursor_line = 0;
        for (uint8_t i = 0; i < total_lines; i++) {
            uint8_t line_end = line_start[i] + line_len[i];
            if (ed.cur >= line_start[i] && ed.cur <= line_end) {
                cursor_line = i;
                break;
            }
            cursor_line = i + 1;  /* 光标不在本行 → 继续查找 */
        }
        /* 光标超出所有行 (如缓冲末) → 归到最后一行 */
        if (ed.cur == ed.len) cursor_line = total_lines - 1;
    }

    /* ── 第2遍: 计算滚动起点 (光标始终可见, 默认显示末尾 VISIBLE_ROWS 行) ── */
    uint8_t scroll_top = 0;
    if (total_lines > VISIBLE_ROWS) {
        scroll_top = total_lines - VISIBLE_ROWS;          /* 默认最末7行 */
        if (cursor_line < scroll_top)
            scroll_top = cursor_line;                     /* 光标在上方 → 滚上去 */
        if (cursor_line >= scroll_top + VISIBLE_ROWS)
            scroll_top = cursor_line - VISIBLE_ROWS + 1;  /* 光标在下方 → 滚下来 */
    }

    /* ── 第3遍: 渲染 VISIBLE_ROWS 行 ── */
    for (uint8_t row = 0; row < VISIBLE_ROWS; row++) {
        uint8_t li = scroll_top + row;
        if (li >= total_lines) break;

        uint8_t  start = line_start[li];
        uint8_t  llen  = line_len[li];
        uint8_t  py    = row * FONT_HEIGHT;

        for (uint8_t col = 0; col <= COLS; col++) {
            uint8_t px = col * FONT_WIDTH;
            if (col < llen) {
                char ch = ed.buf[start + col];
                if (start + col == ed.cur && ed.blink_on)
                    OLED_ShowCharInvert(px, py, ch);
                else
                    OLED_ShowChar(px, py, ch);
            } else if (start + col == ed.cur && ed.blink_on) {
                /* 光标在行尾 / COLS 边界 → 下划线 */
                for (uint8_t i = 0; i < 5; i++)
                    OLED_DrawPixel(px + i + 1, py + 7, 1);
                break;
            } else if (col >= llen) {
                break;
            }
        }
    }

    /* ── 状态栏 (已输入/剩余 [模式] 靠左, Tx 状态靠右) ── */
    uint8_t remaining = EDITOR_MAX_CHARS - ed.len;
    uint8_t footer_x = 0U;
    const uint8_t footer_y = 7U * FONT_HEIGHT;

    if (ed.len >= 10U) {
        OLED_ShowChar(footer_x, footer_y, (char)('0' + ed.len / 10U));
        footer_x += FONT_WIDTH;
    }
    OLED_ShowChar(footer_x, footer_y, (char)('0' + ed.len % 10U));
    footer_x += FONT_WIDTH;
    OLED_ShowChar(footer_x, footer_y, '/');
    footer_x += FONT_WIDTH;
    if (remaining >= 10U) {
        OLED_ShowChar(footer_x, footer_y, (char)('0' + remaining / 10U));
        footer_x += FONT_WIDTH;
    }
    OLED_ShowChar(footer_x, footer_y, (char)('0' + remaining % 10U));
    footer_x += FONT_WIDTH;
    OLED_ShowChar(footer_x, footer_y, '[');
    footer_x += FONT_WIDTH;
    OLED_ShowString(footer_x, footer_y, mode_label[ed.mode]);
    footer_x += (uint8_t)strlen(mode_label[ed.mode]) * FONT_WIDTH;
    OLED_ShowChar(footer_x, footer_y, ']');

    if (ed.tx_status[0] != '\0') {
        uint8_t slen = (uint8_t)strlen(ed.tx_status);
        uint8_t status_x = (uint8_t)(OLED_WIDTH - slen * FONT_WIDTH);
        if (status_x > footer_x + FONT_WIDTH) {
            OLED_ShowString(status_x, footer_y, ed.tx_status);
        }
    }

    OLED_Refresh();
}

/* -------------------------------------------------------------------------- */
/*  缓冲区访问 (供发送模块使用)                                                */
/* -------------------------------------------------------------------------- */

const char* Editor_GetBuffer(void)
{
    return ed.buf;
}

uint8_t Editor_GetLength(void)
{
    return ed.len;
}


uint8_t Editor_IsCursorAtEnd(void)
{
    return (ed.cur >= ed.len) ? 1U : 0U;
}
uint8_t Editor_IsSendRequested(void)
{
    return ed.send_requested;
}

void Editor_ClearSendRequest(void)
{
    ed.send_requested = 0;
}

void Editor_SetTxStatus(const char *status)
{
    if (status) {
        strncpy(ed.tx_status, status, sizeof(ed.tx_status) - 1);
        ed.tx_status[sizeof(ed.tx_status) - 1] = '\0';
    } else {
        ed.tx_status[0] = '\0';
    }
    ed.dirty = 1;
}
