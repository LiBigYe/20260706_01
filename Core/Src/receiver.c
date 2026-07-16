/**
  ******************************************************************************
  * @file           : receiver.c
  * @brief          : 4-FSK 接收状态机 (v5) — 基于 voice_dsp 一次性同步 + FEC
  *
  *   物理层同步与解调交给可移植核心 voice_dsp / voice_fec:
  *     - 前导唤醒 + 1800Hz 同步音一次性锁定 30ms 符号栅格 (不再依赖每符号
  *       guard 下降沿, 免疫室内混响).
  *     - Goertzel 取每 tone 中间 10ms, ±1 bin 容忍频偏, 频谱置信度判决
  *       (无绝对幅值硬门限, 提升接收距离).
  *     - 变长帧 + Hamming(7,4) + 交织 + CRC-8 (抗突发干扰, 取代裸 XOR).
  *
  *   本文件保留旧的公开 API 与地址过滤/显示缓冲/滚动逻辑, main.c 无需改动.
  *   payload 布局: [source_id][mask_lo][mask_hi][text...] (变长, 无填充).
  ******************************************************************************
  */
#include "receiver.h"
#include "voice_proto.h"
#include "voice_fec.h"
#include "voice_dsp.h"
#include "pga112.h"
#include <string.h>

/* ========================================================================== */
/*  字符集映射 (保留供 main.c / 兼容旧调用)                                      */
/* ========================================================================== */
uint8_t RX_CharToIndex(char ch)
{
    if (ch >= 'a' && ch <= 'z') return (uint8_t)(ch - 'a');
    if (ch >= 'A' && ch <= 'Z') return (uint8_t)(ch - 'A' + 26);
    if (ch >= '0' && ch <= '9') return (uint8_t)(ch - '0' + 52);
    switch (ch) {
    case '.': return 62; case '?': return 63; case '!': return 64;
    case ' ': return 65; case '$': return 66; case '(': return 67;
    case ')': return 68; case '+': return 69; case '-': return 70;
    case '*': return 71; case '/': return 72; case '=': return 73;
    case '\n': return 74;
    default: break;
    }
    return 255;
}

char RX_IndexToChar(uint8_t idx)
{
    if (idx <= 25) return (char)('a' + idx);
    if (idx <= 51) return (char)('A' + idx - 26);
    if (idx <= 61) return (char)('0' + idx - 52);
    switch (idx) {
    case 62: return '.'; case 63: return '?'; case 64: return '!';
    case 65: return ' '; case 66: return '$'; case 67: return '(';
    case 68: return ')'; case 69: return '+'; case 70: return '-';
    case 71: return '*'; case 72: return '/'; case 73: return '='; case 74: return '\n';
    }
    return '?';
}

/* ========================================================================== */
/*  状态变量                                                                   */
/* ========================================================================== */
static VoiceRx  vrx;
static uint8_t  rx_state;      /* 映射到 RX_STATE_* 供 main.c 查询 */
static uint8_t  rx_done_flag;

static char     rx_message[RX_MAX_CHARS + 1];
static uint8_t  rx_msg_length;
static uint8_t  rx_source_id;
static uint16_t rx_target_mask;

static char     rx_display_msg[RX_MAX_CHARS + 1];
static uint8_t  rx_display_len;
static uint8_t  rx_display_source_id;
static uint8_t  rx_scroll_line;

static uint8_t  rx_last_digit;
static float    rx_last_mag[4];

static const char *state_names[6] =
    {"IDLE", "LISTENING", "PREAMBLE", "DATA", "DONE", "ERROR"};

/* 把 voice_dsp 内部状态映射到旧 RX_STATE_* 语义 */
static void rx_sync_state(void)
{
    switch (vrx.state) {
    case VD_LISTEN:   rx_state = RX_STATE_LISTENING; break;
    case VD_PREAMBLE: rx_state = RX_STATE_PREAMBLE;  break;
    case VD_DATA:     rx_state = RX_STATE_DATA;      break;
    case VD_DONE:     rx_state = RX_STATE_DONE;      break;
    default:          rx_state = RX_STATE_LISTENING; break;
    }
}

/* ========================================================================== */
/*  内部 — 帧完成处理 (地址过滤 + 填充显示缓冲)                                  */
/* ========================================================================== */
static void rx_on_frame_done(void)
{
    /* payload = [src, mask_lo, mask_hi, text...] */
    if (vrx.payload_len < VP_HEADER_BYTES) { VoiceRx_Start(&vrx); return; }

    rx_source_id   = vrx.payload[0];
    rx_target_mask = (uint16_t)vrx.payload[1] | ((uint16_t)vrx.payload[2] << 8);

    uint8_t text_len = (uint8_t)(vrx.payload_len - VP_HEADER_BYTES);
    if (text_len > RX_MAX_CHARS) text_len = RX_MAX_CHARS;
    for (uint8_t i = 0; i < text_len; i++)
        rx_message[i] = (char)vrx.payload[VP_HEADER_BYTES + i];
    rx_message[text_len] = '\0';
    rx_msg_length = text_len;

    /* 地址过滤: 源 ID 合法 且 (广播或含本机) 才收下 */
    if (rx_source_id >= NET_MIN_DEVICE_ID && rx_source_id <= NET_MAX_DEVICE_ID &&
        NET_IsAddressedTo(rx_target_mask, g_device_id)) {
        rx_state     = RX_STATE_DONE;
        rx_done_flag = 1;
        memcpy(rx_display_msg, rx_message, rx_msg_length + 1);
        rx_display_len       = rx_msg_length;
        rx_display_source_id = rx_source_id;
        rx_scroll_line       = 0;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    } else {
        /* 非本机 → 丢弃, 重新监听 */
        VoiceRx_Start(&vrx);
        rx_sync_state();
    }
}

/* ========================================================================== */
/*  公开 API                                                                   */
/* ========================================================================== */
void RX_Init(void)
{
    VoiceRx_Init(&vrx);
    rx_state = RX_STATE_IDLE;
    rx_done_flag = 0;
    memset(rx_message, 0, sizeof(rx_message));
    memset(rx_display_msg, 0, sizeof(rx_display_msg));
    rx_msg_length = 0; rx_display_len = 0;
    rx_source_id = 0; rx_target_mask = 0; rx_display_source_id = 0;
    rx_scroll_line = 0; rx_last_digit = 0xFF;
    for (int i = 0; i < 4; i++) rx_last_mag[i] = 0.0f;
}

void RX_Start(void)
{
    VoiceRx_Start(&vrx);
    rx_done_flag = 0;
    memset(rx_message, 0, sizeof(rx_message));
    rx_msg_length = 0; rx_source_id = 0; rx_target_mask = 0;
    rx_sync_state();
    /* 回到 LISTENING: 强制设 64x 增益 */
    PGA112_SetGain(PGA_GAIN_INIT_CODE);
    /* 收信指示灯待机熄灭 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void RX_Stop(void)
{
    rx_state = RX_STATE_IDLE;
    rx_done_flag = 0;
}

void RX_ProcessHalfBuffer(const uint16_t *buf)
{
    if (rx_state == RX_STATE_IDLE || rx_state == RX_STATE_DONE) return;

    for (uint8_t i = 0; i < RX_SUBBLOCKS_PER_HALF; i++) {
        const uint16_t *sub = buf + (uint16_t)i * RX_ENV_BLOCK_SIZE;

        /* ========================================================= */
        /* 状态驱动的冻结式 AGC 调度策略 (v5.1 Early-Freeze)          */
        /* ========================================================= */
        if (vrx.state == VD_LISTEN) {
            /* 待机态: 死锁在 32x, 防止突发巨响降增益后无法恢复 (致聋) */
            if (PGA112_GetGain() != PGA_GAIN_INIT_CODE) {
                PGA112_SetGain(PGA_GAIN_INIT_CODE);
                PGA112_AGC_Reset();
            }
        } else if (vrx.state == VD_PREAMBLE && vrx.pilot_trans < 2U) {
            /* 前导前期 (pilot_trans < 2 = 前 ~80ms): AGC 活跃, 允许飙升至 128x.
             * pilot_trans >= 2 后提前冻结: 此时 AGC 已收敛 80ms (16 轮 Update),
             * 提前锁定增益, 为即将到来的 1800Hz SYNC 同步音精定时锚点
             * 确保绝对干净的物理波形 — 零增益跳变瞬态噪声 — 防止 SYNC
             * 检测窗的 True SNR 被阶跃能量污染 → 锚点丢失 → 整帧报废. */
            PGA112_AGC_Update(sub, RX_ENV_BLOCK_SIZE, 1U);
        }
        /* 前导后期 (pilot_trans >= 2) 及数据态 (VD_DATA):
           绝对冻结增益, 禁止任何 PGA 寄存器写入.
           FSK 是频率调制 → 偶发的 ADC 削顶 (方波化) 不会破坏过零点
           频率信息, Goertzel 基波提取不受影响 (限幅器效应).
           但增益跳变引入的 AM 包络调制会立刻引发频谱泄露. */
        /* ========================================================= */

        /* 将子块推入 DSP, 传入当前真实的物理增益码供能量归一化使用 */
        uint8_t was_data = (vrx.state == VD_DATA);
        uint8_t done = VoiceRx_PushBlock(&vrx, sub, PGA112_GetGain());
        rx_last_digit = vrx.last_digit;
        uint8_t now_data = (vrx.state == VD_DATA);
        if (now_data && !was_data)
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        else if (!now_data && was_data)
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        if (done) {
            rx_on_frame_done();
            return;
        }
    }
    rx_sync_state();
}

uint8_t RX_IsBusy(void)
{
    return (rx_state == RX_STATE_LISTENING ||
            rx_state == RX_STATE_PREAMBLE ||
            rx_state == RX_STATE_DATA) ? 1 : 0;
}

uint8_t RX_IsFrameActive(void)
{
    return (rx_state == RX_STATE_PREAMBLE || rx_state == RX_STATE_DATA) ? 1U : 0U;
}

uint32_t RX_GetNoiseFloor(void)      { return vrx.noise_floor; }
uint32_t RX_GetEnergyThreshold(void) { return vrx.last_energy; }
uint8_t  RX_IsDone(void)             { return rx_done_flag; }
uint8_t  RX_GetState(void)           { return rx_state; }
uint16_t RX_GetSymbolCount(void)     { return vrx.sym_count; }

void RX_ClearDone(void)
{
    rx_done_flag = 0;
    rx_state     = RX_STATE_IDLE;
}

const char* RX_GetStateName(void)
{
    if (rx_state <= 5) return state_names[rx_state];
    return "?";
}

const char* RX_GetMessage(void)       { return rx_message; }
uint8_t     RX_GetMessageLength(void) { return rx_msg_length; }
uint8_t     RX_GetSourceId(void)      { return rx_source_id; }
uint16_t    RX_GetTargetMask(void)    { return rx_target_mask; }
uint8_t     RX_GetDisplaySourceId(void) { return rx_display_source_id; }

void RX_GetLastSymbol(uint8_t *digit, float *mag)
{
    *digit = rx_last_digit;
    if (mag) { for (uint8_t i = 0; i < 4; i++) mag[i] = rx_last_mag[i]; }
}

/* ---- v5.1 诊断 getter (供 LiveWatch 实时监控) ---- */
uint8_t RX_GetPilotHits(void)   { return vrx.pilot_hits; }
uint8_t RX_GetEraseRun(void)    { return vrx.erase_run; }
float   RX_GetLastSNR(void)     { return vrx.last_conf; }
uint8_t RX_GetVGain(void)       { return PGA112_GetGain(); }
uint8_t RX_GetDspSubState(void) { return vrx.state; }

const char* RX_GetDisplayMessage(void)  { return rx_display_msg; }
uint8_t     RX_GetDisplayLength(void)   { return rx_display_len; }
uint8_t     RX_GetScrollLine(void)      { return rx_scroll_line; }

void RX_ScrollUp(void)
{
    if (rx_scroll_line > 0) rx_scroll_line--;
}

void RX_ScrollDown(uint8_t total_lines)
{
    if (total_lines > VISIBLE_ROWS) {
        if (rx_scroll_line < total_lines - VISIBLE_ROWS) rx_scroll_line++;
    }
}

void RX_ScrollWrapUp(uint8_t total_lines)
{
    if (total_lines <= VISIBLE_ROWS) return;
    if (rx_scroll_line > 0) rx_scroll_line--;
    else rx_scroll_line = total_lines - VISIBLE_ROWS;
}

const char* RX_GetStatusString(void)
{
    if (rx_state == RX_STATE_LISTENING) return "Stand By";
    if (rx_state == RX_STATE_PREAMBLE || rx_state == RX_STATE_DATA) return "Incoming Data";
    if (rx_state == RX_STATE_DONE) return "Rx Complete";
    return "Stand By";
}
