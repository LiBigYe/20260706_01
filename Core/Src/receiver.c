/**
  ******************************************************************************
  * @file           : receiver.c
  * @brief          : 4-FSK 接收状态机 — 声语信使接收端 (v4, DPLL下降沿同步)
  *
  *   核心改进: 引入 DPLL 下降沿触发 + 死区保护, 根治噪声假锁和符号吞噬.
  *
  *   v4 DPLL 架构:
  *     1. 无条件累积: 每个 5ms 块的 80 采样始终写入 320-entry ring buffer.
  *        无论 HI/LO, ring buffer 永远保存最近 20ms 的完整历史.
  *     2. 下降沿同步: 检测 [HI, HI, LO] 模式 (0x06), 代表 20ms 载波刚刚结束.
  *        此时 ring buffer 中必定躺着刚结束的 320 纯净载波 → Goertzel.
  *     3. 死区保护: 触发解码后闭眼 20ms (4 blocks), 免疫因时钟抖动/涂抹
  *        导致的重复误触发.
  *     4. 假锁拒绝: PREAMBLE 阶段第一个符号为 0xFF (噪声) → 直接退回 LISTENING.
  *     5. 自动恢复: 所有错误/超时自动退回 LISTENING, 不再永久卡死.
  *
  *   状态机:
  *     IDLE → LISTENING → PREAMBLE → DATA → DONE
  *            ↑______________↑__________|  (auto-reset on error)
  ******************************************************************************
  */

#include "receiver.h"
#include "fsk4_decoder.h"
#include "flash_store.h"
#include <string.h>

/* ========================================================================== */
/*  字符集映射 (与发送端 fsk4_encoder.c 一致)                                   */
/* ========================================================================== */

uint8_t RX_CharToIndex(char ch)
{
    if (ch >= 'a' && ch <= 'z') return (uint8_t)(ch - 'a');
    if (ch >= 'A' && ch <= 'Z') return (uint8_t)(ch - 'A' + 26);
    if (ch >= '0' && ch <= '9') return (uint8_t)(ch - '0' + 52);
    if (ch == '.')              return 62;
    if (ch == '?')              return 63;
    if (ch == '!')              return 64;
    if (ch == ' ')              return 65;
    if (ch == '$')              return 66;
    if (ch == '(')              return 67;
    if (ch == ')')              return 68;
    if (ch == '+')              return 69;
    if (ch == '-')              return 70;
    if (ch == '*')              return 71;
    if (ch == '/')              return 72;
    if (ch == '=')              return 73;
    if (ch == '\n')             return 74;
    return 255;
}

char RX_IndexToChar(uint8_t idx)
{
    if (idx <= 25)      return (char)('a' + idx);
    if (idx <= 51)      return (char)('A' + idx - 26);
    if (idx <= 61)      return (char)('0' + idx - 52);
    if (idx == 62)      return '.';
    if (idx == 63)      return '?';
    if (idx == 64)      return '!';
    if (idx == 65)      return ' ';
    if (idx == 66)      return '$';
    if (idx == 67)      return '(';
    if (idx == 68)      return ')';
    if (idx == 69)      return '+';
    if (idx == 70)      return '-';
    if (idx == 71)      return '*';
    if (idx == 72)      return '/';
    if (idx == 73)      return '=';
    if (idx == 74)      return '\n';
    return '?';
}

/* ========================================================================== */
/*  状态变量 (必须定义在函数之前, 因为 rx_decode_message 引用了它们)               */
/* ========================================================================== */

static FSK4_Decoder  rx_decoder;
static uint8_t       rx_state;
static uint8_t       rx_done_flag;

/* 包络检波 — 5-slot 能量历史 (5-bit shift register, bit0=newest) */
static uint8_t       rx_env_hist;       /* 8-bit shift register, 只用低 3-bit 做下降沿检测 */
static uint8_t       rx_env_count;      /* 已累积的 5ms 块数 (用于判断历史是否满) */

/* 包络检波 — 运行计数 */
static uint16_t      rx_block_total;    /* 当前阶段累积块数 */
static uint16_t      rx_consec_hi;      /* 连续 HI 块计数 */
static uint16_t      rx_consec_lo;      /* 连续 LO 块计数 */

/* 样本累积 — 320-entry ring buffer (无条件写入, 永远保存最近 20ms 历史) */
static uint16_t      rx_tone_ring[320];
static uint16_t      rx_tone_wpos;      /* write position (0..319, wraps) */
static uint8_t       rx_tone_full;      /* 1 = 已累积 >=320 样本 */

/* 符号收集 */
static uint8_t       rx_symbols[RX_TOTAL_SYMBOLS];  /* 196 */
static uint16_t      rx_symbol_idx;
static uint8_t       rx_holdoff;      /* 死区定时器 (blocks), 触发后闭眼 20ms */

/* 解码结果 */
static char          rx_message[RX_MAX_CHARS + 1];
static uint8_t       rx_msg_length;

/* 上一次成功接收的消息 (保留供显示滚动) */
static char          rx_display_msg[RX_MAX_CHARS + 1];
static uint8_t       rx_display_len;
static uint8_t       rx_scroll_line;   /* 显示起始行号 */

/* 调试 — 最近一次检测 */
/* (declared in the next section) */

/* ========================================================================== */
/*  内部辅助                                                                   */
/* ========================================================================== */

static uint8_t decode_char_index(uint8_t s3, uint8_t s2, uint8_t s1, uint8_t s0)
{
    return (uint8_t)((s3 << 6) | (s2 << 4) | (s1 << 2) | s0);
}

/**
  * @brief  验证 196 符号 + 4 checksum 符号的 XOR 校验和
  * @param  sym: 196 符号数组 (192 data + 4 checksum)
  * @retval 1=通过, 0=失败
  */
static uint8_t rx_verify_checksum(const uint8_t *sym)
{
    uint8_t calc_cs = 0;

    for (uint16_t i = 0; i < RX_MAX_CHARS; i++) {
        uint8_t idx = decode_char_index(
            sym[i * 4 + 0], sym[i * 4 + 1],
            sym[i * 4 + 2], sym[i * 4 + 3]
        );
        if (idx >= RX_CHARSET_SIZE) return 0;
        calc_cs ^= idx;
    }

    uint8_t rx_cs = decode_char_index(
        sym[RX_MAX_CHARS * 4 + 0], sym[RX_MAX_CHARS * 4 + 1],
        sym[RX_MAX_CHARS * 4 + 2], sym[RX_MAX_CHARS * 4 + 3]
    );

    return (calc_cs == rx_cs) ? 1 : 0;
}

/**
  * @brief  从符号数组解码消息字符串, 以 '$' 终止符截断
  *
  *   TX 端在真实内容后追加 '$' (索引 66) 再空格填充到 48 字符.
  *   接收端扫描 '$' 定位消息尾, 彻底消除尾部空格被误删的 Bug.
  *   若消息恰满 48 字符则无 '$' — 全 48 字符均为真实内容.
  */
static void rx_decode_message(void)
{
    /* 1. 先按定长 48 字符全部解码 */
    for (uint16_t i = 0; i < RX_MAX_CHARS; i++) {
        uint8_t idx = decode_char_index(
            rx_symbols[i * 4 + 0], rx_symbols[i * 4 + 1],
            rx_symbols[i * 4 + 2], rx_symbols[i * 4 + 3]
        );
        rx_message[i] = RX_IndexToChar(idx);
    }
    rx_message[RX_MAX_CHARS] = '\0';

    /* 2. 扫描 '$' 终止符确定真实消息长度 */
    uint8_t real_len = 0;
    while (real_len < RX_MAX_CHARS && rx_message[real_len] != '$') {
        real_len++;
    }
    rx_message[real_len] = '\0';
    rx_msg_length = real_len;
}
static uint8_t       rx_last_digit;
static float         rx_last_mag[4];

static const char *state_names[6] = {
    "IDLE", "LISTENING", "PREAMBLE", "DATA", "DONE", "ERROR"
};

/* ========================================================================== */
/*  内部 — 包络能量计算                                                        */
/* ========================================================================== */

/**
  * @brief  计算 80-sample 块的纯交流包络能量
  *
  *   v4 fix: 不使用固定的 2048 作为 DC 参考, 而是先算出这 80 个采样点的
  *   真实 DC 均值 (mean), 再减去 mean 计算交流能量. 这样即使运放偏置偏移
  *   (如 1.5V→ADC≈1860 而非 2048), 纯直流 guard 的能量仍然 ≈ 0.
  *
  *   实测参考值 (80-sample, 12-bit ADC, 去 DC 后):
  *     3.3Vpp 满幅正弦载波:        ~104,000  (HI)
  *     PWM 载波纹波穿透 (guard 上残留 0.5Vpp): ~12,400 (LO — 低于门限 25000)
  *     时间涂抹半块 (2.5ms tone+2.5ms guard): ~40,000 (HI — 高于门限)
  *     纯直流静音 (任何 DC 偏置):  ~0       (LO)
  *     浮动引脚噪声:               ~1,200   (LO)
  */
static uint32_t rx_compute_energy(const uint16_t *samples)
{
    /* 1. 动态计算当前 80 个采样点的真实 DC 偏置 (均值) */
    uint32_t sum = 0;
    for (uint8_t i = 0; i < RX_ENV_BLOCK_SIZE; i++) {
        sum += samples[i];
    }
    uint16_t mean = (uint16_t)(sum / RX_ENV_BLOCK_SIZE);

    /* 2. 扣除真实 DC, 计算纯交流包络能量 */
    uint32_t energy = 0;
    for (uint8_t i = 0; i < RX_ENV_BLOCK_SIZE; i++) {
        int32_t diff = (int32_t)samples[i] - (int32_t)mean;
        if (diff < 0) diff = -diff;
        energy += (uint32_t)diff;
    }
    return energy;
}

/* ========================================================================== */
/*  内部 — 模式检测                                                            */
/* ========================================================================== */

/**
  * @brief  检查最近的 3-slot 历史是否为下降沿 [HI, HI, LO]
  *
  *   rx_env_hist 为 shift register, bit0=最新.
  *   下降沿 [HI,HI,LO]: bit2..1 = 11 (HI), bit0 = 0 (LO).
  *   即 env_hist & 0x07 == 0x06 (0b110).
  *   这代表 20ms 载波刚刚结束, ring buffer 中必定躺着刚结束的
  *   320 纯净载波采样.
  */
static inline uint8_t rx_pattern_detected(void)
{
    if (rx_env_count < 3) return 0;
    return ((rx_env_hist & 0x07) == 0x06) ? 1 : 0;
}

/**
  * @brief  从 ring buffer 提取 320 个连续样本 (从 wpos 开始) 并运行 Goertzel
  * @retval digit 0~3 或 0xFF (噪声)
  */
static uint8_t rx_goertzel_from_ring(void)
{
    uint16_t tone_contig[FSK4_DECODER_BLOCK_SIZE];

    /* 从 rx_tone_wpos 开始顺时针读取 320 个样本 */
    uint16_t first_part = (uint16_t)(FSK4_DECODER_BLOCK_SIZE - rx_tone_wpos);
    if (first_part > 0) {
        memcpy(tone_contig, &rx_tone_ring[rx_tone_wpos], first_part * sizeof(uint16_t));
    }
    if (first_part < FSK4_DECODER_BLOCK_SIZE) {
        memcpy(tone_contig + first_part, rx_tone_ring,
               (FSK4_DECODER_BLOCK_SIZE - first_part) * sizeof(uint16_t));
    }

    return FSK4_Decoder_DetectBlock(&rx_decoder, tone_contig);
}

/* ========================================================================== */
/*  内部 — 状态转换                                                             */
/* ========================================================================== */

static void rx_enter_listening(void)
{
    /* 收信指示灯: 回到待机时熄灭 */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    rx_state         = RX_STATE_LISTENING;
    rx_env_hist      = 0;
    rx_env_count     = 0;
    rx_block_total   = 0;
    rx_consec_hi     = 0;
    rx_consec_lo     = 0;
    rx_tone_wpos     = 0;
    rx_tone_full     = 0;
    rx_symbol_idx    = 0;
    rx_holdoff       = 0;   /* 重置死区 */
    memset(rx_tone_ring, 0, sizeof(rx_tone_ring));
    memset(rx_symbols, 0, sizeof(rx_symbols));
}
static void rx_enter_error(void)
{
    rx_state     = RX_STATE_ERROR;
    rx_done_flag = 1;
}

static void rx_enter_done(void)
{
    rx_state     = RX_STATE_DONE;
    rx_done_flag = 1;
    /* 保存本次成功消息到显示缓冲区 */
    memcpy(rx_display_msg, rx_message, rx_msg_length + 1);
    rx_display_len = rx_msg_length;
    rx_scroll_line = 0;

    /* 非易失存储: 自动保存到内部 Flash (断电不丢失) */
    FlashStore_SaveMessage(rx_message, rx_msg_length);
}

/**
  * @brief  处理一个 5ms 子块: 无条件采样 → 能量计算 → 下降沿触发 → 状态机
  * @param  samples: 80 个 ADC 采样值
  *
  *   v4 DPLL 架构:
  *     1. 无条件写入 ring buffer — 永远保存最近 20ms 完整历史
  *     2. 下降沿 [HI,HI,LO] 触发 — 20ms 载波刚结束, ring 中有 320 纯净载波
  *     3. 死区 20ms — 免疫时间涂抹导致的重复触发
  *     4. 假锁拒绝 — PREAMBLE 首个符号 0xFF → 退回 LISTENING
  *     5. 自动恢复 — 所有错误退回 LISTENING
  */
static void rx_process_env_block(const uint16_t *samples)
{
    /* ── 1. 无条件累积样本到环形缓冲区 (永远保存最近 20ms 历史) ── */
    for (uint8_t i = 0; i < RX_ENV_BLOCK_SIZE; i++) {
        rx_tone_ring[rx_tone_wpos] = samples[i];
        rx_tone_wpos++;
        if (rx_tone_wpos >= FSK4_DECODER_BLOCK_SIZE) {
            rx_tone_wpos = 0;
            rx_tone_full = 1;
        }
    }

    /* ── 2. 计算能量并更新历史 ── */
    uint32_t energy = rx_compute_energy(samples);
    uint8_t  hi     = (energy >= RX_ENV_ENERGY_HI_THRESH) ? 1 : 0;

    /* 8-bit shift register 防止低位截断, 但只检查低 3-bit 做下降沿 */
    rx_env_hist = (uint8_t)(((uint16_t)rx_env_hist << 1) | hi);
    if (rx_env_count < 8) rx_env_count++;

    if (hi) { rx_consec_hi++; rx_consec_lo = 0; }
    else    { rx_consec_lo++; rx_consec_hi = 0; }

    rx_block_total++;

    /* 推进死区定时器 */
    if (rx_holdoff > 0) rx_holdoff--;

    /* ── 3. 下降沿检测 [HI, HI, LO] ── */
    uint8_t falling_edge = rx_pattern_detected();

    /* ====================================================================== */
    /*  状态机                                                                  */
    /* ====================================================================== */

    switch (rx_state) {

    /* ── LISTENING: 检测持续载波 → PREAMBLE ── */
    case RX_STATE_LISTENING:
        if (rx_consec_hi >= RX_ENV_PREAMBLE_MIN_HI) {
            rx_state       = RX_STATE_PREAMBLE;
            rx_block_total = rx_consec_hi;
            rx_consec_lo   = 0;
        }
        break;

    /* ── PREAMBLE: 等待第一个下降沿 → 验证假锁 → DATA ── */
    case RX_STATE_PREAMBLE:
        if (rx_block_total > RX_ENV_PREAMBLE_TIMEOUT || rx_consec_lo >= 8) {
            rx_enter_listening();
            break;
        }

        if (falling_edge && rx_tone_full && rx_holdoff == 0) {
            uint8_t digit = rx_goertzel_from_ring();

            /* 假锁保护: 若解出噪声 → 退回 LISTENING, 不入 DATA */
            if (digit != 0xFF) {
                rx_symbols[0] = digit;
                rx_symbol_idx = 1;
                rx_last_digit = digit;
                FSK4_Decoder_GetMagnitudes(&rx_decoder, rx_last_mag);

                /* 收信指示灯: 首个有效数据符号 → PC13 低电平点亮 */
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

                rx_state     = RX_STATE_DATA;
                rx_consec_lo = 1;
                rx_consec_hi = 0;
                /* 触发后闭眼 20ms (4 blocks), 免疫涂抹重复触发 */
                rx_holdoff   = 4;
            } else {
                rx_enter_listening();
            }
        }
        break;

    /* ── DATA: 逐符号下降沿采集 196 符号 ── */
    case RX_STATE_DATA:
        /* 丢失同步保护 1: 连续 8 LO → 自动退回待机 */
        if (rx_consec_lo >= RX_ENV_DATA_NOISE_MAX) {
            rx_enter_listening();
            break;
        }
        /* 丢失同步保护 2: 连续 40 HI 无 LO → 自动退回待机 */
        if (rx_consec_hi >= RX_ENV_DATA_SYNC_LOST) {
            rx_enter_listening();
            break;
        }

        if (falling_edge && rx_tone_full && rx_holdoff == 0) {
            uint8_t digit = rx_goertzel_from_ring();

            if (rx_symbol_idx < RX_TOTAL_SYMBOLS) {
                rx_symbols[rx_symbol_idx++] = digit;
            }

            rx_last_digit = digit;
            FSK4_Decoder_GetMagnitudes(&rx_decoder, rx_last_mag);

            /* 触发后闭眼 20ms */
            rx_holdoff = 4;

            if (rx_symbol_idx >= RX_TOTAL_SYMBOLS) {
                if (rx_verify_checksum(rx_symbols)) {
                    rx_decode_message();
                    rx_enter_done();
                } else {
                    /* 校验失败自动重置, 准备接下一包 */
                    rx_enter_listening();
                }
            }
        }
        break;

    case RX_STATE_ERROR:
    case RX_STATE_DONE:
    case RX_STATE_IDLE:
        break;
    }
}

/* ========================================================================== */
/*  公开 API                                                                   */
/* ========================================================================== */

void RX_Init(void)
{
    FSK4_Decoder_Init(&rx_decoder, FSK4_DECODER_BLOCK_SIZE);
    rx_state     = RX_STATE_IDLE;
    rx_done_flag = 0;
    memset(rx_symbols, 0, sizeof(rx_symbols));
    memset(rx_message, 0, sizeof(rx_message));
}

void RX_Start(void)
{
    rx_done_flag = 0;
    memset(rx_symbols, 0, sizeof(rx_symbols));
    memset(rx_message, 0, sizeof(rx_message));
    rx_msg_length = 0;
    rx_enter_listening();
}

void RX_Stop(void)
{
    rx_state     = RX_STATE_IDLE;
    rx_done_flag = 0;
}

/**
  * @brief  处理 DMA 半缓冲 (400 samples = 25ms)
  *
  *   拆分为 5 个 5ms (80-sample) 子块, 逐块驱动 DPLL 状态机.
  *   由 HAL_ADC_ConvHalfCpltCallback / HAL_ADC_ConvCpltCallback 调用.
  */
void RX_ProcessHalfBuffer(const uint16_t *buf)
{
    if (rx_state == RX_STATE_IDLE || rx_state == RX_STATE_DONE) return;

    for (uint8_t i = 0; i < RX_SUBBLOCKS_PER_HALF; i++) {
        rx_process_env_block(buf + (uint16_t)i * RX_ENV_BLOCK_SIZE);
    }
}

uint8_t RX_IsBusy(void)
{
    return (rx_state == RX_STATE_LISTENING ||
            rx_state == RX_STATE_PREAMBLE ||
            rx_state == RX_STATE_DATA) ? 1 : 0;
}

uint8_t RX_IsDone(void)          { return rx_done_flag; }
uint8_t RX_GetState(void)        { return rx_state; }
uint16_t RX_GetSymbolCount(void) { return rx_symbol_idx; }

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

void RX_GetLastSymbol(uint8_t *digit, float *mag)
{
    *digit = rx_last_digit;
    if (mag) {
        for (uint8_t i = 0; i < 4; i++) mag[i] = rx_last_mag[i];
    }
}

/* ── 显示用: 上次成功接收的消息 (RX_Start 不清除) ── */
const char* RX_GetDisplayMessage(void)  { return rx_display_msg; }
uint8_t     RX_GetDisplayLength(void)   { return rx_display_len; }

/* ── 滚动控制 ── */
uint8_t     RX_GetScrollLine(void)      { return rx_scroll_line; }

void RX_ScrollUp(void)
{
    /* 循环翻行: 在顶部翻上 → 跳到最后几行 */
    if (rx_scroll_line > 0) {
        rx_scroll_line--;
    }
}

void RX_ScrollDown(uint8_t total_lines)
{
    if (total_lines > VISIBLE_ROWS) {
        if (rx_scroll_line < total_lines - VISIBLE_ROWS)
            rx_scroll_line++;
    }
}

void RX_ScrollWrapUp(uint8_t total_lines)
{
    if (total_lines <= VISIBLE_ROWS) return;
    if (rx_scroll_line > 0)
        rx_scroll_line--;
    else
        rx_scroll_line = total_lines - VISIBLE_ROWS;
}

/* ── 接收状态文本 ── */
const char* RX_GetStatusString(void)
{
    if (rx_state == RX_STATE_LISTENING)  return "Stand By";
    if (rx_state == RX_STATE_PREAMBLE ||
        rx_state == RX_STATE_DATA)       return "Incoming Data";
    if (rx_state == RX_STATE_DONE)       return "Rx Complete";
    return "Stand By";
}
