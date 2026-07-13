/**
  ******************************************************************************
  * @file           : transmitter.c
  * @brief          : 4-FSK 发送状态机实现 (MCP4921 DAC DDS 版, v4)
  *
  *   时序 (16 kHz tick = 62.5 us):
  *     每符号   = 20 ms tone  = 320 ticks
  *     保护间隔 = 10 ms guard = 160 ticks (DAC 输出 DC 1.65V)
  *     每符号槽 = 30 ms       = 480 ticks
  *
  *   前导码: 200ms, 1500/3000Hz 每 40ms 交替 (无保护间隔)
  *   结束标志: 200ms, 3000Hz 连续音 (无保护间隔)
  *
  *   v4 设计要点: 10ms guard 确保接收端 5ms 切片总能捕获 >=1 个纯净 LO 块.
 *   接收端 v4 DPLL 利用下降沿 [HI,HI,LO] 实现符号定时恢复,
 *   能量门限 25000 免疫 DAC 阶梯输出不会引入 PWM 载波纹波. ≥1 个纯净 LO 块.
  *   接收端 v4 DPLL 利用下降沿 [HI,HI,LO] 实现符号定时恢复,
  *   能量门限 25000 免疫 DAC 阶梯输出不会引入 PWM 载波纹波.
  *
  *   每字符: 4 个 base-4 符号 (索引 0~73 → 4^4 = 256)
  *   48 字符: 192 数据符号 + 4 校验符号 = 196 符号
  *
  *   状态机:
  *     IDLE → PREAMBLE → DATA → CHECKSUM → POSTAMBLE → DONE
  *
  *   由 TIM3 ISR (16kHz) 驱动:
  *     TIM3_IRQHandler → HAL_TIM_IRQHandler
  *     → HAL_TIM_PeriodElapsedCallback → TX_Tick → DAC_MCP4921_Tick
  ******************************************************************************
  */

#include "transmitter.h"
#include "dac_mcp4921.h"
#include <string.h>

/* ========================================================================== */
/*  时序常量 (tick 单位, 16 kHz)                                               */
/* ========================================================================== */

#define TICK_FREQ           16000UL
#define TICKS_PER_SYMBOL    ((FSK4_T_SYMBOL    * TICK_FREQ) / 1000)  /* 320 */
#define TICKS_GUARD         ((FSK4_T_GUARD     * TICK_FREQ) / 1000)  /* 160 */
#define TICKS_PER_SLOT      (TICKS_PER_SYMBOL + TICKS_GUARD)          /* 480 */
#define TICKS_PREAMBLE      ((FSK4_T_PREAMBLE  * TICK_FREQ) / 1000)   /* 3200 */
#define TICKS_POSTAMBLE     ((FSK4_T_POSTAMBLE * TICK_FREQ) / 1000)   /* 3200 */
#define TICKS_PILOT_PERIOD  ((FSK4_PILOT_PERIOD * TICK_FREQ) / 1000)  /* 640 */

/* ========================================================================== */
/*  状态变量                                                                   */
/* ========================================================================== */

static FSK4_Encoder       tx_enc;
static uint8_t            tx_state;
static uint32_t           tx_tick;          /* 总 tick 计数 */
static uint16_t           tx_symbol_idx;    /* 当前符号在 symbols[] 中的索引 */
static uint16_t           tx_slot_tick;     /* 当前符号槽内的 tick (0..479) */
static uint8_t            tx_pilot_phase;   /* 前导相位: 0=LO(1500Hz), 1=HI(3000Hz) */
static uint8_t            tx_done_flag;

/* TIM3 handle (来自 main.c, 16kHz DDS 采样时钟) */
extern TIM_HandleTypeDef htim3;

static const char *state_names[6] = {
    "IDLE", "PREAMBLE", "DATA", "CHECKSUM", "POSTAMBLE", "DONE"
};

/* ========================================================================== */
/*  辅助函数                                                                   */
/* ========================================================================== */

/**
  * @brief  切换到指定符号的频率并重置槽内计时
  */
static void TX_SetSymbol(uint8_t digit)
{
    DAC_MCP4921_SetFreq(digit);
    tx_slot_tick = 0;
}

/**
  * @brief  进入保护间隔: PWM 输出 DC 中值 (无 AC 信号)
  */
static void TX_EnterGuard(void)
{
    DAC_MCP4921_OutputMidscale();
}

/* ========================================================================== */
/*  公开 API                                                                   */
/* ========================================================================== */

void TX_Init(void)
{
    memset(&tx_enc, 0, sizeof(tx_enc));
    tx_state       = TX_STATE_IDLE;
    tx_tick         = 0;
    tx_symbol_idx   = 0;
    tx_slot_tick    = 0;
    tx_pilot_phase  = 0;
    tx_done_flag    = 0;
}

void TX_Start(const char *text, uint8_t source_id, uint16_t target_mask)
{
    /* 编码文本 → symbols[] */
    FSK4_Init(&tx_enc, NULL);   /* sine_lut 参数仅用于验证, 传 NULL 跳过 */
    FSK4_Encode(&tx_enc, text, source_id, target_mask);

    /* 启动状态机 */
    tx_state       = TX_STATE_PREAMBLE;
    tx_tick         = 0;
    tx_symbol_idx   = 0;
    tx_slot_tick    = 0;
    tx_pilot_phase  = 0;
    tx_done_flag    = 0;

    DAC_MCP4921_Start();

    /* 初始频率: 前导 LO = 1500 Hz */
    TX_SetSymbol(FSK4_PILOT_LO);

    /* 发送已经开始，点亮红色发送指示灯 */
    HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);

    /* 重新启动 TIM3 中断 (16kHz DDS 采样时钟).
     * TX_ClearDone() 在上一次传输完成后将其停止.
     * 如果在 HAL_TIM_Base_Start_IT 时 TIM3 已经在运行, HAL 会安全处理. */
    HAL_TIM_Base_Start_IT(&htim3);
}

void TX_Tick(void)
{
    if (tx_state == TX_STATE_IDLE || tx_state == TX_STATE_DONE) {
        return;
    }

    switch (tx_state) {

    /* ================================================================== */
    /*  PREAMBLE: 200ms, 1500/3000Hz 每 40ms 交替 (无保护间隔)            */
    /* ================================================================== */
    case TX_STATE_PREAMBLE:
        if (tx_tick > 0 && (tx_tick % TICKS_PILOT_PERIOD) == 0) {
            tx_pilot_phase ^= 1;
            TX_SetSymbol(tx_pilot_phase ? FSK4_PILOT_HI : FSK4_PILOT_LO);
        }

        if (tx_tick >= TICKS_PREAMBLE) {
            tx_state      = TX_STATE_DATA;
            tx_symbol_idx = 0;
            tx_tick       = 0;
            tx_slot_tick  = 0;
            TX_SetSymbol(tx_enc.symbols[0]);
        }
        break;

    /* ================================================================== */
    /*  DATA: 逐符号发送 (20ms tone + 10ms guard)                         */
    /*   每字符 4 个符号, 校验符号放在最后 4 个                            */
    /*     symbol_count - FSK4_CHECKSUM_SYMBOLS  = 第一个校验符号的索引    */
    /* ================================================================== */
    case TX_STATE_DATA:
        if (tx_slot_tick >= TICKS_PER_SLOT) {
            tx_symbol_idx++;
            if (tx_symbol_idx >= tx_enc.symbol_count - FSK4_CHECKSUM_SYMBOLS) {
                tx_state      = TX_STATE_CHECKSUM;
                tx_symbol_idx = tx_enc.symbol_count - FSK4_CHECKSUM_SYMBOLS;
            }
            TX_SetSymbol(tx_enc.symbols[tx_symbol_idx]);
        } else if (tx_slot_tick == TICKS_PER_SYMBOL) {
            TX_EnterGuard();
        }
        break;

    /* ================================================================== */
    /*  CHECKSUM: 4 个校验符号 (最后 FSK4_CHECKSUM_SYMBOLS 个 symbols)     */
    /* ================================================================== */
    case TX_STATE_CHECKSUM:
        if (tx_slot_tick >= TICKS_PER_SLOT) {
            tx_symbol_idx++;
            if (tx_symbol_idx >= tx_enc.symbol_count) {
                tx_state = TX_STATE_POSTAMBLE;
                tx_tick  = 0;
                TX_SetSymbol(FSK4_PILOT_HI);   /* 3000 Hz 固定音 */
            } else {
                TX_SetSymbol(tx_enc.symbols[tx_symbol_idx]);
            }
        } else if (tx_slot_tick == TICKS_PER_SYMBOL) {
            TX_EnterGuard();
        }
        break;

    /* ================================================================== */
    /*  POSTAMBLE: 200ms 固定音 3000Hz (无保护间隔)                       */
    /* ================================================================== */
    case TX_STATE_POSTAMBLE:
        if (tx_tick >= TICKS_POSTAMBLE) {
            tx_state     = TX_STATE_DONE;
            tx_done_flag = 1;
            /* 设置 PWM 为中值, 并显式调用 Tick 确保输出生效.
             * OutputMidscale 已直接写 CCR1=512, 此处 Tick 为
             * 一致性调用 (phase_inc=0, phase_acc=0 → idx=0 → 512). */
            DAC_MCP4921_OutputMidscale();
            DAC_MCP4921_Tick();
            return;
        }
        break;
    }

    /* 每个 tick 输出一个 DDS 采样点 → PWM 占空比更新 */
    DAC_MCP4921_Tick();

    tx_tick++;
    tx_slot_tick++;
}

/* ========================================================================== */
/*  状态查询                                                                   */
/* ========================================================================== */

uint8_t TX_IsBusy(void)
{
    return (tx_state != TX_STATE_IDLE && tx_state != TX_STATE_DONE) ? 1 : 0;
}

uint8_t TX_IsDone(void)
{
    return tx_done_flag;
}

void TX_ClearDone(void)
{
    tx_done_flag = 0;
    tx_state     = TX_STATE_IDLE;

    /* 发送结束或取消，熄灭红色发送指示灯 */
    HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);

    /* 确保 DAC 输出回到中值 1.65V DC。 */
    DAC_MCP4921_OutputMidscale();

    /* 停止 TIM3 中断: 彻底消除状态机在后台泄漏的任何可能性.
     * 下一次 TX_Start() 会重新启动. */
    HAL_TIM_Base_Stop_IT(&htim3);
}
const char* TX_GetStateName(void)
{
    if (tx_state <= 5) return state_names[tx_state];
    return "?";
}

/* ========================================================================== */
/*  HAL 回调: TIM3 每次溢出 (16 kHz) → TX_Tick                               */
/* ========================================================================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        TX_Tick();
    }
}
