/**
  ******************************************************************************
  * @file           : transmitter.c
 * @brief          : 4-FSK 发送状态机 (v5, 变长帧 + FEC + 同步音)
  *
  *   帧结构 (与接收端 voice_dsp/voice_fec 严格对应):
 *     PREAMBLE  : 2000ms, 1500/2400Hz 每 40ms 交替 — 频率序列捕获
  *     SYNC      : 1800Hz 20ms 单音 + 10ms guard — 唯一精定时锚点
  *     DATA      : 变长, 每符号 20ms tone + 10ms guard
  *     DATA TAIL : 30ms DC 隔离槽
  *     POSTAMBLE : 120ms 2400Hz 连续音
  *
  *   时序 (16 kHz tick = 62.5us):  tone=320, guard=160, slot=480 ticks.
  *
   *
  *   由 TIM3 ISR (16kHz) 驱动: HAL_TIM_PeriodElapsedCallback → TX_Tick → PWM_DDS_Tick.
  ******************************************************************************
  */
#include "transmitter.h"
#include "pwm_dds.h"
#include "voice_proto.h"
#include "voice_fec.h"
#include <string.h>

#define TICK_FREQ            16000UL
#define TICKS_TONE           VP_TONE_SAMPLES    /* 320 */
#define TICKS_GUARD          VP_GUARD_SAMPLES   /* 160 */
#define TICKS_SLOT           VP_SLOT_SAMPLES    /* 480 */
#define TICKS_PREAMBLE       ((VP_PREAMBLE_MS  * TICK_FREQ) / 1000)
#define TICKS_POSTAMBLE      ((VP_POSTAMBLE_MS * TICK_FREQ) / 1000)  /* 1920 */
#define TICKS_PILOT_PERIOD   ((VP_PILOT_PERIOD_MS * TICK_FREQ) / 1000) /* 640 */

/* 状态 */
#define ST_IDLE       0
#define ST_PREAMBLE   1
#define ST_SYNC       2
#define ST_DATA       3
#define ST_DATA_TAIL  4
#define ST_POSTAMBLE  5
#define ST_DONE       6

static uint8_t   tx_state;
static uint32_t  tx_tick;
static uint16_t  tx_slot_tick;
static uint16_t  tx_sym_idx;
static uint16_t  tx_sym_count;
static uint8_t   tx_pilot_phase;
static uint8_t   tx_done_flag;
static uint8_t   tx_symbols[VP_MAX_DATA_SYMBOLS];

extern TIM_HandleTypeDef htim3;

static const char *state_names[7] =
    {"IDLE", "PREAMBLE", "SYNC", "DATA", "TAIL", "POSTAMBLE", "DONE"};

static void tx_set_symbol(uint8_t digit) { PWM_DDS_SetFreq(digit); }
static void tx_guard(void)               { PWM_DDS_OutputMidscale(); }

/* ── 公开 API (main.c 不变) ── */

void TX_Init(void)
{
    tx_state = ST_IDLE;
    tx_tick = 0; tx_slot_tick = 0;
    tx_sym_idx = 0; tx_sym_count = 0;
    tx_pilot_phase = 0; tx_done_flag = 0;
}

void TX_Start(const char *text, uint8_t source_id, uint16_t target_mask)
{
    if (source_id < NET_MIN_DEVICE_ID || source_id > NET_MAX_DEVICE_ID)
        source_id = NET_MIN_DEVICE_ID;
    target_mask &= NET_VALID_TARGET_MASK;

    uint8_t payload[VP_MAX_PAYLOAD_BYTES];
    uint8_t plen = 0;
    payload[plen++] = source_id;
    payload[plen++] = (uint8_t)(target_mask & 0xFFU);
    payload[plen++] = (uint8_t)(target_mask >> 8);
    for (const char *t = text; *t && plen < VP_MAX_PAYLOAD_BYTES; t++)
        payload[plen++] = (uint8_t)*t;

    tx_sym_count = VoiceFEC_BuildDataSymbols(payload, plen, tx_symbols);

    tx_state = ST_PREAMBLE;
    tx_tick = 0; tx_slot_tick = 0;
    tx_sym_idx = 0; tx_pilot_phase = 0; tx_done_flag = 0;
    tx_set_symbol(VP_PILOT_LO);

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
    HAL_TIM_Base_Start_IT(&htim3);
}

void TX_Tick(void)
{
    if (tx_state == ST_IDLE || tx_state == ST_DONE) return;

    switch (tx_state) {

    case ST_PREAMBLE:
        if (tx_tick > 0 && (tx_tick % TICKS_PILOT_PERIOD) == 0) {
            tx_pilot_phase ^= 1;
            tx_set_symbol(tx_pilot_phase ? VP_PILOT_HI : VP_PILOT_LO);
        }
        if (tx_tick >= TICKS_PREAMBLE) {
            tx_state = ST_SYNC;
            tx_slot_tick = 0;
            tx_set_symbol(VP_SYNC_DIGIT);
        }
        break;

    case ST_SYNC:
        if (tx_slot_tick == TICKS_TONE) {
            tx_guard();
        } else if (tx_slot_tick >= TICKS_SLOT) {
            tx_state = ST_DATA;
            tx_slot_tick = 0;
            tx_sym_idx = 0;
            if (tx_sym_count > 0) tx_set_symbol(tx_symbols[0]);
        }
        break;

    case ST_DATA:
        if (tx_slot_tick == TICKS_TONE) {
            tx_guard();
        } else if (tx_slot_tick >= TICKS_SLOT) {
            tx_slot_tick = 0;
            tx_sym_idx++;
            if (tx_sym_idx >= tx_sym_count) {
                /* Keep the final data window isolated from the continuous
                 * 2400 Hz postamble when the two audio clocks drift. */
                tx_state = ST_DATA_TAIL;
                tx_tick = 0;
                tx_guard();
            } else {
                tx_set_symbol(tx_symbols[tx_sym_idx]);
            }
        }
        break;

    case ST_DATA_TAIL:
        if (tx_tick >= TICKS_SLOT) {
            tx_state = ST_POSTAMBLE;
            tx_tick = 0;
            tx_set_symbol(VP_PILOT_HI);
        }
        break;

    case ST_POSTAMBLE:
        if (tx_tick >= TICKS_POSTAMBLE) {
            tx_state = ST_DONE;
            tx_done_flag = 1;
            HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
            PWM_DDS_OutputMidscale();
            PWM_DDS_Tick();
            return;
        }
        break;

    default:
        break;
    }

    PWM_DDS_Tick();
    tx_tick++;
    tx_slot_tick++;
}

uint8_t TX_IsBusy(void)
{
    return (tx_state != ST_IDLE && tx_state != ST_DONE) ? 1 : 0;
}

uint8_t TX_IsDone(void) { return tx_done_flag; }

void TX_ClearDone(void)
{
    tx_done_flag = 0;
    tx_state = ST_IDLE;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
    PWM_DDS_OutputMidscale();
    HAL_TIM_Base_Stop_IT(&htim3);
}

const char* TX_GetStateName(void)
{
    if (tx_state <= ST_DONE) return state_names[tx_state];
    return "?";
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        TX_Tick();
    }
}
