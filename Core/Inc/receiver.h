/**
  ******************************************************************************
  * @file           : receiver.h
  * @brief          : 4-FSK 接收状态机 — 声语信使接收端 (v4, DPLL下降沿同步)
  *
  *   硬件:
  *     ADC1_IN8 (PB0) <- 音频输入
  *     TIM2 TRGO 触发 ADC @ 16 kHz (PLL 50MHz)
  *     DMA2_Stream0 循环传输 ADC 数据到缓冲区
  *     I2C1 (PB6/PB7) -> OLED 128x64
  *     4x4 键盘 (PA1~PA7, PB3)
  *
  *   用法:
  *     RX_Start()              : 开始监听
  *     RX_ProcessHalfBuffer()  : DMA HT/TC ISR 中调用
  *     RX_IsDone()             : 接收完成
  *     RX_GetMessage()         : 获取解码后的消息字符串
  ******************************************************************************
  */

#ifndef __RECEIVER_H
#define __RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "network_protocol.h"

/* ========================================================================== */
/*  帧结构常量 (与发送端 fsk4_encoder.c 一致)                                  */
/* ========================================================================== */

#define RX_MAX_CHARS          48
#define RX_SYMBOLS_PER_CHAR   4
#define RX_CHECKSUM_SYMBOLS   4
#define RX_HEADER_SYMBOLS     NET_HEADER_SYMBOLS
#define RX_MESSAGE_SYMBOLS    (RX_MAX_CHARS * RX_SYMBOLS_PER_CHAR)
#define RX_TOTAL_SYMBOLS      (RX_HEADER_SYMBOLS + RX_MESSAGE_SYMBOLS + RX_CHECKSUM_SYMBOLS)
#define RX_CHARSET_SIZE       75  /* 含 '$' 终止符 + \n */
#define VISIBLE_ROWS           7
#define DISP_COLS              21

/* ========================================================================== */
/*  DMA / 采样常量                                                             */
/* ========================================================================== */

#define RX_DMA_BUF_SIZE       800
#define RX_BLOCK_SIZE         400
#define RX_SAMPLE_RATE        16000

/* ========================================================================== */
/*  包络检波常量 (DPLL 下降沿同步)                                             */
/* ========================================================================== */

#define RX_ENV_BLOCK_SIZE     80
#define RX_ENV_TONE_BLOCKS    4
#define RX_ENV_GUARD_BLOCKS   2
#define RX_ENV_HISTORY_SIZE   8
#define RX_SUBBLOCKS_PER_HALF (RX_BLOCK_SIZE / RX_ENV_BLOCK_SIZE)  /* 5 */
#define RX_ENV_ENERGY_MIN_THRESH 25000U
#define RX_ENV_NOISE_FLOOR_INIT  1200U
#define RX_ENV_NOISE_IIR_SHIFT   4U
#define RX_ENV_NOISE_MULTIPLIER  4U
#define RX_ENV_NOISE_MARGIN      4000U
#define RX_ENV_ENERGY_MAX_THRESH 60000U
#define RX_ENV_STARTUP_QUIET_BLOCKS  40U
#define RX_ENV_STARTUP_REJECT_THRESH 60000U
#define RX_PREAMBLE_PILOT_CHECK_BLOCKS 4U
#define RX_PREAMBLE_MIN_TRANSITIONS    1U

#define RX_ENV_PREAMBLE_MIN_HI   6
#define RX_ENV_PREAMBLE_TIMEOUT  60
#define RX_ENV_FALLING_EDGE_MASK  0x07
#define RX_ENV_FALLING_EDGE_PAT   0x06
#define RX_ENV_HOLDOFF_BLOCKS    4
#define RX_ENV_DATA_NOISE_MAX    8
#define RX_ENV_DATA_SYNC_LOST    40

#define RX_PILOT_LO  0
#define RX_PILOT_HI  3

/* ========================================================================== */
/*  接收器状态                                                                 */
/* ========================================================================== */

#define RX_STATE_IDLE          0
#define RX_STATE_LISTENING     1
#define RX_STATE_PREAMBLE      2
#define RX_STATE_DATA          3
#define RX_STATE_DONE          4
#define RX_STATE_ERROR         5

/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

void      RX_Init(void);
void      RX_Start(void);
void      RX_Stop(void);
void      RX_ProcessHalfBuffer(const uint16_t *buf);

uint8_t   RX_IsBusy(void);
uint8_t   RX_IsFrameActive(void);
uint32_t  RX_GetNoiseFloor(void);
uint32_t  RX_GetEnergyThreshold(void);
uint8_t   RX_IsDone(void);
uint8_t   RX_GetState(void);
uint16_t  RX_GetSymbolCount(void);
void      RX_ClearDone(void);
const char* RX_GetStateName(void);

const char* RX_GetMessage(void);
uint8_t     RX_GetMessageLength(void);
uint8_t     RX_GetSourceId(void);
uint16_t    RX_GetTargetMask(void);
uint8_t     RX_GetDisplaySourceId(void);

void      RX_GetLastSymbol(uint8_t *digit, float *mag);

/* ---- v5.1 诊断 getter (供 LiveWatch 实时监控) ---- */
uint8_t  RX_GetPilotHits(void);   /* 前导命中计数 (0..VD_PILOT_HITS_REQ) */
uint8_t  RX_GetEraseRun(void);    /* 数据段连续擦除计数 (0..VD_MAX_ERASE_RUN, saturates) */
float    RX_GetLastSNR(void);     /* 最近一个符号的线性 True SNR */
uint8_t  RX_GetVGain(void);       /* PGA112 当前增益码 (0..7, 1x..128x) */
uint16_t RX_GetAGCVpp(void);      /* 最近 AGC 块的 ADC 峰峰值 (counts) */
uint32_t RX_GetPGAErrorCount(void); /* SPI 传输错误次数 */
uint8_t  RX_GetDspSubState(void); /* voice_dsp 内部子状态: 0.LISTEN 1.PRE 2.DATA 3.DONE */

const char* RX_GetDisplayMessage(void);
uint8_t     RX_GetDisplayLength(void);
uint8_t     RX_GetScrollLine(void);
void        RX_ScrollUp(void);
void        RX_ScrollDown(uint8_t total_lines);
void        RX_ScrollWrapUp(uint8_t total_lines);
const char* RX_GetStatusString(void);

uint8_t   RX_CharToIndex(char ch);
char      RX_IndexToChar(uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* __RECEIVER_H */
