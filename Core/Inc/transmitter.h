/**
  ******************************************************************************
  * @file           : transmitter.h
  * @brief          : 4-FSK 发送状态机 — 声语信使项目 (PWM+DDS 版, v4)
  *
  *   状态转换:
  *     IDLE → PREAMBLE(200ms) → DATA(12地址+192正文符号) → CHECKSUM(4符号)
  *          → POSTAMBLE(200ms) → DONE
  *
  *   时序 (16 kHz tick):
  *     符号 = 20 ms tone + 10 ms guard = 480 ticks
  *     帧  = 前导 + N×480 + 结束
  *
  *   v4 关键设计: 10ms guard 输出 DC 1.65V (PWM_DDS_OutputMidscale),
  *   形成交流能量真空期. 接收端 v4 DPLL 利用 5ms 切片
  *   检测下降沿 [HI,HI,LO] 实现物理层符号定时恢复.
  *   门限 25000 免疫 PWM 48.83kHz 载波纹波穿透.
  *
  *   由 TIM3 ISR (16kHz) 驱动: TX_Tick() 每 62.5 us 调用一次。
  *
  *   用法:
  *     TX_Start(text, source_id, target_mask): 编码地址与正文并发送
  *     TX_IsBusy()      : 发送中 → 主循环暂停键盘/编辑器
  *     TX_IsDone()      : 刚完成 → 显示结果后 TX_ClearDone()
  ******************************************************************************
  */

#ifndef __TRANSMITTER_H
#define __TRANSMITTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fsk4_encoder.h"

/* ========================================================================== */
/*  发送状态                                                                   */
/* ========================================================================== */

#define TX_STATE_IDLE        0
#define TX_STATE_PREAMBLE    1
#define TX_STATE_SYNC        2
#define TX_STATE_DATA        3
#define TX_STATE_POSTAMBLE   4
#define TX_STATE_DONE        5
/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

void      TX_Init(void);
void      TX_Start(const char *text, uint8_t source_id, uint16_t target_mask);
void      TX_Tick(void);           /* 由 TIM3 ISR 调用 */
uint8_t   TX_IsBusy(void);
uint8_t   TX_IsDone(void);
void      TX_ClearDone(void);
/* ── v5.1 ACK ── */
void      TX_SendAck(void);
uint8_t   TX_IsAckDone(void);

const char* TX_GetStateName(void);

#ifdef __cplusplus
}
#endif

#endif /* __TRANSMITTER_H */

