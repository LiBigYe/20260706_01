/**
  ******************************************************************************
  * @file           : pwm_dds.h
  * @brief          : PWM-based DDS Sine Generator — 声语信使项目 (v4)
  *
  *   Hardware:
  *     TIM1_CH1: PA8, PWM output, ARR=1023 (10-bit, 48.83kHz carrier)
  *     RC filter: 1kΩ + 47nF → fc≈3.4kHz → smooth sine
  *
  *   DDS:
  *     1024-point × 10-bit sine LUT (uint16_t), 32-bit phase accumulator
  *     Sample rate = 16 kHz (TIM3 ISR), 62.5us per tick
  *     Frequency switch = change phase_inc → zero latency
  *     4 frequencies: 1500, 2000, 2500, 3000 Hz (4-FSK)
  *
  *   Output chain:
  *     DDS phase_acc → LUT[(phase_acc>>22) & 0x3FF] → TIM1->CCR1
  *     LUT 值直接写入 CCR1 (10-bit 原生, 中心512, 振幅510)
  *     → 10-bit 分辨率 = 1024 电压等级, Vpp ≈ 3.287V
  *
  *   v4 关键设计: OutputMidscale() 在 guard 期间输出 CCR1=512 (1.65V DC),
  *   形成 10ms 交流能量真空期. 接收端 v4 DPLL 利用此真空期
  *   检测下降沿 [HI,HI,LO], 实现物理层符号定时恢复.
  ******************************************************************************
  */

#ifndef __PWM_DDS_H
#define __PWM_DDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================== */
/*  常量                                                                       */
/* ========================================================================== */

#define PWM_DDS_LUT_SIZE      1024   /* 正弦查找表点数 (10-bit 相位分辨率) */
#define PWM_DDS_SAMPLE_RATE   16000  /* DDS 采样率 Hz (TIM3 16kHz) */
#define PWM_DDS_PHASE_BITS    32     /* 相位累加器宽度 */
#define PWM_DDS_ARR           1023   /* TIM1 自动重装载 (10-bit PWM, 1024 levels) */
#define PWM_DDS_MIDSCALE      512    /* DC 中值 CCR = ARR/2 (50% 占空比, 1.65V) */

/* ========================================================================== */
/*  API                                                                        */
/* ========================================================================== */

/**
  * @brief  初始化 PWM DDS
  * @param  htim_pwm:  TIM1 handle (PWM 输出, PA8)
  *
  *  启动 TIM1 CH1 PWM 输出 (48.83kHz).
  *  相位累加器归零, 初始频率 = 1500 Hz (digit 0).
  */
void PWM_DDS_Init(TIM_HandleTypeDef *htim_pwm);

/**
  * @brief  设置输出频率 (通过 digit 0~3)
  * @param  digit: 0 → 1500Hz, 1 → 2000Hz, 2 → 2500Hz, 3 → 3000Hz
  */
void PWM_DDS_SetFreq(uint8_t digit);

/**
  * @brief  DDS 单步: 更新 TIM1->CCR1 到下一个正弦采样值
  *
  *  由 TIM3 ISR (16kHz) 调用.
  *  内部: phase_acc += phase_inc → LUT[(phase_acc>>22) & 0x3FF] → CCR1.
  *
  *  LUT 为 10-bit 原生值 (中心512, 振幅510), 直接写入 CCR1, 无需缩放。
  */
void PWM_DDS_Tick(void);

/**
  * @brief  输出 DC 中值 (保护间隔用)
  *
  *  立即写 CCR1 = 512, 50% 占空比 → 经 RC 滤波 = 1.65V DC。
  */
 void PWM_DDS_OutputMidscale(void);

/**
  * @brief  关闭 PWM 输出 (传输结束)
  */
void PWM_DDS_Shutdown(void);

/**
  * @brief  重新启动 PWM 输出 (用于后续传输)
  */
void PWM_DDS_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_DDS_H */