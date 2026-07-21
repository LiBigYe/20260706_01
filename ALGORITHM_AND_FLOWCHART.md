# 声语信使 半双工终端 — 单片机程序算法说明及程序流程图

> **MCU**: STM32F411CEU6 (HSE 25MHz → PLL 50MHz SYSCLK)
> **协议版本**: v5.1 (变长帧 + Hamming(7,4) FEC + 交织 + CRC-8 + 软判决 Chase 解码)
> **文档日期**: 2026-07-22

---

## 目录

1. [系统总体架构](#1-系统总体架构)
2. [半双工顶层状态机](#2-半双工顶层状态机)
3. [发送链路算法](#3-发送链路算法)
4. [接收链路算法](#4-接收链路算法)
5. [DSP 核心算法](#5-dsp-核心算法)
6. [FEC 纠错编解码算法](#6-fec-纠错编解码算法)
7. [PWM DDS 音频合成](#7-pwm-dds-音频合成)
8. [外设驱动与数据流](#8-外设驱动与数据流)
9. [Flash 非易失存储](#9-flash-非易失存储)
10. [程序流程图生成 Prompt](#10-程序流程图生成-prompt)

---

## 1. 系统总体架构

### 1.1 硬件资源分配

| 外设 | 用途 | 参数 |
|------|------|------|
| **TIM1_CH1 (PA8)** | PWM DDS 音频输出 | PSC=0, ARR=1023, 48.83kHz 载波, 1024-pt × 10-bit 正弦 LUT |
| **TIM2 (TRGO)** | ADC 触发源 | PSC=4, ARR=624, 50MHz→16kHz 触发 |
| **TIM3 (ISR)** | DDS 16kHz 采样时钟 + TX 状态机驱动 | PSC=4, ARR=624, 16kHz 中断 |
| **ADC1_IN8 (PB0)** | 音频采样 | 12-bit, TIM2 TRGO 触发, 28 周期采样 |
| **DMA2_Stream0** | ADC→缓冲区循环传输 | 800 halfwords, 双缓冲 (HT/TC) |
| **I2C1 (PB6/PB7)** | OLED 128×64 | 400kHz |
| **SPI1 (PA5/PA6/PA7)** | 外部 Flash PY25Q64HA | 8 MiB, PA4 为 CS |
| **GPIOA[0:3]** | 键盘行扫描 (Output PP) | PA0 兼 WKUP 唤醒 |
| **GPIOA[9:12]** | 键盘列读取 (Input PU) | 4×4 矩阵 |
| **PB2 (LEDG)** | 接收数据指示灯 | 进入数据段点亮 |
| **PB10 (LEDR)** | 发送指示灯 | 发送期间点亮 |
| **PB1 (EXTI1)** | 软开关按键 | 下降沿中断→关机 |
| **PB8 (POWER_CTRL)** | 电源锁存 | High=保持供电 |

### 1.2 软件模块分层

```
┌─────────────────────────────────────────────────────┐
│  应用层 (main.c)                                      │
│  ┌──────────┬──────────┬──────────┬───────────────┐  │
│  │ HM 状态机 │  OLED UI │ 键盘扫描  │ 设备编号选择  │  │
│  └──────────┴──────────┴──────────┴───────────────┘  │
├─────────────────────────────────────────────────────┤
│  发送链路              │  接收链路                    │
│  ┌──────────┐         │  ┌──────────┐               │
│  │ editor.c │ T9编辑  │  │receiver.c│ 地址过滤/缓冲 │
│  └────┬─────┘         │  └────┬─────┘               │
│  ┌────┴──────────┐    │  ┌────┴──────────┐          │
│  │ transmitter.c │    │  │ voice_dsp.c   │ Goertzel │
│  │  TX状态机      │    │  │ 接收DSP状态机  │ 差分能量 │
│  └────┬──────────┘    │  └────┬──────────┘          │
│  ┌────┴──────────┐    │  ┌────┴──────────┐          │
│  │ voice_fec.c   │    │  │ voice_fec.c   │ 软判决   │
│  │ FEC编码+交织   │    │  │ FEC解码+反交织│ Chase    │
│  └────┬──────────┘    │  └───────────────┘          │
│  ┌────┴──────────┐    │                             │
│  │ pwm_dds.c     │    │                             │
│  │ DDS正弦合成    │    │                             │
│  └───────────────┘    │                             │
├─────────────────────────────────────────────────────┤
│  存储层                                               │
│  ┌──────────────┐    ┌──────────────┐               │
│  │ flash_store.c│    │  py25q64.c   │               │
│  │ 消息管理      │    │  SPI Flash驱动│               │
│  └──────────────┘    └──────────────┘               │
├─────────────────────────────────────────────────────┤
│  HAL 层 (STM32F4xx_HAL)                              │
│  ADC / TIM / DMA / I2C / SPI / GPIO / EXTI           │
└─────────────────────────────────────────────────────┘
```

---

## 2. 半双工顶层状态机

### 2.1 状态定义

```
HM_RX       = 0  → 默认接收模式 (含3个子模式)
HM_TX_EDIT  = 1  → 发送编辑模式 (T9输入)
HM_TX_SELECT= 2  → 收件人多选模式
HM_TX_BUSY  = 3  → 发送进行中
```

### 2.2 RX 子模式

```
LS_LISTENING    = 0  → 实时监听, 显示最近收到的消息
LS_BROWSE_LIST  = 1  → 浏览已存储消息列表 (最多64条)
LS_BROWSE_VIEW  = 2  → 查看单条存储消息详情
```

### 2.3 状态转换逻辑

```
上电 → 中文启动页(2s) → 选择设备ID(1-9) → HM_RX(LS_LISTENING)
                                                 │
                    ┌────────────────────────────┼────────────────────────────┐
                    ▼                            ▼                            ▼
            [KEY_FN]                       [KEY_SEND]                   [数字键 0-9]
         切换到 HM_TX_EDIT              进入 LS_BROWSE_LIST           切换到 HM_TX_EDIT
                    │                            │                            │
                    │                     ┌──────┴──────┐                     │
                    │                     ▼             ▼                     │
                    │              [← →] 选择    [SEND] 查看                  │
                    │                     │    LS_BROWSE_VIEW                 │
                    │                     │         │                         │
                    │                     │    [← →] 滚动                     │
                    │                     │    [DEL] 返回列表                  │
                    │                     │    [KEY_FN] 返回监听               │
                    │                     └──────┬──────┘                     │
                    │                            │                            │
                    ▼                            ▼                            ▼
              HM_TX_EDIT ────────────────────────────────────────────────┐
                    │                                                     │
                    ├─ [SEND] (消息非空) → HM_TX_SELECT (选收件人)        │
                    │       │                                             │
                    │       ├─ [1-9] 切换目标                              │
                    │       ├─ [0] 广播                                   │
                    │       ├─ [DEL/KEY_FN] 返回编辑                       │
                    │       └─ [SEND] → HM_TX_BUSY → 发送完成 → HM_TX_EDIT│
                    │                                                     │
                    ├─ [KEY_FN] → HM_RX (保留编辑器内容)                   │
                    │                                                     │
                    ├─ 输入 "rx" + [→] → HM_RX (快速切回收信)              │
                    │                                                     │
                    └─ [KEY_POWER] → Standby (任何状态)                    │
```

### 2.4 模式切换时的外设互斥

| 操作 | ADC+DMA+TIM2 | TIM1 PWM | TIM3 ISR |
|------|:-----------:|:--------:|:--------:|
| HM_RX 启动 | **启动** | 停止 | 运行 (仅ISR空闲) |
| HM_TX_EDIT 进入 | **停止** | 停止 | 运行 |
| HM_TX_BUSY 发送 | **停止** | **启动** | **启动** (TX_Tick驱动) |
| HM_RX 恢复 | **启动** | **停止** | 运行 |

> **关键原则**: ADC+DMA 与 PWM 输出绝不共存; TIM3 在非发送期间仅保持中断使能但不驱动任何外设。

---

## 3. 发送链路算法

### 3.1 整体流程

```
用户T9输入 → Editor缓冲区 → TX_Start() → VoiceFEC编码 → TX_Tick()状态机 → PWM_DDS_Tick() → TIM1_CH1 → RC低通 → 扬声器
```

### 3.2 TX_Start() — 发送启动

1. **校验 source_id** (1~9), 不合法则默认 1
2. **构建 payload**: `[source_id(1B)][mask_lo(1B)][mask_hi(1B)][text...]` (变长, 无填充)
3. **FEC 编码**: 调用 `VoiceFEC_BuildDataSymbols(payload, plen, tx_symbols)`
   - 返回符号数组 `tx_symbols[]` 和符号总数 `tx_sym_count`
4. **设置初始状态**: `ST_PREAMBLE`, 输出 1500Hz 载波
5. **点亮 LEDR, 启动 TIM3 ISR**

### 3.3 TX_Tick() — 发送状态机 (由 TIM3 ISR 每 62.5μs 调用)

```
状态: ST_PREAMBLE → ST_SYNC → ST_DATA → ST_DATA_TAIL → ST_POSTAMBLE → ST_DONE

ST_PREAMBLE (2000ms):
  每 40ms (640 ticks) 交替输出 1500Hz / 2400Hz
  前导完成 → ST_SYNC, 输出 1800Hz

ST_SYNC (30ms):
  前 20ms: 1800Hz 载波 (同步音)
  后 10ms: DC midscale (guard)
  完成 → ST_DATA, sym_idx=0

ST_DATA (变长):
  每符号 30ms 槽:
    前 20ms: PWM_DDS_SetFreq(tx_symbols[sym_idx])
    后 10ms: DC midscale (guard)
  所有符号输出完 → ST_DATA_TAIL

ST_DATA_TAIL (30ms):
  输出 DC midscale (隔离槽, 防止时钟偏差污染 CRC)
  完成 → ST_POSTAMBLE, 输出 2400Hz

ST_POSTAMBLE (120ms):
  连续 2400Hz
  完成 → ST_DONE, 熄灭 LEDR, 输出 DC midscale, 停止 TIM3 ISR
```

### 3.4 Pulse-per-symbol 时序图

```
每符号槽 (30ms = 480 samples @16kHz):
  ┌──────────────┬──────────────┐
  │  20ms Tone   │  10ms Guard  │
  │  320 samples │  160 samples │
  │  f₀/f₁/f₂/f₃ │  DC=1.65V    │
  └──────────────┴──────────────┘
```

---

## 4. 接收链路算法

### 4.1 整体数据流

```
麦克风 → 模拟前端 → ADC1_IN8(PB0) → DMA2_Stream0 → adc_dma_buf[800]
                                                          │
                                          ┌───────────────┴───────────────┐
                                          ▼                               ▼
                                   HT中断 (前400)                  TC中断 (后400)
                                   RX_ProcessHalfBuffer(buf)      RX_ProcessHalfBuffer(buf+400)
                                          │                               │
                                          └───────────┬───────────────────┘
                                                      ▼
                                          以 80-sample 块为单位推进
                                                      │
                                          ┌───────────┴───────────┐
                                          ▼                       ▼
                              VoiceDSP: bandpass_sample()   VoiceDSP: DiffEnergy()
                              1.1-2.8kHz 带通滤波            一阶差分能量(高通)
                                          │                       │
                                          ▼                       ▼
                                   VoiceRx_PushBlock()     唤醒门限判断
                                   (VD_LISTEN→VD_PREAMBLE→VD_DATA→VD_DONE)
                                          │
                                          ▼
                              VoiceDSP_ClassifyMulti() 多窗口Goertzel判决
                                          │
                                          ▼
                              VoiceFEC_ParseDataSymbolsSoft() 软判决Chase解码
                                          │
                                          ▼
                              rx_on_frame_done() 地址过滤 + 显示缓冲 + Flash保存
```

### 4.2 RX_ProcessHalfBuffer() — DMA 半缓冲处理

```
每 400 samples = 5 个 80-sample 块:
  for i = 0..4:
    VoiceRx_PushBlock(&vrx, sub)  // 送入DSP状态机
    if vrx.state 从非DATA变为DATA: 点亮 LEDG/LEDR 双灯
    if PushBlock返回1 (VD_DONE): rx_on_frame_done(); return
  rx_sync_state()  // 同步旧状态字
```

### 4.3 rx_on_frame_done() — 帧完成处理

```
1. 熄灭 LEDR
2. 校验 vrx.crc_ok (CRC-8 通过)
3. 提取 payload: [source_id, mask_lo, mask_hi, text...]
4. 地址过滤:
   source_id 合法(1-9) 且 (广播 || target_mask 含本机ID)
   → 是: 设置 rx_done_flag=1, 复制显示缓冲, 点亮 LED
   → 否: rx_restart_listening() 丢弃帧
5. main.c 检测到 RX_IsDone() → 自动保存Flash → 显示2s → 重启采样
```

### 4.4 地址过滤

```
NET_IsAddressedTo(target_mask, g_device_id):
  return (target_mask == NET_BROADCAST_MASK) ||      // 广播
         (target_mask & NET_DeviceMask(g_device_id))  // 单播/组播
```

---

## 5. DSP 核心算法

### 5.1 VoiceRx_PushBlock() — DSP 状态机（以 80-sample 块驱动）

#### 状态: VD_LISTEN (监听/噪声标定)

```
每块:
1. 带通滤波: bandpass_sample(adc_sample) → vd_filtered_blk[80]
   级联: 1个二阶高通 + 3个二阶低通 = 8阶 1.1-2.8kHz 带通
2. 差分能量: VoiceDSP_DiffEnergy(vd_filtered_blk) = sum|x[i]-x[i-1]|
3. 噪声标定 (前15块 ~75ms):
   快速 IIR: noise_floor = (noise_floor*3 + energy)/4
   禁止前导锁定 (hi=0 强制)
4. 标定完成后:
   慢速 IIR: noise_floor = (noise_floor*15 + energy)/16 (仅低能量时)
   门限 = noise_floor + VD_EN_MARGIN(24), 不低于 VD_EN_MIN(32)
5. 每 160-sample 窗做 Goertzel 分类:
   检测 1500Hz / 2400Hz 导频
   统计: pilot_hits(连续命中), pilot_trans(交替次数)
   条件: pilot_hits≥2 && pilot_trans≥8 && pilot_gap≤6
   → 进入 VD_PREAMBLE
```

#### 状态: VD_PREAMBLE (前导确认/同步音搜索)

```
每 160-sample 窗:
1. Goertzel 分类, 查找 1800Hz (VP_SYNC_DIGIT=1)
2. 连续命中 sync_hits≥2:
   回扫同步音上升沿 (sync_find_onset):
     从当前时刻倒退 400 samples, 步长 8 samples
     每次取 80-sample 窗做 Goertzel(k=9, 对应 1800Hz)
     找到 mag² ≥ peak*0.8 的第一个位置 → tone_start
   锁定栅格: vd_grid_start = tone_start + VP_SLOT_SAMPLES(480)
   进入 VD_DATA
3. 超时: block_in_pre > 380 → 退回 VD_LISTEN
```

**同步音回扫原理**: 1800Hz 同步音在前导中从不出现，一旦 Goertzel 连续检测到 1800Hz，说明已进入同步音区间。回扫找到其能量从低到高的上升沿（前一个 guard→tone 的边界），作为符号栅格的绝对零时刻。

#### 状态: VD_DATA (数据解调)

```
对每 80-sample 块的每个样本:
  abspos = 累计样本数
  if abspos < vd_grid_start: 跳过 (尚未到数据区)
  rel = abspos - vd_grid_start
  sym = rel / 480    // 符号编号
  pos = rel % 480    // 符号内位置
  if pos >= 320: 跳过 (guard 区间)
  填充 win_buf[pos] = vd_filtered_blk[i]

  if 填满 320 samples (完整 20ms tone):
    data_store_symbol(rx):
      1. VoiceDSP_ClassifyMulti(win_buf, 320, snr_threshold):
         3个重叠窗口 (offset=40,80,120, 各160点):
           每个窗口做 Goertzel 4频 (±1 bin 频带能量累加)
           累加 3 窗的 mag²
         频响补偿: acc_mag2[i] *= vd_freq_weight[i]
         找最强频点 → digit
         True SNR = p_signal/p_noise
         判决: SNR≥2.0 或 频谱占比≥1.35 → 输出 digit, 否则擦除(0xFF)
      2. 保存 mag²[4] 到 sym_mag2[][] 供软判决
      3. 擦除计数: erase_run (仅诊断, 不中断帧)
      4. 收满 LEN 前缀(21符号): VoiceFEC_DecodeLen → 求 sym_expected
      5. 收满 sym_expected: VoiceFEC_ParseDataSymbolsSoft → VD_DONE
```

### 5.2 VoiceDSP_ClassifyMulti() — 多窗口 Goertzel 判决

```
输入: tone[320 samples], snr_threshold=2.0

算法:
for w=0..2 (3个重叠窗口):
  offset ∈ {40, 80, 120}
  取子窗口 sub = tone[offset..offset+159]  (160 samples)
  
  去DC: dc = mean(sub[0..159])
  
  for f=0..3 (4个频点):
    k ∈ {14,15,16} for 1500Hz  // 三bin累加
    k ∈ {17,18,19} for 1800Hz
    k ∈ {20,21,22} for 2100Hz
    k ∈ {23,24,25} for 2400Hz
    acc_mag2[f] += goertzel_mag2(sub, 160, k, dc)  // ±1 bin 容错

频响补偿: wgt[f] = acc_mag2[f] * vd_freq_weight[f]
  // {1.33, 1.08, 1.00, 1.02} — 补偿1500Hz声学增益偏低

找最强和次强: best/freq_ratio = bestv/secondv

True SNR:
  p_signal = acc_mag2[best] * (2/160)   // 信号功率
  p_noise = p_total - p_signal           // 噪声功率
  snr = p_signal/p_noise

判决:
  if snr < 2.0 AND freq_ratio < 1.35: return 0xFF (擦除)
  else: return best (0/1/2/3)
```

### 5.3 Goertzel 算法原理

```
Goertzel 是单频点 DFT 的高效递推实现:

递推 (N次):
  q0 = coeff * q1 - q2 + x[i]
  q2 = q1; q1 = q0

终值 (一次):
  mag² = q1² + q2² - q1 * q2 * coeff

其中: coeff = 2*cos(2π*k/N)
      k = f_target * N / f_sample   (整数, 无频谱泄露)

对于 N=160, f_sample=16000:
  1500Hz → k=15 (整数✓)
  1800Hz → k=18 (整数✓)
  2100Hz → k=21 (整数✓)
  2400Hz → k=24 (整数✓)

±1 bin 容错: 对 k-1, k, k+1 分别计算 mag² 并累加
 → 容忍 ±(16000/160)=±100Hz 频偏
```

### 5.4 差分能量检测 (抗 50Hz/DC)

```
VoiceDSP_DiffEnergy(blk[80]):
  return sum{|blk[i] - blk[i-1]|}  for i=1..79

一阶差分等效高通滤波器, 对:
  DC 偏置 (0Hz): 输出 ≈ 0 ✓
  50Hz 工频: 大幅衰减 ✓
  1-3kHz 音频: 保留 ✓
```

### 5.5 带通滤波器 (级联 Biquad)

```
1 个二阶高通 + 3 个二阶低通 = 8 阶 IIR 带通 (1.1 - 2.8 kHz)

每样本处理:
  centered = adc_sample - 2048.0f  (去DC中点)
  filtered = biquad_push(&vd_bp_high, centered)    // 高通
  filtered = biquad_push(&vd_bp_low_1, filtered)   // 低通×3
  filtered = biquad_push(&vd_bp_low_2, filtered)
  filtered = biquad_push(&vd_bp_low_3, filtered)

Biquad 递推 (Direct Form I):
  output = b0*input + z1
  z1 = b1*input - a1*output + z2
  z2 = b2*input - a2*output

首次样本时预置高通状态, 避免非 2048 偏置产生伪瞬态:
  vd_bp_high.z1 = -b0 * centered
  vd_bp_high.z2 =  b2 * centered
```

---

## 6. FEC 纠错编解码算法

### 6.1 编码链: VoiceFEC_BuildDataSymbols()

```
输入: payload[0..plen-1], plen ≤ 51
输出: out_syms[]

步骤:
┌──────────────────────────────────────────────────────────────┐
│ 1. LEN 前缀 (三重冗余)                                        │
│    payload_len → 分2个nibble → 各Hamming(7,4) → 2码字(各7bit) │
│    → 块交织(14 bit) → 7符号                                   │
│    重复 3 次 → 共 21 符号                                     │
├──────────────────────────────────────────────────────────────┤
│ 2. Body = [payload...][CRC]                                   │
│    CRC = VoiceFEC_Crc8([payload_len][payload...])             │
│    每字节 → 2 nibble → 2 Hamming码字                          │
│    全部码字比特做块交织 (行写列读)                              │
│    每 2 bit → 1 个 4-FSK 符号                                 │
└──────────────────────────────────────────────────────────────┘
```

#### Hamming(7,4) 编码

```
输入: 4-bit nibble (d1,d2,d3,d4)
校验位:
  p1 = d1 ⊕ d2 ⊕ d4
  p2 = d1 ⊕ d3 ⊕ d4
  p3 = d2 ⊕ d3 ⊕ d4
输出: 7-bit codeword = [p1, p2, d1, p3, d2, d3, d4]
```

#### 块交织

```
假设有 ncw 个码字, 每个 7 bit:
  原始排列: cw[0][b0..b6], cw[1][b0..b6], ...
  交织后:    cw[0][b0], cw[1][b0], ..., cw[0][b1], cw[1][b1], ...
            (列优先读出)

效果: 连续的 4-FSK 符号错误 → 分散到不同码字的单个 bit
      → Hamming 可纠正单 bit 错误
```

#### 4-FSK 符号映射

```
每 2 bit → 1 符号:
  00 → 1500Hz (digit 0)
  01 → 1800Hz (digit 1)
  10 → 2100Hz (digit 2)
  11 → 2400Hz (digit 3)
```

### 6.2 解码链: VoiceFEC_ParseDataSymbolsSoft() (v5.1 软判决)

```
输入: syms[], mag2[][4], sym_count
输出: out_payload[], out_len

步骤:
┌──────────────────────────────────────────────────────────────┐
│ 1. LEN 解码 (三重冗余多数表决, 硬判决)                         │
│    3 份各7符号 → 反交织 → Hamming解码 → 2 nibble → 1 字节     │
│    3 字节逐 bit 多数表决 → payload_len                        │
│    校验: 0 < payload_len ≤ 51                                 │
├──────────────────────────────────────────────────────────────┤
│ 2. Body 符号区 (LEN之后) 软判决 Chase 解码                     │
│    for each 字节 (2码字):                                     │
│      for each 码字 (7 bit):                                   │
│        a. LLR 计算: 每个 bit 来自 1 个 4-FSK 符号             │
│           符号的 4 频 mag² → MSB: E1=m2+m3, E0=m0+m1          │
│                            → LSB: E1=m1+m3, E0=m0+m2          │
│           llr = sign × ln(max(E1,E0)/min(E1,E0))  (LUT查表)  │
│                                                               │
│        b. Chase 软 Hamming 解码:                              │
│           - 硬判: hard[7] = llr≥0 ? 1 : 0                     │
│           - 找 2 个最不可靠 bit (|LLR| 最小)                   │
│           - 若 min|LLR| > 3.0: 直接返回硬判结果                │
│           - 否则试 4 种翻转组合, 每种做 Hamming 硬解码          │
│           - 取软距离 (Σ 翻转bit的|LLR|) 最小的候选              │
│                                                               │
│        c. 2个码字 → 2 nibble → 1 字节                         │
│                                                               │
├──────────────────────────────────────────────────────────────┤
│ 3. CRC-8 校验                                                 │
│    calc = CRC8([payload_len][payload[0..plen-1]])             │
│    calc == rx_crc ? 成功 : 失败                                │
└──────────────────────────────────────────────────────────────┘
```

#### LLR (对数似然比) 查表法

```
VoiceFEC_ComputeLLR(E1, E0):
  ratio = max(E1,E0) / min(E1,E0)   // [1, 100]
  sign = E1≥E0 ? +1 : -1
  
  // 归一化: ratio = mantissa × 2^octaves
  while ratio ≥ 2: ratio/=2, octaves++
  
  // 查表: ln(mantissa), mantissa ∈ [1, 2)
  index = (ratio - 1) × 16
  ln_mantissa = llr_mantissa_lut[index]
  
  return sign × (octaves × ln(2) + ln_mantissa)
  // 范围 ≈ ±4.6

4-FSK 符号 MSB = bit0:
  E1 = mag²[digit2] + mag²[digit3]  (bit0=1 的符号)
  E0 = mag²[digit0] + mag²[digit1]  (bit0=0 的符号)
  
4-FSK 符号 LSB = bit1:
  E1 = mag²[digit1] + mag²[digit3]  (bit1=1 的符号)
  E0 = mag²[digit0] + mag²[digit2]  (bit1=0 的符号)
```

#### CRC-8

```
多项式: 0x07 (x⁸ + x² + x + 1)
初值: 0x00
输入: [payload_len][payload[0..plen-1]]

算法:
  for each byte:
    crc ^= byte
    for 8 bits:
      if crc & 0x80: crc = (crc<<1) ^ 0x07
      else:          crc = crc<<1
```

---

## 7. PWM DDS 音频合成

### 7.1 DDS 原理

```
32-bit 相位累加器:
  phase_acc += phase_inc   (每 16kHz 时钟周期)

10-bit LUT 索引:
  idx = (phase_acc >> 22) & 0x3FF   // 取高10位, 0-1023

PWM 占空比:
  CCR1 = sine_lut[idx]   // 1024-pt, 10-bit 值 [2, 1022]

频率控制:
  phase_inc = (f_out / 16000) × 2^32

  1500Hz: phase_inc = (1500/16000) × 2^32 = 402653184
  1800Hz: phase_inc = (1800/16000) × 2^32 = 483183821
  2100Hz: phase_inc = (2100/16000) × 2^32 = 563714457
  2400Hz: phase_inc = (2400/16000) × 2^32 = 644245094
```

### 7.2 TIM1 PWM 参数

```
PSC = 0, ARR = 1023
PWM 频率 = 50MHz / (0+1) / (1023+1) = 48.83kHz
DDS 更新频率 = 16kHz (TIM3 触发)
每约 3 个 PWM 周期更新一次 CCR1

RC 低通: 1kΩ + 22nF → fc = 1/(2πRC) ≈ 7.2kHz
  48.83kHz 载波衰减 ≈ -16dB
```

### 7.3 正弦 LUT

```
1024 点 × 10-bit, 中心 = 512, 振幅 = 510
公式: LUT[i] = 512 + round(510 × sin(2πi/1024))
范围: [2, 1022]

DC 中点: CCR1=512 → 50% 占空比 → 1.65V DC
         (PWM_DDS_OutputMidscale 直接写寄存器)
```

---

## 8. 外设驱动与数据流

### 8.1 ADC + DMA 双缓冲

```
TIM2 TRGO @ 16kHz → ADC1 触发 → EOC → DMA2_Stream0 传输

缓冲: adc_dma_buf[800] = 50ms 数据
DMA 循环模式, 双缓冲:
  HT (Half Transfer): 前 400 samples 就绪 → HAL_ADC_ConvHalfCpltCallback
  TC (Transfer Complete): 后 400 samples 就绪 → HAL_ADC_ConvCpltCallback

回调中:
  if hm_mode == HM_RX:
    RX_ProcessHalfBuffer(adc_dma_buf)      // HT
    RX_ProcessHalfBuffer(adc_dma_buf+400)  // TC
```

### 8.2 TIM3 中断

```
TIM3 PSC=4, ARR=624 → 50MHz/5/625 = 16kHz

HAL_TIM_PeriodElapsedCallback:
  if TIM3: TX_Tick() → PWM_DDS_Tick()
  
  TX_Tick 仅在 tx_state != IDLE && != DONE 时执行
  非发送期间: ISR 触发但不驱动任何外设 (TX_Tick 立即返回)
```

### 8.3 键盘扫描

```
4×4 矩阵: PA0-3 行 (Output PP), PA9-12 列 (Input PU)

ScanMatrix():
  逐行拉低 → 读4列 → 有低电平则返回键码
  
Keyboard_Scan():
  去抖: 30ms 稳定期
  长按: 200ms 后每 150ms 重复

按键映射: 0-9数字, LEFT/RIGHT, DELETE, KEY_FN(英/数), KEY_SEND(发送)
           + KEY_POWER (独立 EXTI, PA0/WKUP 复用)
```

### 8.4 OLED 显示

```
SSD1306 128×64, I2C1 400kHz, 5×7 ASCII 字体

OLED_Refresh(): 全帧 Framebuffer → I2C 写入
OLED_Clear(): 清零 Framebuffer

低功耗: 监听空闲 20s → OLED_SetDisplay(0) (0xAE 关屏)
         任意按键/帧活动/接收完成 → OLED_SetDisplay(1) (0xAF 开屏+刷新)
```

### 8.5 软开关 (一键关机)

```
PB1 (POWER_BUTTON) 配置为 EXTI1 下降沿中断:
  HAL_GPIO_EXTI_Callback → g_power_off = 1 → Power_CutOff()
  
Power_CutOff(): GPIOB->BSRR = BR8  (拉低 POWER_CTRL → 切断供电)

PB8 (POWER_CTRL): 默认 High 锁存供电，拉低关机

上电初始化 (main开头, HAL初始化前):
  GPIOB 提前使能 → 检测 PB1 电平
  PB1=Low (按键按下): 锁存 PB8=High, 保持供电
  PB1=High (未按下): 锁存 PB8=Low, 等待掉电
  // 实现开机必须按住电源键
```

---

## 9. Flash 非易失存储

### 9.1 存储架构

```
SPI Flash: PY25Q64HA (8 MiB, 24-bit 地址)

双副本 A/B:
  Copy A: 0x000000 (4 KiB)
  Copy B: 0x001000 (4 KiB)

镜像格式 (StoreImage):
  magic:     0x564F4943  ("VOIC")
  version:   3
  count:     消息数 (0-64)
  generation: 单调递增代数
  crc32:     CRC32 校验
  slots[64]: 每条消息 (valid, source_id, length, data[50])

写入策略:
  交替写入非活动副本 → 擦除 → 写入 → 回读校验 → 切换 active_address
  写入失败 → 内存备份恢复

上电初始化:
  读取 A/B → 选最新有效镜像 → 都无效则创建空镜像
```

### 9.2 消息管理

```
FlashStore_SaveMessageFrom(source_id, msg, len):
  if count < 64: 追加到末尾
  else: memmove 淘汰最旧, 写入最后槽
  commit()

FlashStore_DeleteMessage(index):
  memmove 前移后续槽
  count--
  commit()

commit():
  target = 非活动副本
  image.generation++
  image.crc32 = image_crc(&image)
  PY25Q64_EraseSector(target)
  PY25Q64_Write(target, &image, sizeof(image))
  PY25Q64_Read(target, &verify, sizeof(verify))
  校验通过 → active_address = target
```

---

## 10. 程序流程图生成 Prompt

以下为 7 张核心流程图的 **DALL·E / Midjourney / 绘图工具 生成 Prompt**。
每段 Prompt 均为英文，描述了图的结构、节点、箭头和标注，可直接交付绘图。

---

### 10.1 系统总体流程图 (System Overview Flowchart)

> **建议工具**: Draw.io / Visio / PlantUML (手动绘制更精确)
> **以下是供 AI 绘图工具使用的描述性 Prompt**:

```
A professional technical flowchart showing the complete system architecture of 
a half-duplex voice messenger device based on STM32F411CEU6 microcontroller.

TOP SECTION - "Power-Up Sequence" (left to right, 6 rounded boxes connected by arrows):
Box 1: "Power Button Pressed\n(PB1=Low → PB8=High Latch)"
Box 2: "HAL & Clock Init\n(HSE 25MHz → PLL 50MHz)"
Box 3: "Peripheral Init\n(ADC/TIM/DMA/I2C/SPI/GPIO)"
Box 4: "OLED Startup Screen\n(Chinese splash 2s)"
Box 5: "Select Device ID\n(1-9 key + SEND confirm)"
Box 6: "Enter HM_RX\n(Default listen mode)"

MIDDLE SECTION - "Half-Duplex State Machine" (center, large diamond-and-box diagram):
Draw a central state diagram with 4 large rounded rectangles labeled:
- HM_RX (top-left, blue): "Receive Mode\nADC+DMA+TIM2 ON\nTIM1 PWM OFF"
- HM_TX_EDIT (top-right, green): "Edit Mode\nT9 Text Input\nADC OFF, PWM OFF"
- HM_TX_SELECT (right, yellow): "Select Recipients\n1-9 Toggle / 0=Broadcast"
- HM_TX_BUSY (bottom-right, red): "Transmitting\nTIM1 PWM ON\nTIM3 ISR Driving TX"

Draw directed arrows between states with transition conditions:
- HM_RX → HM_TX_EDIT: "KEY_FN or Digit Key"
- HM_TX_EDIT → HM_RX: "KEY_FN or 'rx'+RIGHT"
- HM_TX_EDIT → HM_TX_SELECT: "SEND (non-empty)"
- HM_TX_SELECT → HM_TX_EDIT: "DEL or KEY_FN"
- HM_TX_SELECT → HM_TX_BUSY: "SEND"
- HM_TX_BUSY → HM_TX_EDIT: "TX Done (auto)"
- Any state → Power Off: "KEY_POWER (EXTI1)"

BOTTOM SECTION - "RX Sub-Modes" (inside HM_RX box, 3 smaller boxes):
- LS_LISTENING: "Live Monitoring\nShow last received msg"
- LS_BROWSE_LIST: "Stored Message List\n(Up to 64, paginated)"
- LS_BROWSE_VIEW: "View Single Message\nScroll with LEFT/RIGHT"
Arrows: LISTENING ←→ BROWSE_LIST (KEY_SEND), BROWSE_LIST ←→ BROWSE_VIEW (SEND/DEL)

RIGHT SIDEBAR - "Peripheral Resource Ownership" (table format):
| Peripheral | HM_RX | HM_TX_EDIT | HM_TX_BUSY |
|------------|-------|------------|------------|
| ADC+DMA+TIM2 | ON | OFF | OFF |
| TIM1 PWM | OFF | OFF | ON |
| TIM3 ISR | IDLE | IDLE | TX_Tick |

Use clean technical diagram style, blue/white color scheme, 
12pt Arial labels, 2px arrows with arrowheads.
```

---

### 10.2 发送链路流程图 (TX Chain Flowchart)

```
A vertical flowchart showing the transmit signal processing chain 
of a 4-FSK voice messenger, from text input to speaker output.

TITLE at top: "TX Signal Chain — Editor → FEC → PWM DDS → Speaker"

LAYOUT: Vertical top-to-bottom flow with 6 main blocks connected by 
thick downward arrows. Each block has a header and sub-items.

BLOCK 1 - "T9 Text Editor" (light gray):
- "User types on 4×4 matrix keyboard"
- "T9 multi-tap: abc/ABC/123 modes"
- "Cursor movement, backspace"
- "Output: text buffer (≤48 chars)"

BLOCK 2 - "TX_Start(text, source_id, target_mask)" (light blue):
- "Build payload: [src_id][mask_lo][mask_hi][text...]"
- "Validates source_id ∈ [1,9]"
- "Calls VoiceFEC_BuildDataSymbols()"

BLOCK 3 - "FEC Encoding (voice_fec.c)" (light yellow):
Sub-blocks in sequence (left to right arrows):
a) "LEN byte → 3× redundant Hamming(7,4) encoding → 21 symbols"
b) "Each payload byte → 2 nibbles → 2 Hamming(7,4) codewords"
c) "CRC-8 computation over [LEN][payload]"
d) "All codewords → block interleaver (column-major readout)"
e) "Interleaved bits → 2-bit groups → 4-FSK symbols [0,1,2,3]"

BLOCK 4 - "TX_Tick() State Machine (TIM3 ISR @16kHz)" (light green):
State boxes in sequence with arrows:
- "ST_PREAMBLE (2000ms)\n  1500/2400Hz alternating every 40ms"
  ↓
- "ST_SYNC (30ms)\n  1800Hz tone 20ms + DC guard 10ms"
  ↓
- "ST_DATA (variable)\n  Each symbol: 20ms tone + 10ms guard\n  tx_symbols[idx] → PWM_DDS_SetFreq()"
  ↓
- "ST_DATA_TAIL (30ms)\n  DC midscale isolation"
  ↓
- "ST_POSTAMBLE (120ms)\n  2400Hz continuous"
  ↓
- "ST_DONE\n  LEDR OFF, TIM3 stop, DC midscale"

BLOCK 5 - "PWM DDS Synthesis (pwm_dds.c)" (light orange):
- "32-bit phase accumulator: phase_acc += phase_inc"
- "10-bit LUT index: idx = (phase_acc >> 22) & 0x3FF"
- "1024-point sine LUT [2..1022], center=512"
- "CCR1 = sine_lut[idx] → TIM1_CH1 PWM @ 48.83kHz"

BLOCK 6 - "Analog Output" (light red):
- "RC Low-pass: 1kΩ + 22nF (fc≈7.2kHz)"
- "Removes 48.83kHz PWM carrier"
- "→ Speaker / Audio jack"

RIGHT SIDEBAR - timing diagram annotation:
"Per-Symbol Timing: [20ms Tone | 10ms Guard] × N symbols\n
Total frame: 2000ms Preamble + 30ms Sync + N×30ms Data + 30ms Tail + 120ms Postamble\n
Worst case (48 chars): ~13.3s < 20s budget"

Use engineering diagram style, monospace font for code labels,
rounded rectangles with shadows, connecting lines with arrowheads.
```

---

### 10.3 接收链路流程图 (RX Chain Flowchart)

```
A vertical technical flowchart showing the complete receive signal processing 
chain of a 4-FSK voice messenger, from microphone input to decoded message display.

TITLE: "RX Signal Chain — ADC → Bandpass → DSP State Machine → FEC Decode → Display"

LAYOUT: Vertical flow with 7 main blocks.

BLOCK 1 - "Analog Front-End & ADC" (gray):
- "Electret mic + preamp → biased to 1.65V DC"
- "ADC1_IN8 (PB0), 12-bit, TIM2 TRGO trigger"
- "DMA2_Stream0: 800-halfword circular buffer"
- "Dual-buffer: HT(400) / TC(400) interrupts"

BLOCK 2 - "Bandpass Filter (1.1-2.8 kHz)" (blue):
- "1× 2nd-order high-pass + 3× 2nd-order low-pass (8th-order IIR)"
- "Each sample: centered = adc - 2048 → biquad chain"
- "Removes PWM residue, 50Hz hum, out-of-band noise"
- "Output: vd_filtered_blk[80] float samples"

BLOCK 3 - "VoiceDSP Receiver State Machine" (green):
Split into 4 sub-states with detailed processing:

SUB-STATE A - "VD_LISTEN (Noise Calibration)":
- "DiffEnergy = Σ|x[i]-x[i-1]| (high-pass, immune to DC/50Hz)"
- "First 15 blocks (~75ms): fast IIR noise floor calibration"
- "Then: slow IIR tracking, threshold = noise_floor + 24 (min 32)"
- "Every 160-sample window: Goertzel classify (4 tones ±1 bin)"
- "Count pilot_hits (1500/2400Hz) and pilot_trans (alternations)"
- "Condition: hits≥2 AND trans≥8 → VD_PREAMBLE"

SUB-STATE B - "VD_PREAMBLE (Sync Tone Search)":
- "Every 160-sample window: Goertzel detect 1800Hz"
- "2 consecutive hits → sync_find_onset():"
- "  Scan backward 400 samples, step=8"
- "  For each position: 80-sample Goertzel at bin k=9 (1800Hz)"
- "  Find first position with mag² ≥ 0.8×peak"
- "  → Lock symbol grid: grid_start = onset + 480"
- "→ VD_DATA"

SUB-STATE C - "VD_DATA (Multi-Window Demodulation)":
- "For each sample in data region:"
- "  Accumulate 320 samples per 20ms tone window"
- "  Skip guard samples (pos ≥ 320 in 480-sample slot)"
- "When tone window full: VoiceDSP_ClassifyMulti():"
- "  3 overlapping windows (offset 40/80/120, N=160)"
- "  Each window: Goertzel 4 tones, ±1 bin band energy"
- "  Accumulate mag² across 3 windows"
- "  Apply frequency response weights: {1.33, 1.08, 1.00, 1.02}"
- "  Compute True SNR = P_signal / P_noise"
- "  Decision: SNR≥2.0 OR freq_ratio≥1.35 → digit; else erasure(0xFF)"
- "  Save mag²[4] for soft-decision FEC"
- "After 21 symbols (LEN prefix): decode payload length"
- "When all symbols received: → VD_DONE"

SUB-STATE D - "VD_DONE":
- "Call VoiceFEC_ParseDataSymbolsSoft() for soft-decision decoding"
- "CRC-8 verification"

BLOCK 4 - "FEC Soft-Decision Decoding (voice_fec.c)" (yellow):
- "LEN decode: 3× redundant codewords → majority vote → payload_len"
- "For each byte (2 codewords × 7 bits):"
- "  Compute LLR for each bit from 4-FSK symbol mag²"
- "  LLR = sign × ln(max(E1,E0)/min(E1,E0)) via LUT"
- "  Chase decoder: find 2 least reliable bits"
- "  Try 4 flip patterns, pick minimum soft-distance candidate"
- "  Hamming(7,4) decode → nibble → byte"
- "CRC-8 check: computed vs received"

BLOCK 5 - "Address Filtering" (orange):
DIAMOND decision: "Is source_id ∈ [1,9]?"
→ YES: "Is target_mask broadcast OR contains my device_id?"
  → YES: "Accept frame → set rx_done_flag=1"
  → NO: "Discard, restart listening"
→ NO: "Discard, restart listening"

BLOCK 6 - "Message Processing" (pink):
- "Copy to display buffer (rx_display_msg)"
- "Auto-save to SPI Flash (FlashStore_SaveMessageFrom)"
- "Show 'Rx Complete' on OLED for 2 seconds"
- "Restart ADC + DMA + TIM2 sampling"

BLOCK 7 - "LED Indicators" (annotations):
- "LEDG (PB2): ON only during VD_DATA (valid data symbols)"
- "LEDR (PB10): OFF during RX (TX only)"
- "LED (PC13): OFF during RX listening, ON when DONE"

Use technical flowchart style with color-coded blocks,
detailed sub-steps in smaller text, diamond decision nodes,
and clear data flow arrows.
```

---

### 10.4 DSP Goertzel 算法流程图

```
A detailed algorithm flowchart for the Goertzel frequency detection 
used in a 4-FSK voice messenger receiver.

TITLE: "Goertzel Multi-Window 4-FSK Classifier (VoiceDSP_ClassifyMulti)"

INPUT: "tone[320] float samples, snr_threshold=2.0"

┌─────────────────────────────────────────────────────┐
│ FOR w = 0 TO 2 (3 windows):                         │
│   offset ∈ {40, 80, 120}                            │
│   sub = tone[offset .. offset+159]                  │
│                                                     │
│   ┌─ DC Removal ─────────────────────────────┐      │
│   │ dc = (Σ sub[i]) / 160                     │      │
│   │ For i=0..159: dev = sub[i] - dc           │      │
│   │ P_total += dev²                           │      │
│   └───────────────────────────────────────────┘      │
│                                                     │
│   ┌─ Goertzel per Frequency (f=0..3) ────────┐      │
│   │ For each target bin k (e.g., 1500Hz→15):  │      │
│   │   band_mag² =                              │      │
│   │     goertzel(k-1, sub, dc) +               │      │
│   │     goertzel(k,   sub, dc) +               │      │
│   │     goertzel(k+1, sub, dc)                 │      │
│   │   acc_mag2[f] += band_mag²                 │      │
│   │                                            │      │
│   │   goertzel(k, window, dc):                 │      │
│   │     coeff = 2·cos(2π·k/160)               │      │
│   │     q1=0, q2=0                             │      │
│   │     FOR i = 0 TO 159:                      │      │
│   │       q0 = coeff·q1 - q2 + (win[i]-dc)     │      │
│   │       q2 = q1; q1 = q0                     │      │
│   │     RETURN q1² + q2² - q1·q2·coeff         │      │
│   └───────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────┘

┌─ Frequency Response Compensation ──────────────────┐
│ wgt[f] = acc_mag2[f] × [1.33, 1.08, 1.00, 1.02]   │
└─────────────────────────────────────────────────────┘

┌─ Find Best & Second Best ──────────────────────────┐
│ best  = argmax(wgt[0..3])                           │
│ bestv = wgt[best]                                    │
│ secondv = max(wgt[i≠best])                           │
│ freq_ratio = bestv / max(secondv, 1e-12)            │
└─────────────────────────────────────────────────────┘

┌─ True SNR Computation ─────────────────────────────┐
│ p_signal = acc_mag2[best] × (2/160)                 │
│ p_noise  = max(P_total - p_signal, 1.0)             │
│ snr = p_signal / p_noise                            │
└─────────────────────────────────────────────────────┘

┌─ Decision ─────────────────DIAMOND─────────────────┐
│         snr ≥ 2.0 OR freq_ratio ≥ 1.35 ?           │
│           ↙ YES              ↘ NO                   │
│   RETURN best (0-3)    RETURN 0xFF (erasure)        │
└─────────────────────────────────────────────────────┘

BELOW the main flow, add a small annotation box:
"Frequency Map: 0=1500Hz 1=1800Hz 2=2100Hz 3=2400Hz"
"Bin Map (N=160, fs=16000, Δf=100Hz): 1500→k=15, 1800→k=18, 2100→k=21, 2400→k=24"
"±1 bin tolerance: k-1, k, k+1 → covers ±100Hz frequency offset"

Use blue-themed flow, rectangular process boxes, diamond for decision,
monospace font for formulas.
```

---

### 10.5 FEC 编解码流程图

```
A comprehensive flowchart showing both FEC encoding and decoding 
chains for a Hamming(7,4)+Interleave+CRC-8 system.

TITLE: "FEC Codec — Hamming(7,4) + Block Interleaver + CRC-8 + Chase Soft Decoder"

Split the diagram into LEFT HALF (Encoder) and RIGHT HALF (Decoder).

==================== LEFT HALF: ENCODER ====================

BOX E1 - "Input": "payload[0..plen-1], plen ≤ 51"

BOX E2 - "CRC-8 Compute":
"crc_input = [plen][payload[0..plen-1]]"
"crc = CRC8(crc_input, poly=0x07, init=0x00)"
"Append crc to payload: body = [plen][payload][crc]"
Box shows XOR-and-shift register diagram: [D7]→[D6]→...→[D0] with XOR at D2,D1,D0

BOX E3 - "LEN Prefix (Triple Redundant)":
"payload_len(1B) → nibbles[2] → HammingEncode×2 → codewords[2]"
"→ interleave(cw, 2) → 14 bits → 7 symbols"
"Repeat 3 times → 21 symbols total"

BOX E4 - "Body Encoding":
"For each byte in body[0..blen-1]:"
"  hi_nibble → HammingEncode → cw[0]"
"  lo_nibble → HammingEncode → cw[1]"
"All codewords → interleave() → bitstream"
"bitstream → 2-bit groups → 4-FSK symbols"

BOX E5 - "Hamming(7,4) Encode sub-diagram":
"Input: d1,d2,d3,d4 (4 bits)"
"p1 = d1⊕d2⊕d4"
"p2 = d1⊕d3⊕d4"
"p3 = d2⊕d3⊕d4"
"Output: [p1,p2,d1,p3,d2,d3,d4] (7 bits)"

BOX E6 - "Interleave sub-diagram":
"Input: ncw codewords × 7 bits"
"Matrix: rows=codewords, cols=7"
"Readout: column-major (col 0→6, each col all rows)"
"Effect: burst error → scattered across codewords"

BOX E7 - "Output": "tx_symbols[0..nsym-1]"

ARROWS connecting E1→E2→E3/E4→E5/E6→E7

==================== RIGHT HALF: DECODER ====================

BOX D1 - "Input": "syms[0..nsym-1], mag2[nsym][4]"

BOX D2 - "LEN Decode (Hard Decision)":
"3 copies × 7 symbols → deinterleave → HammingDecode"
"→ 3 candidate bytes → majority vote per bit → payload_len"
DIAMOND: "0 < payload_len ≤ 51?" → NO: "FAIL"

BOX D3 - "Body Soft Decode (Chase Algorithm)":
"For each byte (2 codewords):"

SUB-BOX D3a - "LLR Computation":
"From 4-FSK mag² per symbol:"
"  MSB(bit0): E1 = mag[2]+mag[3], E0 = mag[0]+mag[1]"
"  LSB(bit1): E1 = mag[1]+mag[3], E0 = mag[0]+mag[2]"
"  LLR = sign × ln(max(E1,E0)/min(E1,E0))"
"  Via LUT: mantissa ∈ [1,2), octaves → fast float"

SUB-BOX D3b - "Chase Hamming Soft Decode (per codeword)":
"1. hard[7] = LLR≥0 ? 1 : 0"
"2. abs_llr[7] = |LLR|"
"3. Find 2 least reliable bits (min abs_llr)"
"4. If min_abs_llr > 3.0: return HammingDecode(hard)"
"5. Else try 4 flip patterns:"
"   Pattern 00: no flip → HammingDecode → ideal"
"   Pattern 01: flip worst1 → HammingDecode → ideal"
"   Pattern 10: flip worst2 → HammingDecode → ideal"
"   Pattern 11: flip both → HammingDecode → ideal"
"   For each: metric = Σ abs_llr[i] × (ideal_bit[i] ≠ hard[i])"
"   Pick minimum metric → return nibble"

SUB-BOX D3c - "Hamming(7,4) Decode":
"syndrome: s1=r1⊕r3⊕r5⊕r7, s2=r2⊕r3⊕r6⊕r7, s3=r4⊕r5⊕r6⊕r7"
"syn = (s3,s2,s1)₂"
"if syn≠0: flip bit at position (7-syn)"
"extract data: [r3, r5, r6, r7]"

SUB-BOX D3d - "Deinterleave":
"Reverse of encoder interleave"
"Read bits column-major → assemble 7-bit codewords"

BOX D4 - "CRC-8 Verify":
DIAMOND: "computed CRC == received CRC?"
→ YES: "SUCCESS: output payload[0..plen-1]"
→ NO: "FAIL: discard frame"

ARROWS connecting D1→D2→D3a→D3b→D3c→D3d→D4

Color scheme:
- Encoder half: green/blue tones
- Decoder half: orange/yellow tones
- Diamond decisions: red outline
- Sub-diagrams: dashed borders
```

---

### 10.6 半双工状态机详细流程图

```
A detailed state machine diagram for the half-duplex mode switching 
of a voice messenger device. Use UML statechart style.

TITLE: "Half-Duplex Top-Level State Machine"

Define 4 superstates with internal substates:

╔══════════════════════════════════════════════════════════╗
║ SUPERSTATE: HM_RX (Default, ADC+DMA+TIM2 active)        ║
║                                                          ║
║  ┌──────────────────────────────────────────────────┐   ║
║  │ LS_LISTENING (initial)                           │   ║
║  │ - Display last received message with scroll      │   ║
║  │ - Real-time status bar (F:source, C:char_count)  │   ║
║  │ - LEFT/RIGHT: scroll wrapped                     │   ║
║  │ - Idle 20s → OLED sleep (0xAE)                   │   ║
║  │ - Any activity → OLED wake (0xAF)                │   ║
║  └────────┬──────────────────────┬─────────────────┘   ║
║           │ KEY_SEND             │ KEY_FN               ║
║           ▼                      ▼                      ║
║  ┌──────────────────┐   (to HM_TX_EDIT)                ║
║  │ LS_BROWSE_LIST   │                                   ║
║  │ - Paginated list  │                                   ║
║  │ - LEFT/RIGHT: nav │                                   ║
║  │ - DEL: delete msg │                                   ║
║  │ - SEND: view msg  │                                   ║
║  └────────┬─────────┘                                   ║
║           │ SEND / DEL                                   ║
║           ▼                                              ║
║  ┌──────────────────┐                                   ║
║  │ LS_BROWSE_VIEW   │                                   ║
║  │ - LEFT/RIGHT scrl │                                   ║
║  │ - DEL: back list  │                                   ║
║  │ - KEY_FN: → LISTEN│                                   ║
║  └──────────────────┘                                   ║
║                                                          ║
║  Entry: HM_StartRxSampling()                             ║
║  Exit:  HM_StopRxSampling()                              ║
╚══════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════╗
║ SUPERSTATE: HM_TX_EDIT (ADC+DMA off, PWM off)           ║
║                                                          ║
║  - T9 multi-tap text input (abc/ABC/123 modes)           ║
║  - 0-9: T9 input / digit input                          ║
║  - LEFT/RIGHT: cursor move                               ║
║  - DEL: backspace                                        ║
║  - KEY_FN: toggle input mode (123→abc→ABC→123)          ║
║  - SEND (non-empty): → HM_TX_SELECT                     ║
║  - KEY_FN (hold or special): → HM_RX                    ║
║  - Type "rx" + RIGHT: → HM_RX (quick switch)            ║
║                                                          ║
║  Status bar: "XX/XX [abc]   Tx Ready"                   ║
╚══════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════╗
║ SUPERSTATE: HM_TX_SELECT (Recipient selection)          ║
║                                                          ║
║  - Display: "Sender ID: X", "Select receivers:"         ║
║  - 1-9: toggle target device                             ║
║  - 0: set broadcast                                      ║
║  - SEND: confirm → HM_TX_BUSY                           ║
║  - DEL/KEY_FN: cancel → HM_TX_EDIT                      ║
╚══════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════╗
║ SUPERSTATE: HM_TX_BUSY (PWM active, TIM3 driving TX)    ║
║                                                          ║
║  - LEDR (PB10) ON                                        ║
║  - TX_Tick() driven by TIM3 ISR @ 16kHz                  ║
║  - Display: "Tx Active" status                           ║
║  - On TX_IsDone():                                       ║
║    → Show "Tx Complete" 1.5s                             ║
║    → TX_ClearDone()                                      ║
║    → PWM_DDS_Shutdown()                                  ║
║    → → HM_TX_EDIT (preserve editor content)              ║
╚══════════════════════════════════════════════════════════╝

Add a GLOBAL transition arrow:
"ANY STATE" ──[KEY_POWER (EXTI1 PB1 falling edge)]──→ "SHUTDOWN"
  Shutdown sequence:
  1. Stop ADC DMA, TIM2, TIM3
  2. RX_Stop(), PWM_DDS_Shutdown()
  3. All LEDs OFF
  4. OLED clear + refresh
  5. PB8 POWER_CTRL → LOW (cut power)

Use UML statechart conventions with rounded rectangles for states,
solid arrows for transitions with guard conditions in [brackets],
and dotted lines for substate boundaries.
```

---

### 10.7 整体系统数据流图 (DFD)

```
A data flow diagram (DFD) showing the complete data path through 
the half-duplex voice messenger system, from user input to speaker/mic.

TITLE: "System Data Flow Diagram — Half-Duplex Voice Messenger"

Use Gane-Sarson DFD notation: circles for processes, 
parallel lines for data stores, rectangles for external entities, 
arrows for data flows with labels.

══════════════ TRANSMIT PATH (top half) ═══════════════

EXTERNAL ENTITY (left): "User" (stick figure)

PROCESS P1: "T9 Editor"
Data flow: "Key presses" → P1
Data flow: P1 → "Text buffer" → DATA STORE DS1: "Editor Buffer"

PROCESS P2: "FEC Encoder\n(Hamming+Interleave+CRC8)"
Data flow: DS1 → "Text + source_id + target_mask" → P2
Data flow: P2 → "4-FSK symbol stream" → DATA STORE DS2: "Symbol Buffer"

PROCESS P3: "TX State Machine\n(TIM3 ISR driven)"
Data flow: DS2 → "Symbol sequence" → P3
Data flow: P3 → "Frequency select (0-3)" → PROCESS P4

PROCESS P4: "PWM DDS\n(Phase Accumulator + LUT)"
Data flow: P4 → "PWM CCR1 values" → PROCESS P5: "TIM1_CH1 PWM"
Data flow: P5 → "48.83kHz PWM wave" → PROCESS P6: "RC Low-Pass"
Data flow: P6 → "Audio signal (1.5-2.4kHz)" → EXTERNAL ENTITY (right): "Speaker"

══════════════ RECEIVE PATH (bottom half) ═══════════════

EXTERNAL ENTITY (right): "Microphone"

PROCESS P7: "ADC + DMA\n(TIM2 TRGO @16kHz)"
Data flow: "Analog audio" → P7
Data flow: P7 → "adc_dma_buf[800]" → DATA STORE DS3: "DMA Buffer"

PROCESS P8: "Bandpass Filter\n(1.1-2.8kHz IIR)"
Data flow: DS3 → "12-bit samples" → P8
Data flow: P8 → "Filtered float samples" → PROCESS P9

PROCESS P9: "VoiceDSP State Machine\n(80-sample blocks)"
Internal sub-processes (show as nested circles):
- P9a: "DiffEnergy + Noise Floor"
- P9b: "Goertzel Classify (preamble)"
- P9c: "Sync Onset Detect"
- P9d: "Multi-Window Goertzel (data)"
Data flow: P9 → "symbols[] + sym_mag2[][]" → PROCESS P10

PROCESS P10: "FEC Decoder\n(LLR + Chase + Deinterleave)"
Data flow: P10 → "payload[] + crc_ok" → PROCESS P11

PROCESS P11: "Address Filter"
DIAMOND annotation: "For me? (broadcast OR my ID)"
Data flow: P11 → "Accepted messages" → DATA STORE DS4: "Message Buffer"

DATA STORE DS5: "SPI Flash\n(PY25Q64HA, dual-copy)"
Data flow: DS4 → "Auto-save" → DS5

PROCESS P12: "OLED Display"
Data flow: DS4 → "Message text" → P12
Data flow: DS5 → "Stored messages" → P12
Data flow: P12 → "I2C framebuffer" → EXTERNAL ENTITY: "OLED 128×64"

Add two control flow annotations (dashed lines):
"HM_RX/TX mode switch" controlling P7/P8/P9 (RX on/off) 
  and P3/P4/P5 (TX on/off) — mutually exclusive
"KEY_POWER" → "g_power_off flag" → all processes stop

Use:
- Circles with numbers and labels for processes
- Two parallel horizontal lines for data stores
- Squares for external entities
- Solid arrows for data flows
- Dashed arrows for control flows
- Gray background for TX path, white for RX path
```

---

## 11. 程序流程图生成 Prompt（中文版）

以下为 7 张核心流程图的**中文描述 Prompt**，可直接交付国内 AI 绘图工具（如通义万相等）生成流程图，
也可参照描述在 Draw.io / Visio / ProcessOn 中手动绘制。
每段 Prompt 详细描述了图的节点、箭头、标注和配色方案。

---

### 11.1 系统总体流程图

```
绘制一张专业的技术流程图，展示基于 STM32F411CEU6 微控制器的半双工声语信使系统的完整架构。

顶部区域 — "上电启动序列"（从左到右排列 6 个圆角矩形，箭头连接）：
方框1: "按下电源键\n(PB1=低电平 → PB8=高电平锁存)"
方框2: "HAL与时钟初始化\n(HSE 25MHz → PLL 50MHz)"
方框3: "外设初始化\n(ADC/TIM/DMA/I2C/SPI/GPIO)"
方框4: "OLED启动画面\n(中文开机页，显示2秒)"
方框5: "选择设备编号\n(按1-9键 + 发送键确认)"
方框6: "进入HM_RX模式\n(默认接收监听)"

中部区域 — "半双工状态机"（居中绘制，4 个大圆角矩形）：
- HM_RX（左上，蓝色）: "接收模式\nADC+DMA+TIM2 开启\nTIM1 PWM 关闭"
- HM_TX_EDIT（右上，绿色）: "编辑模式\nT9 文字输入\nADC 关闭，PWM 关闭"
- HM_TX_SELECT（右侧，黄色）: "选择收件人\n1-9 切换 / 0=广播"
- HM_TX_BUSY（右下，红色）: "发送中\nTIM1 PWM 开启\nTIM3 中断驱动发送"

在状态之间绘制带箭头方向线，标注转换条件：
- HM_RX → HM_TX_EDIT: "按英/数键 或 数字键"
- HM_TX_EDIT → HM_RX: "按英/数键 或 输入'rx'+右键"
- HM_TX_EDIT → HM_TX_SELECT: "按发送键（消息非空）"
- HM_TX_SELECT → HM_TX_EDIT: "按删除键 或 英/数键"
- HM_TX_SELECT → HM_TX_BUSY: "按发送键"
- HM_TX_BUSY → HM_TX_EDIT: "发送完成（自动）"
- 任意状态 → 关机: "按电源键 (EXTI1中断)"

底部区域 — "RX 子模式"（在 HM_RX 框内绘制 3 个小方框）：
- LS_LISTENING: "实时监听\n显示最近收到的消息"
- LS_BROWSE_LIST: "已存储消息列表\n（最多64条，分页显示）"
- LS_BROWSE_VIEW: "查看单条消息\n左右键滚动"
箭头: LISTENING ←→ BROWSE_LIST（发送键切换），BROWSE_LIST ←→ BROWSE_VIEW（发送/删除键切换）

右侧边栏 — "外设资源归属表"（表格格式）：
| 外设 | HM_RX | HM_TX_EDIT | HM_TX_BUSY |
|------|-------|------------|------------|
| ADC+DMA+TIM2 | 开启 | 关闭 | 关闭 |
| TIM1 PWM | 关闭 | 关闭 | 开启 |
| TIM3 中断 | 空闲 | 空闲 | 驱动发送 |

使用简洁的工程图风格，蓝白配色方案，12pt 字体标注，2px 带箭头连接线。
```

---

### 11.2 发送链路流程图

```
绘制一张纵向技术流程图，展示 4-FSK 声语信使从文字输入到扬声器输出的完整发送信号处理链路。

顶部标题: "发送信号链路 — 编辑器 → FEC编码 → PWM DDS → 扬声器"

布局: 从上到下纵向排列，6 个主处理块由粗向下箭头连接。每个块包含标题和子项。

第1块 — "T9 文字编辑器"（浅灰色）:
- "用户在 4×4 矩阵键盘上输入"
- "T9 多次击键: 小写abc/大写ABC/数字123 三种模式"
- "光标左右移动、退格删除"
- "输出: 文字缓冲区（最多48个字符）"

第2块 — "TX_Start(文字, 源ID, 目标掩码)"（浅蓝色）:
- "构建负载: [源ID(1字节)][掩码低字节][掩码高字节][文字...]"
- "校验源ID范围 [1,9]"
- "调用 VoiceFEC_BuildDataSymbols() 进行 FEC 编码"

第3块 — "FEC 编码 (voice_fec.c)"（浅黄色）:
子步骤从左到右排列:
a) "LEN字节 → 3重冗余 Hamming(7,4) 编码 → 21个符号"
b) "每个负载字节 → 2个半字节 → 2个 Hamming(7,4) 码字"
c) "对 [LEN][负载] 计算 CRC-8 校验值"
d) "全部码字比特 → 块交织器（按列优先读出）"
e) "交织后的比特流 → 2比特一组 → 4-FSK 符号 [0,1,2,3]"

第4块 — "TX_Tick() 发送状态机（TIM3中断驱动，16kHz）"（浅绿色）:
状态方框依次排列:
- "ST_PREAMBLE (2000ms)\n  每40ms交替输出 1500/2400Hz" → 
- "ST_SYNC (30ms)\n  1800Hz载波20ms + DC保护间隔10ms" → 
- "ST_DATA (变长)\n  每符号: 20ms载波 + 10ms保护\n  tx_symbols[idx] → PWM_DDS_SetFreq()" → 
- "ST_DATA_TAIL (30ms)\n  输出DC中点（隔离槽）" → 
- "ST_POSTAMBLE (120ms)\n  连续 2400Hz" → 
- "ST_DONE\n  LEDR灯灭，TIM3停止，输出DC中点"

第5块 — "PWM DDS 合成 (pwm_dds.c)"（浅橙色）:
- "32位相位累加器: phase_acc += phase_inc（每16kHz周期）"
- "10位LUT索引: idx = (phase_acc >> 22) & 0x3FF"
- "1024点正弦查找表，值域 [2..1022]，中心=512"
- "CCR1 = sine_lut[idx] → TIM1_CH1 PWM输出 @ 48.83kHz"

第6块 — "模拟输出"（浅红色）:
- "RC低通滤波器: 1kΩ + 22nF（截止频率约 7.2kHz）"
- "滤除 48.83kHz PWM 载波分量"
- "→ 扬声器 / 音频插孔"

右侧标注 — 时序示意:
"每符号时序: [20ms载波 | 10ms保护] × N个符号\n
完整帧: 2000ms前导 + 30ms同步 + N×30ms数据 + 30ms尾部隔离 + 120ms结束音\n
最坏情况（48字符）: 约13.3秒 < 20秒上限"

使用工程制图风格，代码标签使用等宽字体，圆角矩形带阴影，连接线带箭头。
```

---

### 11.3 接收链路流程图

```
绘制一张纵向技术流程图，展示 4-FSK 声语信使从麦克风输入到解码消息显示的完整接收信号处理链路。

顶部标题: "接收信号链路 — ADC → 带通滤波 → DSP 状态机 → FEC 解码 → 显示"

布局: 从上到下纵向排列，7 个主处理块。

第1块 — "模拟前端与ADC采样"（灰色）:
- "驻极体麦克风 + 前置放大器 → 偏置到 1.65V 直流中点"
- "ADC1_IN8 (PB0引脚)，12位精度，TIM2 TRGO 触发"
- "DMA2_Stream0: 800个半字循环缓冲区"
- "双缓冲机制: HT半传输中断(前400) / TC传输完成中断(后400)"

第2块 — "带通滤波器 (1.1-2.8 kHz)"（蓝色）:
- "1个二阶高通 + 3个二阶低通 = 8阶 IIR 带通滤波器"
- "每样本处理: centered = ADC值 - 2048 → 级联双二阶递推"
- "滤除PWM残留、50Hz工频干扰、带外噪声"
- "输出: vd_filtered_blk[80] 浮点采样数组"

第3块 — "VoiceDSP 接收状态机"（绿色）:
分为 4 个子状态，各含详细处理步骤:

子状态A — "VD_LISTEN（噪声标定阶段）":
- "一阶差分能量 = Σ|x[i]-x[i-1]|（等效高通，免疫DC/50Hz）"
- "前15块（约75ms）: 快速 IIR 噪声基线标定"
- "之后: 慢速 IIR 跟踪，门限 = 噪声基线 + 24，不低于32"
- "每160采样窗口: Goertzel 4频分类（±1 bin容错）"
- "统计前导读中次数(pilot_hits)和1500/2400交替次数(pilot_trans)"
- "进入前导条件: hits≥2 且 trans≥8"

子状态B — "VD_PREAMBLE（同步音搜索）":
- "每160采样窗口: Goertzel 检测 1800Hz"
- "连续命中2次 → 同步音上升沿回扫（sync_find_onset）:"
- "  从当前时刻向前扫描 400 采样，步长 8 采样"
- "  每个位置取80采样窗口做 Goertzel（k=9，对应1800Hz）"
- "  找到 mag² ≥ 峰值×0.8 的第一个位置"
- "  → 锁定符号栅格: 栅格起点 = 上升沿位置 + 480采样"
- "→ 进入 VD_DATA"

子状态C — "VD_DATA（多窗口解调）":
- "对于数据区内的每个采样点:"
- "  每符号累积 320 采样（完整20ms载波窗口）"
- "  跳过保护间隔采样（符号内位置 ≥ 320 的部分）"
- "载波窗口填满后 → VoiceDSP_ClassifyMulti():"
- "  3个重叠窗口（起点偏移 40/80/120，窗长=160）"
- "  每个窗口: Goertzel 4频，±1 bin 频带能量累加"
- "  累加 3 个窗口的 mag² 值"
- "  施加频响补偿权重: {1.33, 1.08, 1.00, 1.02}"
- "  计算真信噪比 True SNR = 信号功率 / 噪声功率"
- "  判决: SNR≥2.0 或 频谱占比≥1.35 → 输出数字(0-3)；否则擦除(0xFF)"
- "  保存 mag²[4] 供软判决 FEC 使用"
- "收到 21 个符号后（LEN前缀）: 解码负载长度"
- "收齐全部符号 → 进入 VD_DONE"

子状态D — "VD_DONE":
- "调用 VoiceFEC_ParseDataSymbolsSoft() 进行软判决解码"
- "CRC-8 校验"

第4块 — "FEC 软判决解码 (voice_fec.c)"（黄色）:
- "LEN解码: 3份冗余码字 → 逐比特多数表决 → payload_len"
- "对每个字节（2个码字 × 7比特）:"
- "  从 4-FSK 符号的 mag² 值计算每比特 LLR"
- "  LLR = 符号 × ln(max(E1,E0)/min(E1,E0))（查表法）"
- "  Chase 解码器: 找 2 个最不可靠比特位"
- "  尝试 4 种翻转组合，选取最小软距离候选"
- "  Hamming(7,4) 解码 → 半字节 → 字节"
- "CRC-8 校验: 计算结果 vs 接收结果"

第5块 — "地址过滤"（橙色）:
菱形判断: "源ID ∈ [1,9]？"
→ 是: "目标掩码是广播 或 包含本机ID？"
  → 是: "接收本帧 → 设置 rx_done_flag=1"
  → 否: "丢弃，重新开始监听"
→ 否: "丢弃，重新开始监听"

第6块 — "消息处理"（粉色）:
- "复制到显示缓冲区 (rx_display_msg)"
- "自动保存到外置 SPI Flash (FlashStore_SaveMessageFrom)"
- "OLED 显示 'Rx Complete' 持续 2 秒"
- "重新启动 ADC + DMA + TIM2 采样"

第7块 — "LED 指示灯"（标注说明）:
- "绿色LED（PB2）: 仅在 VD_DATA 期间点亮（收到有效数据符号）"
- "红色LED（PB10）: 接收期间熄灭（仅发送时点亮）"
- "板载LED（PC13）: 接收监听时熄灭，接收完成时点亮"

采用技术流程图风格，各块用不同颜色区分，子步骤用小字体标注，
菱形表示决策节点，数据流箭头清晰标注方向。
```

---

### 11.4 Goertzel 算法流程图

```
绘制一张详细的算法流程图，展示 4-FSK 声语信使接收端使用的 Goertzel 多窗口频率检测算法。

标题: "Goertzel 多窗口 4-FSK 分类器 (VoiceDSP_ClassifyMulti)"

输入标注: "tone[320] 浮点采样数组，信噪比门限 = 2.0"

主流程（从上到下）:

┌─ 循环: w = 0 到 2（共3个窗口）────────────────────────────┐
│ offset 取值 {40, 80, 120}                                   │
│ 子窗口 sub = tone[offset .. offset+159]                     │
│                                                             │
│ ┌─ 去直流分量 ───────────────────────────────────┐          │
│ │ dc = (Σ sub[i]) / 160                           │          │
│ │ 对 i=0..159: dev = sub[i] - dc                  │          │
│ │ 时域总功率 P_total += dev²                      │          │
│ └─────────────────────────────────────────────────┘          │
│                                                             │
│ ┌─ 四频 Goertzel 检测 (f=0..3) ──────────────────┐          │
│ │ 对每个目标频率的中心 bin k（如 1500Hz→k=15）:   │          │
│ │   频带能量 =                                      │          │
│ │     goertzel(k-1, sub, dc) +  // -100Hz 偏移     │          │
│ │     goertzel(k,   sub, dc) +  // 中心频率        │          │
│ │     goertzel(k+1, sub, dc)    // +100Hz 偏移     │          │
│ │   acc_mag2[f] += 频带能量                        │          │
│ │                                                  │          │
│ │   goertzel(k, window, dc) 递推算法:              │          │
│ │     coeff = 2×cos(2π×k/160)                     │          │
│ │     q1=0, q2=0                                   │          │
│ │     循环 i=0 到 159:                             │          │
│ │       q0 = coeff×q1 - q2 + (window[i]-dc)        │          │
│ │       q2 = q1; q1 = q0                           │          │
│ │     返回 q1² + q2² - q1×q2×coeff                 │          │
│ └─────────────────────────────────────────────────┘          │
└──────────────────────────────────────────────────────────────┘

┌─ 频响补偿 ─────────────────────────────────────────┐
│ wgt[f] = acc_mag2[f] × [1.33, 1.08, 1.00, 1.02]   │
│ // 补偿1500Hz在声学链路中增益偏低的问题               │
└─────────────────────────────────────────────────────┘

┌─ 找最强频点和次强频点 ─────────────────────────────┐
│ 最强 = argmax(wgt[0..3])，最强值 = wgt[最强]        │
│ 次强值 = max(wgt[i≠最强])                           │
│ 频谱占比 = 最强值 / max(次强值, 1e-12)              │
└─────────────────────────────────────────────────────┘

┌─ 真信噪比计算 ─────────────────────────────────────┐
│ 信号功率 = acc_mag2[最强] × (2/160)                  │
│ 噪声功率 = max(P_total - 信号功率, 1.0)              │
│ SNR = 信号功率 / 噪声功率                             │
└─────────────────────────────────────────────────────┘

┌─ 判决（菱形）──────────────────────────────────────┐
│          SNR ≥ 2.0 或 频谱占比 ≥ 1.35 ？            │
│           ↙ 是                     ↘ 否             │
│   返回 digit (0-3)           返回 0xFF (擦除)        │
└─────────────────────────────────────────────────────┘

流程图下方附加说明框:
"频率映射: 0=1500Hz  1=1800Hz  2=2100Hz  3=2400Hz"
"bin 映射 (N=160, fs=16000, Δf=100Hz): 1500→k=15, 1800→k=18, 2100→k=21, 2400→k=24"
"±1 bin 容错: k-1, k, k+1 → 覆盖 ±100Hz 频率偏差"

使用蓝色主题，矩形为处理步骤，菱形为决策节点，公式使用等宽字体。
```

---

### 11.5 FEC 编解码流程图

```
绘制一张综合流程图，同时展示 Hamming(7,4)+块交织+CRC-8 系统的 FEC 编码和解码链路。

标题: "FEC 编解码器 — Hamming(7,4) + 块交织 + CRC-8 + Chase 软判决解码"

将图分为左半部分（编码器）和右半部分（解码器）。

══════════ 左半部分: 编码器 ═══════════

方框E1 — "输入": "payload[0..plen-1]，plen ≤ 51"

方框E2 — "CRC-8 计算":
"crc_input = [plen][payload[0..plen-1]]"
"crc = CRC8(crc_input, 多项式=0x07, 初值=0x00)"
"将 crc 追加到 payload: body = [plen][payload][crc]"
方框内同时画出移位寄存器示意图: [D7]→[D6]→...→[D0]，在 D2,D1,D0 处标注异或门

方框E3 — "LEN 前缀（三重冗余）":
"payload_len(1字节) → 2个半字节 → 各做 HammingEncode → 2个码字(各7比特)"
"→ 交织(2个码字) → 14比特 → 7个符号"
"重复 3 次 → 共计 21 个符号"

方框E4 — "负载编码":
"对 body[0..blen-1] 中的每个字节:"
"  高半字节 → HammingEncode → 码字[0]"
"  低半字节 → HammingEncode → 码字[1]"
"全部码字 → 块交织() → 比特流"
"比特流 → 2比特一组 → 4-FSK 符号"

方框E5 — "Hamming(7,4) 编码子图":
"输入: d1,d2,d3,d4（4个数据比特）"
"校验位计算:"
"  p1 = d1⊕d2⊕d4"
"  p2 = d1⊕d3⊕d4"
"  p3 = d2⊕d3⊕d4"
"输出: [p1,p2,d1,p3,d2,d3,d4]（7比特码字）"

方框E6 — "块交织子图":
"输入: ncw 个码字 × 7 比特"
"排列成矩阵: 行=码字，列=7"
"读出顺序: 按列优先（第0列→第6列，每列遍历所有行）"
"效果: 连续突发错误 → 分散到不同码字中"

方框E7 — "输出": "tx_symbols[0..nsym-1]"

用箭头连接 E1→E2→(分叉到E3和E4)→E5/E6→E7

══════════ 右半部分: 解码器 ═══════════

方框D1 — "输入": "syms[0..nsym-1]，mag2[nsym][4]"

方框D2 — "LEN 解码（硬判决）":
"3份副本 × 7个符号 → 反交织 → HammingDecode"
"→ 3个候选字节 → 逐比特多数表决 → payload_len"
菱形判断: "0 < payload_len ≤ 51 ？" → 否: "解码失败"

方框D3 — "负载软判决解码（Chase 算法）":
"对每个字节（2个码字）:"

子框D3a — "LLR 计算":
"从 4-FSK 每符号的 mag² 计算:"
"  MSB（比特0）: E1 = mag[2]+mag[3]，E0 = mag[0]+mag[1]"
"  LSB（比特1）: E1 = mag[1]+mag[3]，E0 = mag[0]+mag[2]"
"  LLR = 符号 × ln(max(E1,E0)/min(E1,E0))"
"  用查表法实现: 尾数 ∈ [1,2)，倍频程 → 快速浮点计算"

子框D3b — "Chase 软 Hamming 解码（每个码字）":
"1. 硬判: hard[7] = LLR≥0 ? 1 : 0"
"2. 绝对值: abs_llr[7] = |LLR|"
"3. 找 2 个最不可靠比特（abs_llr 最小的两个位置）"
"4. 如果最小 |LLR| > 3.0: 直接返回 HammingDecode(hard) 结果"
"5. 否则尝试 4 种翻转组合:"
"   组合 00: 不翻转 → HammingDecode → 理想码字"
"   组合 01: 翻转最不可靠比特1 → HammingDecode → 理想码字"
"   组合 10: 翻转最不可靠比特2 → HammingDecode → 理想码字"
"   组合 11: 两比特都翻转 → HammingDecode → 理想码字"
"   每种组合计算: 软距离 = Σ abs_llr[i] × (理想比特[i] ≠ 硬判比特[i])"
"   选取最小软距离 → 返回半字节"

子框D3c — "Hamming(7,4) 解码":
"伴随式计算: s1=r1⊕r3⊕r5⊕r7, s2=r2⊕r3⊕r6⊕r7, s3=r4⊕r5⊕r6⊕r7"
"伴随式值 syn = (s3,s2,s1)₂"
"如果 syn ≠ 0: 翻转第 (7-syn) 位置的比特"
"提取数据: [r3, r5, r6, r7] → 4比特半字节"

子框D3d — "反交织":
"编码交织的逆过程"
"按列优先读取比特 → 组装成 7 比特码字"

方框D4 — "CRC-8 校验":
菱形判断: "计算的 CRC == 接收的 CRC ？"
→ 是: "成功: 输出 payload[0..plen-1]"
→ 否: "失败: 丢弃本帧"

用箭头连接 D1→D2→D3a→D3b→D3c→D3d→D4

配色方案:
- 编码器半部分: 绿色/蓝色调
- 解码器半部分: 橙色/黄色调
- 菱形决策节点: 红色边框
- 子图: 虚线边框
```

---

### 11.6 半双工状态机详细流程图

```
绘制一张详细的半双工模式切换状态机图，采用 UML 状态图风格。

标题: "半双工顶层状态机"

定义 4 个超状态，每个包含内部子状态:

═══════════════════════════════════════════════════════════
超状态: HM_RX（默认状态，ADC+DMA+TIM2 工作）
                                                          
  ┌──────────────────────────────────────────────────┐   
  │ LS_LISTENING（初始子状态）                        │   
  │ - 显示最近收到的消息，支持滚动                    │   
  │ - 实时状态栏（F:来源ID，C:字符数）                │   
  │ - 左/右键: 循环滚动消息内容                       │   
  │ - 空闲 20 秒 → OLED 熄屏（0xAE 命令）             │   
  │ - 任意活动 → OLED 唤醒（0xAF 命令+刷新）          │   
  └────────┬──────────────────────┬─────────────────┘   
           │ 按发送键              │ 按英/数键            
           ▼                       ▼                      
  ┌──────────────────┐   （切换到 HM_TX_EDIT）           
  │ LS_BROWSE_LIST   │                                   
  │ - 分页消息列表    │                                   
  │ - 左/右键: 导航   │                                   
  │ - 删除键: 删除    │                                   
  │ - 发送键: 查看    │                                   
  └────────┬─────────┘                                   
           │ 发送键 / 删除键                              
           ▼                                              
  ┌──────────────────┐                                   
  │ LS_BROWSE_VIEW   │                                   
  │ - 左/右键: 滚动   │                                   
  │ - 删除键: 返回列表│                                   
  │ - 英/数键: 回监听 │                                   
  └──────────────────┘                                   
                                                          
  进入: HM_StartRxSampling()                              
  退出: HM_StopRxSampling()                               
═══════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════
超状态: HM_TX_EDIT（ADC+DMA关闭，PWM关闭）
                                                          
  - T9 多次击键文字输入（abc/ABC/123 三种模式）           
  - 数字键: T9 输入 / 数字直接输入                        
  - 左/右键: 光标左右移动                                 
  - 删除键: 退格删除                                      
  - 英/数键: 切换输入模式（123→abc→ABC→123 循环）        
  - 发送键（消息非空时）: → HM_TX_SELECT                 
  - 英/数键（特殊触发）: → HM_RX                         
  - 输入 "rx" 然后按右键: → HM_RX（快速切回收信）        
                                                          
  状态栏: "已输入字符数/剩余字符数 [模式]   Tx Ready"      
═══════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════
超状态: HM_TX_SELECT（收件人选择）
                                                          
  - 显示: "发送者 ID: X"，"选择收件人:"                   
  - 数字 1-9: 切换目标设备（多选）                        
  - 数字 0: 设为广播                                      
  - 发送键: 确认 → HM_TX_BUSY                           
  - 删除键/英/数键: 取消 → HM_TX_EDIT                    
═══════════════════════════════════════════════════════════

═══════════════════════════════════════════════════════════
超状态: HM_TX_BUSY（PWM工作，TIM3中断驱动发送）
                                                          
  - 红色LED（PB10）点亮                                   
  - TX_Tick() 由 TIM3 中断 @ 16kHz 驱动                   
  - 显示: "Tx Active" 状态                                
  - 当 TX_IsDone() 返回真:                                
    → 显示 "Tx Complete" 持续 1.5 秒                      
    → 调用 TX_ClearDone()                                 
    → 调用 PWM_DDS_Shutdown()                             
    → → 切换到 HM_TX_EDIT（保留编辑器内容）               
═══════════════════════════════════════════════════════════

添加一条全局转换箭头:
"任意状态" ──[按电源键（EXTI1 PB1 下降沿中断）]──→ "关机"
  关机流程:
  1. 停止 ADC DMA，停止 TIM2，停止 TIM3
  2. 调用 RX_Stop()，调用 PWM_DDS_Shutdown()
  3. 熄灭所有 LED
  4. OLED 清屏并刷新
  5. PB8 POWER_CTRL 置低电平（切断电源）

使用 UML 状态图规范: 状态用圆角矩形表示，转换用实线箭头，
转换条件用方括号标注 [条件]，子状态边界用虚线表示。
```

---

### 11.7 整体系统数据流图

```
绘制一张数据流图（DFD），展示半双工声语信使系统从用户输入到扬声器/麦克风的完整数据通路。

标题: "系统数据流图 — 半双工声语信使"

使用 Gane-Sarson DFD 标注法: 圆形表示处理过程，
双平行线表示数据存储，矩形表示外部实体，
带标签的箭头表示数据流。

═══════════ 发送通路（上半部分） ═══════════

外部实体（左侧）: "用户"（用人形图标表示）

处理过程 P1: "T9 编辑器"
数据流: "按键输入" → P1
数据流: P1 → "文字缓冲" → 数据存储 DS1: "编辑器缓冲区"

处理过程 P2: "FEC 编码器\n（Hamming+交织+CRC8）"
数据流: DS1 → "文字+源ID+目标掩码" → P2
数据流: P2 → "4-FSK 符号流" → 数据存储 DS2: "符号缓冲区"

处理过程 P3: "TX 发送状态机\n（TIM3中断驱动）"
数据流: DS2 → "符号序列" → P3
数据流: P3 → "频率选择 (0-3)" → 处理过程 P4

处理过程 P4: "PWM DDS\n（相位累加器+查找表）"
数据流: P4 → "PWM CCR1 值" → 处理过程 P5: "TIM1_CH1 PWM"
数据流: P5 → "48.83kHz PWM波" → 处理过程 P6: "RC 低通滤波"
数据流: P6 → "音频信号 (1.5-2.4kHz)" → 外部实体（右侧）: "扬声器"

═══════════ 接收通路（下半部分） ═══════════

外部实体（右侧）: "麦克风"

处理过程 P7: "ADC + DMA\n（TIM2 TRGO @ 16kHz）"
数据流: "模拟音频" → P7
数据流: P7 → "adc_dma_buf[800]" → 数据存储 DS3: "DMA 缓冲区"

处理过程 P8: "带通滤波器\n（1.1-2.8kHz IIR）"
数据流: DS3 → "12位采样值" → P8
数据流: P8 → "滤波后浮点采样" → 处理过程 P9

处理过程 P9: "VoiceDSP 状态机\n（80采样块驱动）"
内部子过程（用嵌套小圆表示）:
- P9a: "差分能量+噪声基线"
- P9b: "Goertzel 分类（前导）"
- P9c: "同步音上升沿检测"
- P9d: "多窗口 Goertzel（数据）"
数据流: P9 → "symbols[] + sym_mag2[][]" → 处理过程 P10

处理过程 P10: "FEC 解码器\n（LLR+Chase+反交织）"
数据流: P10 → "payload[] + crc_ok" → 处理过程 P11

处理过程 P11: "地址过滤器"
菱形标注: "是给我的吗？（广播 或 含本机ID）"
数据流: P11 → "已接受的消息" → 数据存储 DS4: "消息缓冲区"

数据存储 DS5: "SPI Flash\n（PY25Q64HA，双副本）"
数据流: DS4 → "自动保存" → DS5

处理过程 P12: "OLED 显示"
数据流: DS4 → "消息文字" → P12
数据流: DS5 → "已存储消息" → P12
数据流: P12 → "I2C 帧缓冲" → 外部实体: "OLED 128×64"

添加两条控制流标注（虚线）:
"HM_RX/TX 模式切换" 控制 P7/P8/P9（接收 开/关）
  和 P3/P4/P5（发送 开/关）—— 二者互斥
"电源键" → "g_power_off 标志" → 停止所有处理过程

使用规范:
- 圆形+编号+标签表示处理过程
- 两条平行横线表示数据存储
- 矩形表示外部实体
- 实线箭头表示数据流
- 虚线箭头表示控制流
- 发送通路用浅灰背景，接收通路用白色背景
```

---

## 附录: 关键常量速查

| 常量 | 值 | 说明 |
|------|-----|------|
| VP_SAMPLE_RATE | 16000 | ADC/DDS 采样率 |
| VP_TONE_SAMPLES | 320 | 20ms 载波样本数 |
| VP_GUARD_SAMPLES | 160 | 10ms 保护间隔样本数 |
| VP_SLOT_SAMPLES | 480 | 30ms 符号槽样本数 |
| VP_PREAMBLE_MS | 2000 | 前导时长 |
| VP_POSTAMBLE_MS | 120 | 结束音时长 |
| VP_MAX_CHARS | 48 | 单条消息最大字符数 |
| VP_HEADER_BYTES | 3 | [src, mask_lo, mask_hi] |
| VP_SYMS_PER_BYTE | 7 | 每字节→7个4-FSK符号 |
| VD_BLOCK | 80 | 处理块大小 (5ms) |
| VD_WIN | 160 | Goertzel 窗大小 (10ms) |
| VD_MULTI_WIN_COUNT | 3 | 多窗口数量 |
| VD_SNR_MIN | 2.0 | 数据段最低 SNR 门限 |
| VD_FREQ_RATIO_MIN | 1.35 | 频谱占比最低门限 |
| VD_EN_MIN | 32 | 差分能量绝对最低门限 |
| FLASH_STORE_MAX_MSGS | 64 | 最大消息存储数 |
