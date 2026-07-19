# 声语信使 — 半双工项目进度文档

> 最后更新: 2026-07-19
> 阶段: v5.1 多窗口 Goertzel + 自适应 SNR + 软判决 Chase + PGA112 冻结式 AGC

---

## 一、项目概述

电子系统课程设计 — "声语信使"（Voice Messenger）半双工终端，基于 STM32F411CEU6。
同一块板既是发送端也是接收端，但不能同时进行(半双工)。

---

## 二、架构总览

### 顶层状态机 (半双工)

```
HM_RX (默认接收)
  ├─ LS_LISTENING: 监听/显示上次消息 (默认子模式)
  ├─ LS_BROWSE_LIST: 已存储消息列表 (64条循环缓冲)
  └─ LS_BROWSE_VIEW: 查看单条存储消息
HM_TX_EDIT (编辑短信息)
HM_TX_SELECT (收件人多选)
HM_TX_BUSY (发送中)
```

### 模式切换

| 从 | 到 | 触发 |
|---|----|------|
| HM_RX | HM_TX_EDIT | KEY_FN 或 T9数字键 |
| HM_RX | LS_BROWSE_LIST | KEY_SEND |
| HM_TX_EDIT | HM_TX_SELECT | KEY_SEND (消息非空) |
| HM_TX_SELECT | HM_TX_BUSY | KEY_SEND (确认发送) |
| HM_TX_EDIT | HM_RX | KEY_FN 或 KEY_RIGHT(光标在末尾) |
| HM_TX_BUSY | HM_TX_EDIT | TX done 自动返回 (保留编辑器内容) |
| RX 子模式切换 | LS_LISTENING↔BROWSE_LIST↔BROWSE_VIEW | KEY_FN / KEY_SEND / 数字键 |
| 任意 | 关机 | POWER_BUTTON (PB1 下降沿) |

---

## 三、硬件配置

| 模块 | 配置 |
|------|------|
| MCU | STM32F411CEU6 |
| 时钟 | **HSE 25MHz** 外部晶振 → PLL 50MHz SYSCLK, CSS 使能 |
| 音频输出 | PA8, TIM1_CH1 PWM DDS, 48.83kHz 载波 → RC 低通 |
| 音频输入 | PB0, ADC1_IN8, TIM2 TRGO_UPDATE 触发 @ 16kHz (PSC=4, ARR=624) |
| DMA | DMA2_Stream0, 循环 800 halfwords |
| DDS 时钟 | TIM3 ISR @ 16kHz (PSC=4, ARR=624) |
| 键盘 | 4x4 矩阵, PA0~PA3 rows, **PA9~PA12** cols |
| OLED | I2C1 (PB6/PB7), SSD1306 128x64 |
| 第二级放大 | PGA112, SPI2 (PB12=CS, PB13=SCK, PB15=MOSI), **16x** 初始增益 |
| 外部存储 | PY25Q64 8MB SPI NOR Flash, SPI1 (PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI) |
| 电源管理 | PB1=POWER_BUTTON (EXTI1), PB8=POWER_CTRL (硬件锁存) |
| LED | PC13=收发状态指示, PB2=LEDG(绿/灭), PB10=LEDR(红/灭) |

### 引脚分配

| 功能 | 引脚 | 模式 |
|------|------|------|
| KB_ROW0~3 | PA0~PA3 | Output PP |
| **KB_COL0~3** | **PA9~PA12** | Input PU |
| PWM 音频输出 | PA8 | AF1 PP |
| ADC 音频输入 | PB0 | Analog |
| OLED SCL/SDA | PB6/PB7 | AF4 OD |
| LED | PC13 | Output PP |
| LEDG | PB2 | Output PP |
| LEDR | PB10 | Output PP |
| POWER_BUTTON | PB1 | Input PU + EXTI1 |
| POWER_CTRL | PB8 | Output PP |
| F_CS (Flash) | PA4 | Output PP |
| PG112_CS | PB12 | Output PP |

---

## 四、协议 (v5.1)

### 帧结构

```
[Preamble 200ms] [SYNC 1800Hz 30ms] [LEN×3冗余] [Payload+CRC8: Hamming(7,4)+交织]
[DC保护槽 30ms] [Postamble 2400Hz 120ms]
```

- **变长帧**: payload = [source_id(1B)][mask_lo(1B)][mask_hi(1B)][text...]
- **FEC**: Hamming(7,4) + 块交织, 每字节→7符号
- **CRC-8**: poly 0x07, 覆盖 LEN + payload
- **LEN**: 三重冗余, 逐bit多数表决
- **符号时序**: 20ms tone + 10ms guard = 30ms 槽
- **4-FSK**: 1500/1800/2100/2400 Hz

### 源文件 (12个活跃)

| 文件 | 功能 |
|------|------|
| main.c | 顶层状态机 + UI |
| transmitter.c | v5 发送 (变长帧 + FEC) |
| receiver.c | v5 接收 (状态驱动 AGC) |
| voice_dsp.c | v5.1 DSP (True SNR) |
| voice_fec.c | Hamming(7,4) + CRC-8 |
| pwm_dds.c | PWM DDS 正弦生成 |
| fsk4_encoder.c | DDS 相位增量计算 |
| flash_store.c + py25q64.c | SPI Flash 存储 |
| pga112.c | PGA112 AGC |
| oled.c | SSD1306 OLED |
| keyboard.c | 4x4 键盘扫描 |
| editor.c | T9 文本编辑 |

---

## 五、信息存储 — PY25Q64 SPI NOR Flash

- 外部 PY25Q64 8MB SPI Flash (SPI1)
- A/B 双副本 (0x000000 / 0x001000), CRC32 完整性校验 + generation 仲裁
- 64 条消息 FIFO 循环缓冲
- 写入前暂停 ADC DMA

---

## 六、变更记录

### 2026-07-16 — v5.1 True SNR 分类器 + 频域抗噪重构

针对接收端两大隐蔽缺陷进行 DSP 层 + 状态机层重构:

**缺陷 1 — 差分能量对频率偏置导致弱信号漏检**: `VoiceDSP_DiffEnergy` 对 1500Hz
载波只有 2400Hz 的 62.5% (dE ∝ A×f), 相同声学幅度下低频符号的时域能量天然更低,
被 `VD_EN_MARGIN=1000` 挡在门外.

**缺陷 2 — 时域能量误导信号丢失判定**: VD_PREAMBLE 的 `lo_run >= 20U` 和 VD_DATA
的 `lo_run >= 100U` 在数据段 1500Hz 符号出现时误累加, 导致接收中途 LED 熄灭 →
帧被掐断.

**修改 1 — True SNR 分类器** (`voice_dsp.c` VoiceDSP_Classify):
旧判据 `best/second ≥ 1.6` 在宽带噪声下四个 bin 同时抬高 → 比值偶然越过门限 →
噪声误判为有效符号. 新判据 True SNR = P_signal / (P_total - P_signal):
- P_total = Σ(x[i]-x̄)² (时域方差, 全频段能量)
- P_signal = max Goertzel mag² × 2/N (最强载波, 换算到方差单位)
- α = 2/N = 0.0125 (N=160, 整数 bin 精确对齐)
- 门限: raw_mag² ≥ 200,000 且 SNR ≥ 2.0 (6dB)

**修改 2 — 低能量门限 + 双重频域锁** (`voice_dsp.c`):
VD_EN_MARGIN 1000→500, VD_EN_MIN 800→500 (弱信号放进门), 但进场后须连续
2 次 Goertzel 命中同一导频 (1500/2400Hz) 才放行. 噪声频谱平坦 → 不可能连续命中.

**修改 3 — 频域擦除计数替代 lo_run** (`voice_dsp.c`):
移除 VD_PREAMBLE 的 `lo_run>=20U` 退出 (仅留 700ms 硬超时) 和 VD_DATA 的
`lo_run>=100U` 退网. 改为 `data_store_symbol` 中连续 4 符号 Goertzel 返回 0xFF
才判信号丢失. 瞬时擦除交给 Hamming(7,4) 纠错.

**修改 4 — LiveWatch 诊断接口** (`receiver.c/h`):
新增 RX_GetPilotHits/GetEraseRun/GetLastSNR/GetVGain/GetDspSubState 五个 getter.

**修改文件**: voice_dsp.h (+pilot_hits/erase_run, +宏), voice_dsp.c (True SNR +
双重锁 + 擦除计数), receiver.c/h (诊断接口).

### 2026-07-15 — 状态驱动冻结式 AGC (突发模式)

修复两个 AGC 致命缺陷:

**缺陷 1 — 待机致聋死锁**: `pga112.c` 中 RMS 过高降增益逻辑无 `frame_active` 保护.
待机时突发巨响触发降增益, 随后 `frame_active==0` 阻止回调 → 永久卡死在低增益.
修复: VD_LISTEN 期间不调用 `PGA112_AGC_Update`, 直接死锁在 32x.

**缺陷 2 — 增益突变撕裂 Goertzel 频谱**: 每 5ms 无条件调增益, 数据段中增益跳变
引入阶跃信号 → 全频段能量泄露 → 次强幅度飙升 → 置信度崩溃 → 符号大面积擦除 0xFF.
修复: VD_DATA 期间完全冻结增益, 禁止任何 AGC 调节.

**修改文件**: `Core/Src/receiver.c` — `RX_ProcessHalfBuffer` 新增状态驱动 AGC 调度:
- `VD_LISTEN`: 锁死 32x, 若偏离则强制复位
- `VD_PREAMBLE` 前期 (pilot_trans < 2): 激活 AGC, 允许升至 128x
- `VD_PREAMBLE` 后期 + `VD_DATA`: 跳过 AGC, 绝对冻结增益

### 2026-07-15 — 帧尾 30ms DC 保护槽 (方案一)

**问题**: 最后一个 CRC 符号频频收不到. 根因: 最后一个符号 guard 结束后零间隙进入
2400Hz postamble, postamble 能量通过声学拖尾/房间混响渗入 Goertzel 判决窗,
且 postamble 频率恰好是 digit 3 (2400Hz), 拉高次强 bin → 置信度崩溃 → 符号擦除.

**修复**: `transmitter.c` ST_DATA 最后符号之后插入 30ms (480 tick) DC 保护槽,
将 postamble 与最后一个符号的 Goertzel 窗物理隔离.

**时序变化**:
```
旧: [最后 tone 20ms] [guard 10ms] → [postamble 2400Hz 120ms]  (零间隙)
新: [最后 tone 20ms] [guard 10ms] [DC 30ms] → [postamble 2400Hz 120ms]
```

接收端无需任何改动: 它在收到 `sym_expected` 个符号后已进入 DONE, 额外的 30ms 对
接收端完全透明. 最坏 48 字符: 总帧长 ~11.93s, 仍在 20s 预算内.

### 2026-07-14 — v5 收发协同重构 (针对 DSP 复核 5 缺陷)

半双工同时含发送与接收, 两端均迁移到 v5 (与单工 01/02 逐字节共享 transmitter.c /
receiver.c / voice_fec.c / voice_dsp.c):
- **发送**: 变长帧 `[前导][1800Hz同步音][LEN三重冗余][payload+CRC8]`, Hamming(7,4)+交织.
- **接收**: 前导+同步音一次锁定 30ms 栅格 (免疫混响), 频谱置信度判决 (无绝对幅值门限,
  提升距离), 差分能量高通检测 (免疫 50Hz/DC), ±1 bin 频偏容忍, FEC 抗突发.
- **公开 API / 顶层半双工状态机 (HM_RX/HM_TX_EDIT/HM_TX_BUSY) / 外设互斥启停 / 双 LED /
  Flash / 中文启动页 / 软开关全部不变**: main.c 无需改动.
- **CMake**: 新增 `voice_fec.c`/`voice_dsp.c`. 引脚/定时器 (TIM1/2/3) 无变化.
- PC 端 (gcc) 编译 + 链接 + FEC 单测 + 信道仿真 + 收发全链路均通过; **待真机烧录 (两板对调) 声学实测**.

### 2026-07-14 (补3) — PGA112 数据手册核对 + 命令修正

对照 pga112.pdf (TI SBOS424C) 核对协议, 修正一处错误:
- **删除伪造的 reset 命令 0x20** (手册中不存在). 上电 POR 后增益/通道寄存器本为
  全 0 (增益=1, 通道=VCAL/CH0), 无需 reset. Init 改为发 SDN_DIS (0xE1 0x00) 退出
  关机模式再写增益.
- **WRITE 命令确认**: 高字节 0x2A, 低字节 = [G3G2G1G0(高4)] | [CH3CH2CH1CH0(低4)].
- **增益码确认**: binary 增益 0..7 = 1/2/4/8/16/32/64/128; 32x = 码 5.
- **SPI 时序确认**: Mode 0 (CPOL=0, CPHA=0); CS 低有效; 与 CubeMX 现有 SPI2 配置完全一致.

### 2026-07-14 (补4) — 进门严格化 + 灯只在数据段亮 (消除噪声误唤醒/乱闪)

**改动1 (voice_dsp.c)**: 连续 6 块高能量后, 追加要求最近 160 样本经 Goertzel 分类
必须为 1500Hz 或 2400Hz 导频音, 才进 PREAMBLE.
**改动2 (receiver.c)**: 点灯条件从"PREAMBLE 或 DATA"改为"仅 DATA".

### 2026-07-11 — 提高②③⑥⑦：半双工、可靠接收与低功耗

- **PGA112 AGC**: 第二级放大改用 PGA112 可编程增益放大器, SPI2 控制.
  - 削顶保护 (触及量程两端) → 即时降一档
  - RMS 双门限 + Vpp 辅助判定
  - 增益档 0..7 = 1/2/4/8/16/32/64/128 倍
- **半双工资源互斥**：RX 进入编辑/发送前统一停止 ADC DMA、TIM2 与接收状态机.
- **可靠性**：接收端增加背景噪声 IIR 基线和自适应包络门限；Goertzel 改用每窗口动态直流均值.
- **Flash 存储迁移**: 从内部 Flash Sector 3 迁移到外部 PY25Q64 SPI Flash.

### 2026-07-10 — 多终端通信 + 开机设置设备编号 + 硬件锁存软开关

- 支持 ID 1~9、任意多选目标及广播.
- 收件人页：1~9 切换目标，0 广播，发送确认.
- 开机动态选择设备编号 (SelectDeviceId).
- PB1=POWER_BUTTON (EXTI1), PB8=POWER_CTRL (硬件锁存).
- BOR 防重启等待, 开机按键松手检测 + 50ms 防抖.
- EXTI 回调立即解除电源锁存；主循环停止所有外设后等待硬件掉电.

### 2026-07-08 (CubeMX 重生成后修复)

- **GPIO 重分配**: 键盘矩阵改为全部 GPIOA (PA0~PA3 rows, PA9~PA12 cols).
- keyboard.c 移除 `col_ports[4]`，恢复单 `COL_PORT` 宏.
- 修复 LED 初始态 `GPIO_PIN_RESET` → `GPIO_PIN_SET`.
- 清理 receiver.c `rx_enter_error` 死代码注释.

### 2026-07-07 — 从单工合并修复

- 新增 flash_store.c/h (当时为内部Flash非易失存储, 5条循环缓冲).
- 修复 LED 初始状态 GPIO_PIN_RESET→GPIO_PIN_SET (上电熄灭).
- 修复 RX Done 处理顺序 (先显示"Rx Complete"2s → 再清除重启).
- 新增 RX 子模式 (LS_LISTENING/BROWSE_LIST/BROWSE_VIEW, KEY_FN切换).
- 新增 FlashStore_Init 初始化调用.
- 新增 receiver.c 自动保存 (rx_enter_done → FlashStore_SaveMessage).

---

### 2026-07-16 (v5.1 重大升级) — 多窗口 Goertzel + 自适应 SNR + 软判决 FEC + ACK 协议

**方案1 — 多窗口 Goertzel 累加** (oice_dsp.c):
- 取满 20ms tone (320 samples), 做 3 个重叠 Goertzel 窗 (offset 40/80/120, N=160).
- 累加 3 窗 mag² 再做 True SNR 判决 → ~4.8dB SNR 增益.
- win_buf 从 160 扩展到 320 samples. VD_WIN_OFFSET 从 80 改为 0.
- 新增 VoiceDSP_ClassifyMulti() 函数.

**方案2 — 软判决 FEC 解码** (oice_fec.c):
- LUT 查表法 LLR: 256 项 ln(x) 预计算表, 覆盖比率 1.0~100.6.
- 4-FSK digit→bits 映射: bit0 用 mag[1]+mag[3] vs mag[0]+mag[2], 类似 bit1.
- Chase 软 Hamming 解码: 找 2 个最不可靠 bit, 试 4 种翻转组合, 取最小软距离.
- 新增 VoiceFEC_ParseDataSymbolsSoft() 接口.
- VoiceRx 结构体新增 sym_mag2[385][4] 数组 (6160B).
- 老 ParseDataSymbols 回退到软判决内部实现.

**方案5 — 半双工 ACK 协议** (	ransmitter.c + main.c):
- 发送端: 新增 ST_ACK 状态, TX_SendAck() 发 100ms 1500Hz 纯音.
- 接收端: 收到有效帧后自动回复 ACK.
- 发送端等待: TX_DONE → 切换 RX → VoiceDSP_Classify 检测 1500Hz (3 次连续命中).
- 500ms 超时 → "No ACK", 否则 "Sent OK!".
- 新增 TX_IsAckDone() 查询接口.

**方案7 — 自适应 SNR 门限** (oice_dsp.c):
- VD_PREAMBLE 期间累加每次 Goertzel 的 SNR 值.
- 进入 VD_DATA 时: data_snr_threshold = max(VD_SNR_MIN, preamble_mean_snr × 0.5).
- 强信号从严, 弱信号从宽.
- VoiceRx 新增 preamble_snr_sum, preamble_snr_count, data_snr_threshold 字段.

**修改文件清单**:
| 文件 | 改动 |
|------|------|
| voice_dsp.h | +VD_TONE_SAMPLES, +sym_mag2, +自适应SNR字段, +VoiceDSP_ClassifyMulti |
| voice_dsp.c | 完整重写: 多窗口+自适应SNR+mag2输出+数据填满20ms |
| voice_fec.h | +VoiceFEC_ComputeLLR, +VoiceFEC_ParseDataSymbolsSoft |
| voice_fec.c | +256项 LLR LUT, +Chase软解码, +LLR反交织 |
| transmitter.h | +TX_SendAck, +TX_IsAckDone |
| transmitter.c | +ST_ACK状态, +TX_SendAck实现 |
| main.c | +voice_dsp.h include, +ACK发送(RX侧), +ACK等待(TX侧) |

**编译结果**: FLASH 63556B (+14KB), RAM 17960B (+11KB). 0 error, 0 warning.

### 2026-07-19 — 无 AGC v5.1 修复同步到带 AGC 分支

- 软判决按交织索引逐码字解码，修复帧末越界，DMA 路径栈用量由 7016B 降至 200B。
- 同步音使用 5ms 整数 bin 窗口定位；自适应 SNR 门限钳制在 2.0..20.0，避免纯净前导错误抬高门限。
- PGA112 在进入数据段后锁定写入，SPI 两字节传输期间屏蔽 DMA 回调，避免数据 Goertzel 窗内发生增益阶跃。
- 监听阶段检测到 PGA 档位变化时按档位比例同步差分能量噪声底；ADC DMA 启动前先完成 RX/PGA 初始化。
- RX/TX 模式重启强制回到 16x 时保留切换前档位，首个采样块再同步噪声底，避免跨帧门限使用旧增益标尺。
- 验证: ARM Debug 构建通过；FEC 1..51 字节往返与清洁帧 DSP 同步/CRC 仿真通过。


## 七、待完成

- [ ] 硬件联调: 半双工模式切换 (RX↔TX) 无外设资源冲突
- [ ] 端到端收发测试 (两板对调)
- [ ] PA0 一键开关机 Standby 唤醒电流 ≤1mA
- [ ] 突发模式 AGC: 远距离弱信号接收距离提升验证
- [ ] 数据段增益冻结后符号擦除率是否显著下降
- [ ] CRC 符号丢失率是否下降 (30ms 保护槽效果)
- [ ] True SNR 对宽带噪声(风扇/空调)的免疫验证
- [ ] 前导双重频域锁: 误唤醒率为零的验证
- [ ] PGA112 16x 初始增益实机验证
- [ ] PY25Q64 SPI Flash 读写稳定性验证
- [x] ~~CMakeLists.txt 移除死代码~~ fsk4_decoder.c + fsk16_encoder.c (保留: 仍有编译引用, 暂不删除)
- [ ] receiver.h 清理 v4 遗留宏
