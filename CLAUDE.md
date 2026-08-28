# 声语信使 (Voice Messenger) — 半双工终端 (STM32F411CEU6)

本文档为「声语信使」半双工项目的**当前**权威说明。同一块板既能发信也能收信，但不能同时进行（半双工）。
本分支 (`报告用`) 为**固定增益**版本：接收链路不含 AGC（PGA112 编程增益已弃用，详见「硬件已知问题」）。

---

## 1. 项目概述

- **MCU**: STM32F411CEU6（核心板）+ 自绘扩展板
- **通信介质**: 空气声波，4-FSK 音频（1.5~2.4 kHz）
- **模式**: 半双工（RX ↔ TX 软件互斥切换）
- **功能**: 多终端组网（1~9 号 + 广播）、≤48 字符消息、T9 编辑、OLED 显示、
  Flash 非易失存储、一键软开关机、双 LED 指示
- **协作约定**: 软件手（本仓库维护者）负责 `stm32f411ceu6` 的 MCU 代码；
  在每次回答之后更新 `PROGRESS.md`。

---

## 2. 硬件配置（与原理图 / `.ioc` 一致）

### 2.1 时钟

```
HSE 25MHz → PLL (M=25, N=100, P=2) → SYSCLK = 50MHz
APB1 = APB2 = 50MHz (Div1), FLASH_LATENCY_1
```

（注意：不是 HSI 16MHz，是 HSE 25MHz。）

### 2.2 供电 / 开机

- **vbat = 5V**（实测 4.9V+），经**一键软开关**（MOS 软开关：Q3 AO3401A + Q2 MMBT3904）锁存供电；
  `PB8 = POWER_CTRL`（高=锁存上电），`PB1 = POWER_BUTTON`（EXTI1 下降沿，开机按键）。
- `+5V → AMS1117-3.3 → VCC/3.3V`。
- 监听空闲 20s 自动熄屏 OLED；待机用 `__WFI()`。

### 2.3 引脚分配

| 功能 | 引脚 | 模式 | 说明 |
|------|------|------|------|
| KB_ROW0~3 | PA0~PA3 | Output PP | 矩阵键盘行扫描 |
| KB_COL0~3 | PA9~PA12 | Input PU | 矩阵键盘列读取 |
| PWM 音频输出 | PA8 | AF1 PP | TIM1_CH1, 48.83kHz 载波 → 有源低通 → PAM8403 |
| 音频输入 | PB0 | Analog | ADC1_IN8（模拟前端放大→带通→ADC）|
| OLED SCL / SDA | PB6 / PB7 | AF4 OD | I2C1 400kHz, SSD1306 128×64 |
| 指示 LED | PB2 = LEDG | Output PP | 接收指示/待机 |
| 指示 LED | PB10 = LEDR | Output PP | 收到数据指示 |
| 板载 LED | PC13 | Output PP | 已弃用（固定熄灭）|
| 电源锁存 | PB8 | Output PP | POWER_CTRL（高=上电锁存）|
| 电源按键 | PB1 | EXTI1 下降沿 | POWER_BUTTON（上拉）|
| 外部 Flash | PA4=F_CS, PA5=SCK, PA6=MISO, PA7=MOSI | SPI1 主机 | PY25Q64（8MB）|
| （AGC 预留）| PB12=CS, PB13=SCK, PB15=MOSI | — | SPI2 驱动 PGA112，**当前分支未初始化** |

### 2.4 发送链路

`PA8 (PWM-DDS) → 有源低通 (OPA1642, fc≈2.5kHz) → PAM8403 (D 类, 单声道左声道 BTL) → 喇叭`

### 2.5 接收链路

`麦克风 (GMI9745P) → OPA1642 两级固定增益 → OPA1642 带通 (1.5~2.5kHz) → 0Ω 跳线 → PGA112（可选 AGC，当前绕过）→ PB0 (ADC1_IN8)`

---

## 3. 定时器

| 定时器 | 用途 | PSC | ARR | 频率 |
|--------|------|-----|-----|------|
| TIM1 | PWM 载波 (CH1, PA8) | 0 | 1023 | 48.828kHz |
| TIM2 | ADC 触发 (TRGO, 16kHz) | 4 | 624 | 16.0kHz |
| TIM3 | 发送状态机 tick (16kHz) | 4 | 624 | 16.0kHz |

ADC：12-bit，28 周期采样，TIM2_TRGO 上升沿触发，**DMA2_Stream0 循环** 800 halfwords
（半缓冲 HT/TC 各 400 样本 = 25ms 回调）。

---

## 4. 协议（v5.1，收发共享）

### 4.1 帧结构（变长）

```
[前导 2000ms 1500/2400Hz 每 40ms 交替]
[同步 1800Hz 30ms 单音]          ← 唯一精定时锚点
[LEN 三重冗余 21 符号]           ← payload 长度, 逐 bit 多数表决
[payload + CRC8]                 ← Hamming(7,4) + 块交织 + Chase 软解码
[30ms DC 隔离槽]
[结束 2400Hz 120ms]
```

- **符号时序**: 20ms 载波 + 10ms 保护 = 30ms 槽 = 480 样本 @16kHz。
- **4-FSK 频率**: 1500 / 1800 / 2100 / 2400 Hz（digit 0~3）。
- **符号率/容量**: 每字符 → 每字节 FEC 编码 → 2bit/符号；最坏 48 字符 ≈ 11.5s 数据。
- **FEC**: Hamming(7,4)（每码字纠 1bit）+ 块交织 + CRC-8 (poly 0x07) + LEN 三重冗余；
  软判决 LLR + Chase 翻位软解码（相对硬判决约 +2dB）。
- **载荷**: `[source_id][mask_lo][mask_hi][text...]`，无填充，len ≤ 51 字节（48 字符）。

### 4.2 接收 DSP（voice_dsp.c）

- 前导 1500/2400 交替序列（≥8 次）捕获 → 1800Hz 同步音**一次性锁定 30ms 符号栅格**。
- 数据段**每 20ms tone 取 3 个重叠 160 点窗累加** Goertzel（±1bin 容忍），频谱置信度判决。
- 唤醒用**一阶差分能量**（免疫 DC/50Hz）；判决无绝对幅值门限（提升弱信号距离）。
- 1.1~2.8kHz 流式带通预处理（1 高通 + 3 级低通），1500Hz 频响补偿权重 1.33。

---

## 5. 顶层状态机

```
上电 → 开机选择设备号(1~9) → HM_RX (默认接收)
HM_RX（含子模式 LS_LISTENING / LS_BROWSE_LIST / LS_BROWSE_VIEW）
  ├─ KEY_FN      切到 HM_TX_EDIT（或退出浏览）
  ├─ KEY_SEND    浏览已存消息（/ 回退浏览）
  ├─ 数字键      进入 HM_TX_EDIT 并输入
  └─ 收完成    显示 2s → 自动存 Flash → 重启采样
HM_TX_EDIT
  ├─ KEY_SEND    → HM_TX_SELECT（选收件人 1~9/0 广播）
  ├─ KEY_FN      → HM_RX
  └─ 光标末尾+KEY_RIGHT → HM_RX（"rx" 快速切回）
HM_TX_SELECT → 收件人多选 → HM_TX_BUSY（发送）
HM_TX_BUSY  → TX done → 回 HM_TX_EDIT（保留编辑内容）
切换时: RX↔TX 互斥启停 ADC+DMA+TIM2 ↔ TIM1+TIM3
```

---

## 6. 源文件

| 文件 | 职责 |
|------|------|
| `Core/Src/main.c` | 顶层半双工状态机、UI 子模式、RX/TX 外设互斥、软开关、设备号/收件人 |
| `Core/Src/transmitter.c` | v5 发送状态机（TIM3 16kHz ISR 驱动）|
| `Core/Src/pwm_dds.c` | PWM-DDS 正弦生成（1024×10bit LUT, 32bit 相位累加）|
| `Core/Src/receiver.c` | 接收公开 API、地址过滤、显示/滚动缓冲 |
| `Core/Src/voice_dsp.c` | 接收 DSP 核心（带通 + Goertzel 多窗 + 状态机）|
| `Core/Src/voice_fec.c` | Hamming(7,4)+交织+CRC8+Chase 软解码 |
| `Core/Inc/voice_proto.h` | 协议常量（帧/时序/FEC 尺寸）|
| `Core/Inc/network_protocol.h` | 设备号/目标掩码/广播 |
| `Core/Src/flash_store.c` | 外部 Flash 消息存储（64 条环形）|
| `Core/Src/py25q64.c` | PY25Q64 SPI Flash 驱动 |
| `Core/Src/oled.c` | SSD1306 驱动（5×7 字体）|
| `Core/Src/keyboard.c` | 4×4 矩阵键盘 |
| `Core/Src/editor.c` | T9 文本编辑器 |
| （遗留）`fsk4_encoder.c` | v4 编码器（仅提供 DDS phase_inc 表）|
| （遗留）`fsk16_encoder.c` / `fsk4_decoder.c` | 未使用 |

---

## 7. 信息存储（外部 SPI Flash）

- 芯片 **PY25Q64**（8MB），SPI1（PA4~PA7），JEDEC 0x852017 校验。
- 最多 **64 条**消息，环形缓冲，满时淘汰最旧。
- **双副本 v3**：`STORE_COPY_A=0x000000` / `STORE_COPY_B=0x001000`，写入后回读 CRC 校验，
  掉电保留上一份有效数据；旧内部 Flash Sector 布局可自动迁移。
- 接收完成且校验通过后自动保存；`FlashStore_Init()` 在 `main()` 初始化阶段加载。

> 注意：若核心板实际为 W25Q64（JEDEC 0xEF4017），`PY25Q64_Init` 返回失败 →
> 存储会被**静默禁用**（`store_ready=0`），消息不保存。请核对核心板 Flash 型号。

---

## 8. 硬件已知问题（重要）

1. **PGA112 VREF 违反数据手册**（AGC 失败根因）：
   `U9 VREF = R4(100K)→vbat + C36(10µF)→GND`，无低阻基准。
   TI 手册要求 VREF 接“**DC+AC 双低阻、拉灌 ≥2mA**”的基准或直接接地；100K 只能供 ~50µA，超差 ~40 倍。
   且 vbat=5V → VREF≈5V=AVDD → VOUT 零正裕量 + 直流 5V 超 3.3V ADC 绝对最大额定。
   **后果**：`with-agc` 分支的 AGC（`pga112.c`，SPI2 PB12/13/15）在真机失败。
   **修法**：用空闲 OPA1642 缓冲 `VCC/2=1.65V` 低阻基准接 VREF（10K/10K 分压 + 运放跟随 + 1~10µF）。
2. **PAM8403 单声道**：右声道弃用（OUTR+/OUTR− 悬空，右声道 PVDD(13) 接地以关断它）；左声道 BTL 接喇叭。
3. **PAM8403 VREF 旁路 = 0.1µF**（手册推荐 0.47µF，轻微）。
4. **MUTE#/SHDN# 悬空**：无静音/关断控制（轻微，建议 SHDN# 接 VDD、MUTE# 接 GND 或各接 GPIO）。
5. **接收链路中点裕量**：运放跑 5V、中点 2.5V，ADC 3.3V → 正向裕量仅 0.8V（正常信号无碍，
   麦克风贴喇叭会削顶；前端 C31 10µF 耦合 + R25 20K 偏置缓解）。

---

## 9. 分支说明

| 分支 | 说明 |
|------|------|
| `报告用`（当前）| 固定增益版本，无 PGA112 驱动 |
| `with-agc` | 含 `pga112.c`（SPI2 驱动 + AGC 控制律），因硬件 VREF 问题未成 |
| `without-agc` / `main` / `last` / `DAC_MCP4921` | 其他历史/备用分支 |

---

## 10. 构建

```sh
cmake --preset Debug
cmake --build --preset Debug --parallel
```

需要 `arm-none-eabi` 工具链（当前开发沙箱未安装，未生成 ELF，需本地/真机构建烧录）。

---

## 11. 待办 / 已知限制

- 端到端声学收发标定（唤醒门限、导频识别、远距离误码率）——真机验证。
- 半双工模式切换（RX↔TX）外设资源冲突检查。
- 软开关 WAKEUP 电流 ≤1mA 验证。
- 接收链路门限（VD_EN_MIN 等）按固定增益链路标定，换硬件需重新标定。
- DSP 全链路验证基于 PC 理想信道仿真，真机实测仍需继续。
