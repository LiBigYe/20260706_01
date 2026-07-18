# AGENTS.md — 声语信使 半双工 (项目03) 代码实际描述
# 最后更新: 2026-07-18
# 原则: 以代码为准, 本文档必须在每次代码变更后同步更新

# 项目03 — 半双工 (Half-Duplex)

本目录为声语信使系统的半双工终端代码. 同一块板既可发信也可收信, 但不能同时进行.

## 硬件配置

- MCU: STM32F411CEU6 (**HSE 25MHz 外部晶振** → PLL 50MHz SYSCLK, CSS 已使能)
- TIM1_CH1 (PA8): PWM DDS 音频输出 → RC 低通滤波
- **TIM2**: 16kHz ADC 触发 (**TRGO_UPDATE**), **PSC=4, ARR=624**
- **TIM3**: 16kHz DDS 采样时钟, **PSC=4, ARR=624**
- ADC1_IN8 (PB0): 音频输入 (经 PGA112 第二级放大 + 模拟前端 → 偏置到 1.65V)
- DMA2_Stream0: 循环 DMA 传输 ADC→缓冲 (800 halfwords)
- I2C1 (PB6/PB7): OLED 128x64
- **4x4 键盘**: **PA0~PA3 rows, PA9~PA12 cols**
- **外部 SPI Flash**: PY25Q64 8MB (SPI1: PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI)
- **PGA112**: 第二级可编程增益放大器 (SPI2: PB12=CS, PB13=SCK, PB15=MOSI, 只发不收)
- **电源管理**: PB1=独立电源键 (EXTI1 下降沿), PB8=电源锁存 (POWER_CTRL)

## 引脚分配

| 功能 | 引脚 | 模式 | 说明 |
|------|------|------|------|
| KB_ROW0~3 | PA0~PA3 | Output PP | 矩阵键盘行扫描 |
| **KB_COL0~3** | **PA9~PA12** | Input PU | 矩阵键盘列读取 |
| PWM 音频输出 | PA8 | AF1 PP | TIM1_CH1, 48.83kHz → RC 低通 |
| 音频输入 | PB0 | Analog | ADC1_IN8 |
| OLED SCL | PB6 | AF4 OD | I2C1 400kHz |
| OLED SDA | PB7 | AF4 OD | I2C1 400kHz |
| LED (PC13) | PC13 | Output PP | 收发状态指示 |
| LEDG (绿) | PB2 | Output PP | 外部下拉, 高电平点亮; 仅发送期间亮 |
| LEDR (红) | PB10 | Output PP | 外部下拉, 高电平点亮; 仅数据解码期间亮 |
| POWER_BUTTON | PB1 | Input PU + EXTI1 | 独立电源键 (下降沿) |
| POWER_CTRL | PB8 | Output PP | 电源锁存输出 |
| F_CS | PA4 | Output PP | PY25Q64 SPI Flash 片选 |
| PG112_CS | PB12 | Output PP | PGA112 片选 (SPI2) |

## 源文件结构

**活跃业务源文件 (13个, CMakeLists.txt 中编译；`main.c` 由 CubeMX 子项目加入)**:

| 文件 | 说明 |
|------|------|
| `Core/Src/main.c` | 主程序, 半双工状态机 (HM_RX/HM_TX_EDIT/HM_TX_SELECT/HM_TX_BUSY), UI子模式 |
| `Core/Src/transmitter.c` | v5 发送状态机 (变长帧 + FEC + 同步音 + DC保护槽) |
| `Core/Src/fsk4_encoder.c` | 4-FSK编码器 (仅用于 DDS 相位增量计算 `FSK4_GetPhaseInc()`) |
| `Core/Src/pwm_dds.c` | PWM DDS 正弦波生成 (1024-pt × 10-bit LUT) |
| `Core/Src/receiver.c` | v5 接收状态机 + 状态驱动 AGC + voice_dsp 接口 |
| `Core/Src/voice_dsp.c` | v5.1 接收 DSP 核心 (True SNR + 双重频域锁 + 擦除计数) |
| `Core/Src/voice_fec.c` | Hamming(7,4) + 交织 + CRC-8 (收发共用, 纯逻辑无HAL依赖) |
| `Core/Src/flash_store.c` | 外部 PY25Q64 SPI Flash 消息存储 (64条, A/B双副本) |
| `Core/Src/py25q64.c` | PY25Q64 SPI NOR Flash 驱动 (SPI1) |
| `Core/Src/pga112.c` | PGA112 可编程增益放大器 + AGC (SPI2) |
| `Core/Src/oled.c` | SSD1306 OLED驱动 (I2C, 5x7字体) |
| `Core/Src/keyboard.c` | 4x4矩阵键盘扫描 |
| `Core/Src/editor.c` | T9 文本编辑器 |

**死代码 (编译但未使用, 可从 CMakeLists.txt 移除)**:

- `Core/Src/fsk4_decoder.c/h` — v4 Goertzel 解码器 (N=320), 已被 voice_dsp.c 取代
- `Core/Src/fsk16_encoder.c/h` — 16-FSK 编码器 (预留, 从未使用)

## 半双工模式切换

```
上电 → SelectDeviceId (选择设备编号 1~9)
      → HM_RX (默认接收)
         │ KEY_FN → HM_TX_EDIT
         │ T9键 → HM_TX_EDIT
         │ KEY_SEND → 浏览已存储消息 (LS_BROWSE_LIST)
         │   ├─ LEFT/RIGHT → 选择消息
         │   ├─ 数字键 → 查看消息 (LS_BROWSE_VIEW)
         │   ├─ DELETE → 删除选中
         │   └─ KEY_FN → 退出浏览 (LS_LISTENING)
         │
         HM_TX_EDIT:
         │ KEY_SEND → HM_TX_SELECT (收件人选择)
         │ KEY_FN → HM_RX
         │ KEY_RIGHT (光标在末尾) → HM_RX (快速切回收信)
         │
         HM_TX_SELECT:
         │ 1~9 → 切换目标设备 (多选)
         │ 0 → 广播
         │ KEY_SEND → HM_TX_BUSY (开始发送)
         │ KEY_FN/DELETE → HM_TX_EDIT (取消)
         │
         HM_TX_BUSY → done → HM_TX_EDIT (保留编辑器内容)
```

切换时: RX↔TX 互斥启动/停止 ADC+DMA+TIM2 / TIM1+TIM3.

## TIM 参数

| 定时器 | 用途 | PSC | ARR | 频率 |
|--------|------|-----|-----|------|
| TIM1 | PWM 载波 (CH1, PA8) | 0 | 1023 | 48.83kHz |
| **TIM2** | **ADC 触发 (TRGO_UPDATE)** | **4** | **624** | **16kHz** |
| TIM3 | DDS 时钟 + ISR | 4 | 624 | 16kHz |

## v5.1 协议帧结构

```
[Preamble 200ms: 1500/2400Hz 每40ms交替] [SYNC 30ms: 1800Hz 20ms + DC保护10ms]
[LEN ×3 冗余] [Payload + CRC8: Hamming(7,4)+交织]
[DC保护槽 30ms] [Postamble 120ms: 2400Hz]
```

- **变长帧**: payload = [source_id(1B)][mask_lo(1B)][mask_hi(1B)][text...(≤48B)]
- **FEC**: Hamming(7,4) 编码, 每字节→2码字→14bit→7符号. 块交织分散突发错误.
- **CRC-8**: poly 0x07, 覆盖 LEN + payload.
- **LEN 三重冗余**: 3 份独立 Hamming 块, 逐bit多数表决.
- **符号时序**: 每符号 20ms tone + 10ms guard (DC) = 30ms 槽.
- **4-FSK 频率**: 1500/1800/2100/2400 Hz (digit 0~3).
- **DC 保护槽**: 最后数据符号后插入 30ms DC, 将 postamble 2400Hz 与 CRC 符号 Goertzel 判决窗物理隔离.
- 最坏 48 字符: ~11.93s, 在 20s 预算内.

## OLED 布局

- 第0~6行: 文本区 (支持 \n 换行, 屏幕宽度自动换行, 末尾7行自动滚动)
- 第7行: 状态栏 — 左侧 Count:XX或XX/XX[模式], 右侧状态文字
- **20 秒无操作自动息屏** (`oled_idle_timeout_ms = 20000`)

## 信息存储 — 外部 SPI Flash (PY25Q64)

使用 **外部 PY25Q64 SPI NOR Flash** (8MB, SPI1: PA4=CS, PA5=SCK, PA6=MISO, PA7=MOSI)
存储最多 **64 条消息**, A/B 双副本 + CRC32 + generation 版本仲裁.

**Flash 布局** (每副本 1 个 4KB sector):

```
0x000000: Copy A
  Word 0: Magic 0x564F4943 ("VOIC")
  Word 1: Version(16) | count(16)
  Word 2: generation(32)
  Word 3: crc32(32)
  Words 4+: 64 × FlashStore_MsgSlot (valid, source_id, length, data[50])

0x001000: Copy B (结构相同, 交替写入, generation 递增仲裁)
```

**写入策略**: 每次保存/删除 → 擦除对侧 sector → 全量重写 → 读取验证 CRC.
循环缓冲: 满 64 条时淘汰最旧 (FIFO).
**写入前暂停 ADC DMA** 以避免 SPI Flash 擦写期间的总线冲突.

## UI 操作表

| 按键 | 物理标签 | RX监听 | RX浏览列表 | RX浏览消息 | TX编辑 | TX发送中 |
|------|---------|--------|-----------|----------|--------|---------|
| 0~9 | 数字 | →TX编辑 | 查看选中 | — | T9输入 | — |
| ← | 左移 | 向上滚行 | 选择上一条 | 向上滚 | 左移光标 | — |
| → | 右移 | 向下滚行 | 选择下一条 | 向下滚 | **光标在末尾→切回收信** | — |
| 删除 | 删除 | — | 删除选中 | 退回列表 | 退格 | — |
| 英/数 | KEY_FN | →TX编辑 | 退出浏览 | 退出浏览 | →接收 | — |
| 发送 | KEY_SEND | 浏览已存 | 浏览选中 | 退回列表 | →收件人选择 | — |

## PGA112 第二级可编程增益

- SPI2: PB12=CS, PB13=SCK, PB15=MOSI (Simplex_Bidirectional_Master, Mode0/MSB/8bit)
- **当前初始增益: 16x** (PGA_GAIN_INIT_CODE = 4)
- 增益档 0..7 = 1/2/4/8/16/32/64/128 倍
- PGA112 命令为 16bit: 写增益 `0x2A, (gain_code << 4)`, 退出软件关断 `0xC2, 0x00`。
- 增益写入分两步: DMA 回调中仅 `PGA112_RequestGain()`，主循环中的 `PGA112_Service()` 再执行 SPI 轮询，避免 SPI 超时阻塞采样处理。
- `PGA112_SetGain()` / `PGA112_Init()` 返回 HAL 状态；`PGA112_GetLastStatus()` 与 `PGA112_GetErrorCount()` 供诊断。SPI 失败的异步请求以 10ms 间隔重试。PGA112 未接 MISO，`HAL_OK` 只能证明 STM32 SPI 已发送，不能证明芯片物理接收。
- AGC 以 ADC 峰峰值控制，目标死区为 1120..2234 counts（约 0.9..1.8Vpp，VDDA=3.3V 时）。同一 5ms 块内至少 3 个近轨采样才触发即时降档。
- AGC 调度策略 (receiver.c `RX_ProcessHalfBuffer`):
  - **VD_LISTEN 静默**: 保持 16x；退出前导后恢复初始增益并清空 AGC 计数器。
  - **VD_LISTEN 候选信号**: 连续 6 个高能量块后，仅允许上调至 64x，帮助弱导频通过频域锁，避免静态底噪爬升。
  - **VD_PREAMBLE**: 前 4 次导频交替内闭环调节，可在 16x..128x 间升降；之后冻结，给同步音保留约 40ms 稳定时间。
  - **VD_DATA**: 取消未执行的前导写请求并冻结增益，保证数据段 Goertzel 窗内没有增益阶跃。

## v5.1 DSP 核心 (voice_dsp.c)

- **True SNR 分类器** (`VoiceDSP_Classify`):
  - `SNR = P_signal / (P_total - P_signal)`
  - N=160 (10ms 中间窗), 整数 k 精确定位 1500/1800/2100/2400Hz
  - α = 2/N = 0.0125 (mag²→方差单位)
  - 门限: raw_mag² ≥ 200,000 且 SNR ≥ 2.0 (6dB)
  - 频响补偿权重: {1.33, 1.08, 1.00, 1.02} (消除模拟前端通带不平坦)
- **双重频域锁**: 差分能量门限 500, 进门后连续 2 次 Goertzel 命中同一导频 (1500/2400Hz) 才进 PREAMBLE
- **频域擦除计数**: 连续 4 符号 Goertzel 返回 0xFF 才判定信号丢失 (替换 lo_run)
- **LiveWatch 诊断接口**: `RX_GetPilotHits/GetEraseRun/GetLastSNR/GetVGain/GetAGCVpp/GetPGAErrorCount/GetDspSubState`；`RX_GetLastSNR()` 返回线性值而非 dB。
- 多窗口数据 SNR 的信号能量与总能量均按相同的三个重叠 160-sample 窗累加，避免分子和分母标尺不一致。
- 软判决 LLR 使用归一化尾数 LUT + `ln(2)` 倍频，覆盖 1:1 到 100:1；4-FSK 的 LLR 位序与 MSB/LSB 硬判位序一致。

## 开机流程

1. 寄存器级早期锁存 PB1 (电源键) + PB8 (电源锁存) → 防 BOR 重启竞态
2. HAL_Init → SystemClock_Config (HSE 25MHz → PLL 50MHz, CSS 使能) → MX_*_Init
3. POWER_CTRL 置高锁存电源, LED 全部灭
4. PGA112_Init (SPI2, 初始 16x)
5. OLED 启动画面 2s
6. 等待电源键松手 + 50ms 防抖
7. **SelectDeviceId()** — 按 1~9 选择设备编号, 发送确认, 删除清除
8. 初始化 Editor / PWM_DDS / TX / FlashStore / RX
9. 启动 TIM3 ISR (供 transmitter.c 使用)
10. 默认进入 HM_RX 接收模式

## 关键变更历史

| 日期 | 内容 |
|------|------|
| 2026-07-07 | Flash 存储 + LED 修复 + RX 子模式 + auto-save |
| 2026-07-08 | 键盘全部 GPIOA (COL=PA9~PA12) |
| 2026-07-10 | 硬件锁存软开关 (PB1+PB8) + 多终端通信 + 开机选择设备编号 |
| 2026-07-11 | PGA112 AGC + 噪声基线 IIR + 前导读频交替校验 |
| 2026-07-14 | v5 收发协同重构 (变长帧+FEC+同步音) + PGA112 初始 32x + 进门严格化 |
| 2026-07-15 | 状态驱动冻结式 AGC + 帧尾 30ms DC 保护槽 |
| 2026-07-16 | v5.1 True SNR 分类器 + 频域抗噪重构 (当前版本) |
| 2026-07-18 | PGA112 AGC 重构: 候选弱信号获取、统一 Vpp 死区、三采样削顶判定、DMA 回调外 SPI 服务与错误诊断；修正 16x 初始档、SDN_DIS 命令、CRC 完成门控、LED 结束时机、多窗口 SNR 标尺与软判决 LLR 位序/LUT。 |

## 待验证项

- 半双工模式切换 (RX→TX→RX) 外设资源冲突检查
- PA0 一键开关机 Standby 唤醒电流 ≤1mA
- 端到端收发测试 (两板对调)
- 突发模式 AGC: 验证候选增益最多升至64x、前导闭环目标 1120..2234 counts、数据段无增益写入
- 数据段增益冻结后符号擦除率是否显著下降
- CRC 符号丢失率是否下降 (30ms 保护槽效果)
- True SNR 对宽带噪声(风扇/空调)的免疫验证
- 前导双重频域锁: 误唤醒率为零的验证
- PGA112 16x 初始增益与 SPI 错误计数实机验证
- PY25Q64 SPI Flash 读写稳定性验证

## 与旧版文档的差异速查

| 旧文档 (v4) | 代码实际 (v5.1) |
|-------------|----------------|
| HSI 16MHz 时钟 | **HSE 25MHz** 外部晶振 |
| KB_COL = PA4~PA7 | **PA9~PA12** |
| TIM2: PSC=0 ARR=3124 | **PSC=4 ARR=624** |
| 内部 Flash Sector 3, 5条 | 外部 **PY25Q64**, 64条, A/B双副本 |
| v4 固定 192 符号 + XOR | **v5.1** 变长帧 + Hamming(7,4) + CRC-8 + 同步音 |
| "rx" + KEY_RIGHT 切回收信 | KEY_RIGHT 光标在末尾直接切回 |
| HAL_PWR_EnableWakeUpPin 已添加 | **不存在** (代码中无此调用) |
| PGA112 初始增益 8x | **16x** |
| RC 22nF | pwm_dds.c 注释 47nF/100nF |
| 仅 PC13 LED | PC13 + PB2(LEDG) + PB10(LEDR) |
| DEVICE_ID 编译宏 | 开机动态选择 |
