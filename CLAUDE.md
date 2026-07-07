请先研读项目根目录下"电子系统课程设计-声语信使-设计任务书(补充传输时间要求).pdf"文件然后准备开始现在的项目设计，我会提供给你我现在的需求，你只需要完成我的需求即可，你需要自行验证逻辑的完备性与功能的正确性，如在项目实际测试中遇到实际问题我会告知你进行修改，在项目全程如有需要cubemx进行引脚定义与分配请告知我；
我是项目小组中的软件手，主要负责本项目中使用的stm32f411ceu6的mcu代码编写。
在每次回答之后更新项目进度

## 项目03 — 半双工 (Half-Duplex)

本目录为声语信使系统的半双工终端代码。同一块板既可发信也可收信，但不能同时进行。

### 硬件配置
- MCU: STM32F411CEU6 (HSI 16MHz → PLL 50MHz SYSCLK)
- PA0-WKUP: 独立电源键 (上升沿从 Standby 唤醒)
- TIM1_CH1 (PA8): PWM DDS 音频输出 → RC 低通滤波 (1kΩ + 22nF)
- TIM2: 16kHz ADC 触发 (PSC=0, ARR=3124, 50MHz→16kHz)
- TIM3: 16kHz DDS 采样时钟 (PSC=4, ARR=624, 50MHz→16kHz)
- ADC1_IN8 (PB0): 音频输入 (经模拟前端放大→偏置到1.65V)
- DMA2_Stream0: 循环DMA传输 ADC→缓冲 (800 halfwords)
- I2C1 (PB6/PB7): OLED 128x64
- 4x4键盘 (PA1~PA4 rows, PA5~PA7+PB3 cols)
- PC13: LED 指示灯

### 引脚分配

| 功能 | 引脚 | 模式 | 说明 |
|------|------|------|------|
| POWER/WKUP | PA0 | Input PD | 独立按键→VDD, 按下=高唤醒 |
| KB_ROW0~3 | PA1~PA4 | Output PP | 矩阵键盘行扫描 |
| KB_COL0~2 | PA5~PA7 | Input PU | 矩阵键盘列读取 |
| KB_COL3 | PB3 | Input PU | 矩阵键盘列读取 |
| PWM 音频输出 | PA8 | AF1 PP | TIM1_CH1, 48.83kHz → RC 低通 |
| 音频输入 | PB0 | Analog | ADC1_IN8 |
| OLED SCL | PB6 | AF4 OD | I2C1 400kHz |
| OLED SDA | PB7 | AF4 OD | I2C1 400kHz |
| LED | PC13 | Output PP | 板载 LED |

### 源文件结构
- `Core/Src/main.c` — 主程序, 半双工状态机 (HM_RX / HM_TX_EDIT / HM_TX_BUSY), UI子模式
- `Core/Src/transmitter.c` — 4-FSK发送状态机
- `Core/Src/fsk4_encoder.c` — 4-FSK编码器 (75个字符)
- `Core/Src/fsk16_encoder.c` — 16-FSK编码器 (预留, 当前未使用)
- `Core/Src/pwm_dds.c` — PWM DDS 正弦波生成 (1024-pt × 10-bit LUT)
- `Core/Src/receiver.c` — 接收状态机 + DPLL 下降沿同步 (v4)
- `Core/Src/fsk4_decoder.c` — Goertzel 4-FSK解码器 (N=320, 整数k)
- `Core/Src/flash_store.c` — 内部Flash非易失消息存储 (Sector 3, 16KB)
- `Core/Src/oled.c` — SSD1306 OLED驱动 (I2C, 5x7字体)
- `Core/Src/keyboard.c` — 4x4矩阵键盘扫描
- `Core/Src/editor.c` — T9 文本编辑器

### 半双工模式切换

```
上电 → HM_RX (默认接收)
         │ KEY_SEND / T9键 → HM_TX_EDIT (编辑)
         │              │ KEY_SEND → HM_TX_BUSY (发送)
         │              │              │ TX done → HM_RX
         │              │ KEY_MODE/数字 → 留在编辑模式
         │ KEY_MODE → 循环 RX 子模式:
         │   LS_LISTENING → LS_BROWSE_LIST → 回到 LS_LISTENING
         │ KEY_LEFT/RIGHT → 浏览列表导航 / 滚动查看
         │
         POWER键 → Standby (PA0 WKUP唤醒)

切换时: RX↔TX 互斥启动/停止 ADC+DMA+TIM2 / TIM1+TIM3
```

### TIM 参数

| 定时器 | 用途 | PSC | ARR | 频率 |
|--------|------|-----|-----|------|
| TIM1 | PWM 载波 (CH1, PA8) | 0 | 1023 | 48.83kHz |
| TIM2 | ADC 触发 (TRGO) | 0 | 3124 | 16kHz |
| TIM3 | DDS 时钟 + ISR | 4 | 624 | 16kHz |

### v4 协议帧结构 (与单工版本相同)
```
[Preamble 200ms] [Data 192 symbols] [Checksum 4 symbols] [Postamble 200ms]
```
- 字符集: a-z, A-Z, 0-9, .?!, $, (), +-*\/=, 空格, \n (共75个)
- 每字符: 4 个 base-4 符号
- 校验和: 8-bit XOR → 4个base-4符号
- TX 真实字符后插入 '$' 终止符, 再空格填充到 48 字符

### OLED 布局
- 第0~6行: 文本区 (支持 \n 换行, 屏幕宽度自动换行, 末尾7行自动滚动)
- 第7行: 状态栏 — 左侧 Count:XX或XX/XX[模式], 右侧状态文字

### 信息存储 (提高部分③, 2026-07-07)

使用 STM32F411CEU6 内部 Flash **Sector 3** (0x0800C000, 16KB) 存储最多 5 条消息.
Code 占用 ~30.6KB, Sector 3 完全空闲.

**Flash 布局** (67 words = 268 bytes):
```
Word 0: Magic 0x564F4943 ("VOIC")
Word 1: Version(16) | msg_count(16)
Words 2-14:  消息槽位 0 (13 words, valid/length/data[50])
Words 15-27: 消息槽位 1
Words 28-40: 消息槽位 2
Words 41-53: 消息槽位 3
Words 54-66: 消息槽位 4
```

**写入策略**: 每次保存/删除 → 擦除 sector → 全量重写 5 槽.
在 rx_enter_done 中自动保存 (接收完成且校验通过后).
循环缓冲: 满 5 条时淘汰最旧 (slot 0).

**UI 操作 (RX 子模式)**:

| 按键 | LS_LISTENING | LS_BROWSE_LIST | LS_BROWSE_VIEW |
|------|-------------|---------------|---------------|
| MODE | 进入浏览列表 | 返回监听 | 返回列表 |
| LEFT/RIGHT | 滚动当前消息 | 选择消息 | 滚动内容 |
| 数字键 0~9 | → 发送编辑 | 查看选中消息 | — |
| DELETE | — | 删除选中(边沿) | 返回列表 |
| SEND | → 发送编辑 | → 发送编辑 | → 发送编辑 |

### 2026-07-07 修复与合并

从单工发送端/接收端合并至半双工:
- **新增 flash_store.c/h**: 内部 Flash 非易失消息存储 (5条满循环缓冲)
- **修复 LED 初始状态**: GPIO_PIN_RESET→GPIO_PIN_SET (上电时LED熄灭)
- **修复 RX Done 显示**: 先显示"Rx Complete" 2s 再清除, 而非先清除后空等
- **新增 RX 子模式**: KEY_MODE 循环切换 LS_LISTENING/浏览列表/查看消息
- **新增 FlashStore_Init**: main() 初始化阶段加载已存储消息
- **新增 auto-save**: receiver.c rx_enter_done() 自动保存到 Flash

### 待验证项
- 半双工模式切换 (RX→TX→RX) 外设资源冲突检查
- PA0 一键开关机 Standby 唤醒电流 ≤1mA
- 端到端收发测试
