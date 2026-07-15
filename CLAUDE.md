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
| KB_ROW0~3 | PA0~PA3 | Output PP | 矩阵键盘行扫描 |
| KB_COL0~3 | PA4~PA7 | Input PU | 矩阵键盘列读取 |
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
         │ KEY_FN → HM_TX_EDIT
         │ T9键 → HM_TX_EDIT
         │ KEY_SEND → 浏览已存储消息 (LS_BROWSE_LIST)
         │   ├─ LEFT/RIGHT → 选择消息
         │   ├─ 数字键 → 查看消息 (LS_BROWSE_VIEW)
         │   ├─ DELETE → 删除选中
         │   └─ KEY_FN → 退出浏览 (LS_LISTENING)
         │
         HM_TX_EDIT:
         │ KEY_SEND → HM_TX_BUSY → done → HM_TX_EDIT (保留编辑器内容)
         │ KEY_FN → HM_RX
         │ "rx" + KEY_RIGHT → HM_RX (快速切回收信)
         │

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

**UI 操作**:

| 按键 | 物理标签 | RX监听 | RX浏览列表 | RX浏览消息 | TX编辑 | TX发送中 |
|------|---------|--------|-----------|----------|--------|---------|
| 0~9 | 数字 | →TX编辑 | 查看选中 | — | T9输入 | — |
| ← | 左移 | 滚动消息 | 选择上一条 | 向上滚 | 左移光标 | — |
| → | 右移 | 滚动消息 | 选择下一条 | 向下滚 | 右移光标 | — |
| 删除 | 删除 | — | 删除选中 | 退回列表 | 退格 | — |
| 英/数 | KEY_FN | →TX编辑 | 退出浏览 | 退出浏览 | →接收 | — |
| 发送 | KEY_SEND | 浏览已存 | 浏览选中 | 退回列表 | 发送 | — |

**"rx" 快速切回**: 在编辑器中输入 "rx" (不区分大小写), 按 KEY_RIGHT 立即切换到接收模式.

### 2026-07-14 — v5 收发协同重构 (针对 DSP 复核 5 缺陷)

半双工同时含发送与接收, 两端均迁移到 v5 (与单工 01/02 逐字节共享 transmitter.c /
receiver.c / voice_fec.c / voice_dsp.c):
- **发送**: 变长帧 `[前导][1800Hz同步音][LEN三重冗余][payload+CRC8]`, Hamming(7,4)+交织.
- **接收**: 前导+同步音一次锁定 30ms 栅格 (免疫混响), 频谱置信度判决 (无绝对幅值门限,
  提升距离), 差分能量高通检测 (免疫 50Hz/DC), ±1 bin 频偏容忍, FEC 抗突发.
- **公开 API / 顶层半双工状态机 (HM_RX/HM_TX_EDIT/HM_TX_BUSY) / 外设互斥启停 / 双 LED /
  Flash / 中文启动页 / 软开关全部不变**: main.c 无需改动. TX/RX 静态变量无冲突, vd_* 全局
  仅 RX 期使用 (切 TX 前已停 RX), 无资源冲突.
- **CMake**: 新增 `voice_fec.c`/`voice_dsp.c`. 引脚/定时器 (TIM1/2/3) 无变化.
- PC 端 (gcc) 编译 + 链接 + FEC 单测 + 信道仿真 + 收发全链路均通过; 无 arm-none-eabi
  未生成 ELF, **待真机烧录 (两板对调) 声学实测**.

### 2026-07-08 键盘命名重构

- **KEY_MODE → KEY_FN**: 物理"英/数"键, 在半双工中兼作输入模式切换和 RX↔TX 切换.
- **KEY_SEND 增加浏览功能**: 在接收模式按 KEY_SEND 进入已存储消息浏览, 在编辑模式执行发送.
- **key_names[] 更新**: 改为中文标签 "开/关","发送","英/数","删除","←","→" 匹配物理键盘.
- **KEY_ 注释更新**: 所有 define 注释反映半双工中的实际功能.

### 2026-07-07 修复与合并

从单工发送端/接收端合并至半双工:
- **新增 flash_store.c/h**: 内部 Flash 非易失消息存储 (5条满循环缓冲)
- **修复 LED 初始状态**: GPIO_PIN_RESET→GPIO_PIN_SET (上电时LED熄灭)
- **修复 RX Done 显示**: 先显示"Rx Complete" 2s 再清除, 而非先清除后空等
- **新增 RX 子模式**: KEY_MODE 循环切换 LS_LISTENING/浏览列表/查看消息
- **新增 FlashStore_Init**: main() 初始化阶段加载已存储消息
- **新增 auto-save**: receiver.c rx_enter_done() 自动保存到 Flash

### 2026-07-08 修复 (CubeMX 重生成后)

- **简化 keyboard.c**: GPIO 已重新分配为全部在 GPIOA (PA0~PA7, 与单工一致),
  移除 `col_ports[4]` 端口查找表, 恢复原始单 `COL_PORT` 宏。更新注释。
- **修复 keyboard.h 注释**: 反映 PA0~PA3 行、PA4~PA7 列的布局。
- **修复 LED 初始状态**: CubeMX 生成 `GPIO_PIN_RESET` → `GPIO_PIN_SET` (上电 LED 熄灭)。
- **恢复 WKUP 使能**: 添加 `HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1)`,
  PA0 作为键盘矩阵行的同时兼作 Standby 唤醒源。
- **清理 receiver.c 死代码**: `rx_enter_error` 取消注释 (函数存在但未被调用, 与单工保持一致)。

### 引脚分配 (2026-07-08 更新, 与单工一致)

| 功能 | 引脚 | 模式 | 说明 |
|------|------|------|------|
| KB_ROW0~3 | PA0~PA3 | Output PP | 矩阵键盘行扫描 |
| KB_COL0~3 | PA4~PA7 | Input PU | 矩阵键盘列读取 |
| PWM 音频输出 | PA8 | AF1 PP | TIM1_CH1, 48.83kHz → RC 低通 |
| 音频输入 | PB0 | Analog | ADC1_IN8 |
| OLED SCL | PB6 | AF4 OD | I2C1 400kHz |
| OLED SDA | PB7 | AF4 OD | I2C1 400kHz |
| LED | PC13 | Output PP | 板载 LED |

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
- `VD_PREAMBLE`: 激活 AGC (`frame_active=1`), 允许升至 128x
- `VD_DATA`: 跳过 AGC, 绝对冻结增益 (保护 Goertzel 频谱完整性)

`pga112.c` / `voice_dsp.c` 无需修改: 致聋路径已不可达, 能量归一化逻辑已支持变增益.

### 2026-07-15 — 帧尾 30ms DC 保护槽 (方案一)

**问题**: 最后一个 CRC 符号频频收不到。根因: 最后一个符号 guard 结束后零间隙进入
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
接收端完全透明. 最坏 48 字符: 总帧长 11.58s + 0.35s ≈ 11.93s, 仍在 20s 预算内.

### 待验证项
- 半双工模式切换 (RX→TX→RX) 外设资源冲突检查
- PA0 一键开关机 Standby 唤醒电流 ≤1mA
- 端到端收发测试
- 突发模式 AGC: 远距离弱信号接收距离提升验证
- 数据段增益冻结后符号擦除率是否显著下降
- CRC 符号丢失率是否下降 (30ms 保护槽效果)
