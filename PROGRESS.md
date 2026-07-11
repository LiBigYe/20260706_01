# 声语信使 — 半双工项目进度文档

> 最后更新: 2026-07-07
> 阶段: v4 DPLL下降沿同步 + 信息存储 (提高部分③)

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
  ├─ LS_BROWSE_LIST: 已存储消息列表 (5条循环缓冲)
  └─ LS_BROWSE_VIEW: 查看单条存储消息
HM_TX_EDIT (编辑短信息)
HM_TX_BUSY (发送中)
```

### 模式切换

| 从 | 到 | 触发 |
|---|----|------|
| HM_RX | HM_TX_EDIT | KEY_SEND 或 T9数字键 |
| HM_TX_EDIT | HM_TX_BUSY | KEY_SEND (消息非空) |
| HM_TX_BUSY | HM_RX | TX done 自动返回 |
| HM_RX 子模式切换 | LS_LISTENING↔BROWSE_LIST↔BROWSE_VIEW | KEY_MODE |
| 任意 | Standby | KEY_POWER |

---

## 三、信息存储 (提高部分③)

### 硬件

STM32F411CEU6 内部 Flash Sector 3 (0x0800C000, 16KB)

### 存储格式

67 words = 268 bytes, 全量擦写策略

### 循环缓冲

5条FIFO, 满时自动淘汰最旧消息

### 自动保存

接收完成后(rx_enter_done)自动写入Flash, 断电不丢失

---

## 四、v4 DPLL 下降沿 (与单工相同)

收发共享协议帧: Preamble(200ms) + Data(192 symbols) + Checksum(4) + Postamble(200ms)

---

## 五、待完成

- [ ] 硬件联调: 半双工模式切换 (RX↔TX) 无外设资源冲突
- [ ] 端到端收发测试 (两板对调)
- [ ] Flash 写入时 CPU stall 对正在进行的 DMA 的影响 (仅在 RX_STATE_DONE 时写入, 无 DMA)
- [ ] 新增 flash_store.c 需加入 CMakeLists.txt 编译

---

## 六、变更记录

### 2026-07-08 (CubeMX 重生成后修复)

**GPIO 重分配**: 键盘矩阵改为全部 GPIOA (PA0~PA7, 与单工一致):
- keyboard.c 移除 `col_ports[4]`，恢复单 `COL_PORT` 宏
- keyboard.h 注释更新为 PA0~PA3 rows, PA4~PA7 cols
- 修复 LED 初始态 `GPIO_PIN_RESET` → `GPIO_PIN_SET`
- 添加 `HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1)` (PA0 兼作 WKUP)
- 清理 receiver.c `rx_enter_error` 死代码注释

### 2026-07-07

**从单工合并修复**:
- 新增 flash_store.c/h (内部Flash非易失存储, 5条循环缓冲)
- 修复 LED 初始状态 GPIO_PIN_RESET→GPIO_PIN_SET (上电熄灭)
- 修复 RX Done 处理顺序 (先显示"Rx Complete"2s → 再清除重启)
- 新增 RX 子模式 (LS_LISTENING/BROWSE_LIST/BROWSE_VIEW, KEY_MODE切换)
- 新增 FlashStore_Init 初始化调用
- 新增 receiver.c 自动保存 (rx_enter_done → FlashStore_SaveMessage)

### 2026-07-10 — 增加硬件锁存软开关

- 复用发送端软开关方案：`PB1=POWER_BUTTON/EXTI1`，`PB8=POWER_CTRL`。
- 增加 HAL 初始化前的寄存器级早期锁存和 BOR 防重启等待逻辑。
- 增加开机按键松手检测与 50ms 防抖。
- EXTI 回调立即解除电源锁存；主循环停止 ADC DMA、TIM2、TIM3、RX 和 PWM DDS，关闭指示灯及 OLED 后等待硬件掉电。
- `cmake --preset Debug` 与 `cmake --build --preset Debug --parallel` 构建通过；FLASH 49120B，RAM 7096B。
- 待实机验证：RX/TX/浏览三种状态下均可可靠关机，且关机电流不大于 1mA。

### 2026-07-10 — 多终端通信

- 本终端编号设为 `DEVICE_ID=3`，复制终端工程时只需修改该宏。
- 支持ID 1~9、任意多选目标及广播；发送编辑与收件人选择使用独立状态，避免数字键输入冲突。
- 收件人页：1~9切换目标，0广播，发送确认，删除/英数取消；不会因取消最后一个目标而意外广播。
- 接收端仅处理广播或包含本机ID的消息，并在实时与历史显示中标注发送端ID。
- Flash v2保存来源ID并兼容v1旧消息；完整帧固定约6.64秒。
- Debug构建通过。

### 2026-07-10 — 开机设置设备编号

- 半双工终端移除固定编号，每次开机按1~9选择，发送键确认，删除键清除。
- 未确认前不进入RX/TX状态；确认后才初始化收发状态机并进入监听。
- 发送源ID、接收地址过滤和收件人选择页统一使用运行时编号。
