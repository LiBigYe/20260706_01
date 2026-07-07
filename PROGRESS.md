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

### 2026-07-07

**从单工合并修复**:
- 新增 flash_store.c/h (内部Flash非易失存储, 5条循环缓冲)
- 修复 LED 初始状态 GPIO_PIN_RESET→GPIO_PIN_SET (上电熄灭)
- 修复 RX Done 处理顺序 (先显示"Rx Complete"2s → 再清除重启)
- 新增 RX 子模式 (LS_LISTENING/BROWSE_LIST/BROWSE_VIEW, KEY_MODE切换)
- 新增 FlashStore_Init 初始化调用
- 新增 receiver.c 自动保存 (rx_enter_done → FlashStore_SaveMessage)
