# Core/Src 函数说明

本文覆盖项目自行维护的 `Core/Src/*.c`。`Drivers/` 中的 HAL/CMSIS 是 ST 的第三方实现，
没有逐函数展开；`build/` 中的 `CMakeCCompilerId.c` 是 CMake 生成文件，也不属于固件业务代码。

## 总体流程

设备启动后由 `main.c` 初始化外设，选择本机 ID，然后进入半双工接收模式。发送路径为：
编辑器 -> `transmitter.c` -> `voice_fec.c` -> PWM DDS。接收路径为：ADC DMA ->
`receiver.c` -> `voice_dsp.c` -> `voice_fec.c` -> 显示与外置 Flash 存储。

## editor.c

| 函数 | 具体内容及作用 |
|---|---|
| `CycleLen`（静态） | 取得当前输入模式下某个 T9 键的候选字符数；供多击循环取模。 |
| `GetChar`（静态） | 按 T9 键号和第几次连按，从映射表取字符；无效输入返回 `?`。 |
| `BufInsert`（静态） | 从光标处右移缓冲区后插入字符，更新长度、光标和重绘标记；实现中间插入。 |
| `BufBackspace`（静态） | 删除光标前字符，将后续字符左移并补终止符；实现退格。 |
| `BufReplaceLast`（静态） | 替换光标前一个字符；同一 T9 键在超时前连续按下时用于切换候选字。 |
| `HandleT9`（静态） | 判断是否同键且未超时：是则循环替换前一字符，否则插入新字符；维护连按时间和次数。 |
| `NextMode`（静态） | 在 `123`、`abc`、`ABC` 三种输入模式间循环，并取消未提交的 T9 连按状态。 |
| `Editor_Init` | 清空编辑器状态，设置 T9 空键、初始可见光标和脏标志。 |
| `Editor_HandleKey` | 把键盘码分发为 T9 输入、删除、左右移动、模式切换或发送请求；发送时只置标志，由主循环执行。 |
| `Editor_Tick` | 每轮检查光标闪烁周期和 T9 超时，必要时使画面失效等待刷新。 |
| `Editor_UpdateDisplay` | 按换行和屏宽将缓冲区分视觉行，保证光标可见地滚动，在 OLED 上绘制文本、闪烁光标和状态栏。 |
| `Editor_GetBuffer` | 返回以 `\0` 结尾的待发文本缓冲区。 |
| `Editor_GetLength` | 返回当前文本字符数。 |
| `Editor_IsCursorAtEnd` | 判断光标是否位于末尾，用于主界面切换等逻辑。 |
| `Editor_IsSendRequested` | 返回发送键是否已触发。 |
| `Editor_ClearSendRequest` | 清除发送请求，避免主循环重复启动一次发送。 |
| `Editor_SetTxStatus` | 安全复制发送状态文本到编辑器状态栏，并请求重绘。 |

## flash_store.c

该文件将 64 条消息组成一个带魔数、版本、代数和 CRC32 的镜像，并在 SPI Flash 的 A/B 扇区间轮换提交。

| 函数 | 具体内容及作用 |
|---|---|
| `crc32_update`（静态） | 用多项式 `0xEDB88320` 对一段字节更新 CRC32。 |
| `image_crc`（静态） | 对镜像版本、数量、代数和所有消息槽计算 CRC，魔数与 CRC 字段自身不参与计算。 |
| `image_valid`（静态） | 同时验证魔数、版本、数量上限和 CRC；防止使用损坏的副本。 |
| `generation_newer`（静态） | 用有符号差比较两个递增代数，可正确处理无符号计数回绕。 |
| `commit`（静态） | 把内存镜像写到非活动副本：更新元数据与 CRC、擦扇区、写入、读回校验，成功后切换活动地址。 |
| `FlashStore_Init` | 初始化 PY25Q64，读取两个副本，选择最新有效镜像；二者都无效时创建空镜像。 |
| `FlashStore_IsReady` | 返回外置 Flash 是否已成功初始化。 |
| `FlashStore_SaveMessage` | 以来源 ID `0` 保存消息，是带来源保存函数的简化入口。 |
| `FlashStore_SaveMessageFrom` | 截断到槽容量后追加消息；满 64 条时丢弃最旧项；提交失败会恢复内存备份。 |
| `FlashStore_DeleteMessage` | 删除指定消息并前移后续槽，提交失败时恢复原镜像。 |
| `FlashStore_GetCount` | 返回当前已存消息数。 |
| `FlashStore_GetTotal` | 返回固定容量 `FLASH_STORE_MAX_MSGS`。 |
| `FlashStore_GetMessage` | 对合法且有效的下标返回槽指针，否则返回 `NULL`。 |
| `FlashStore_EraseAll` | 清空所有槽和数量并提交；提交失败则恢复备份。 |

## fsk16_encoder.c

这是预留的 16-FSK 旧编码器，当前实际发送链路未调用它。

| 函数 | 具体内容及作用 |
|---|---|
| `FSK16_CharToIndex` | 将字母、数字和允许标点映射到字符表索引，非法字符返回 `255`。 |
| `FSK16_IndexToChar` | 将字符表索引恢复为字符，非法索引返回 `?`。 |
| `FSK16_GetFrequency` | 从 16 个频点表取得对应 Hz 值，越界返回 0。 |
| `FSK16_GetPhaseInc` | 从预计算 DDS 表取得 32 位相位增量，越界返回 0。 |
| `FSK16_Init` | 清零编码器状态，依据频率计算相位增量，并保存正弦表指针。 |
| `FSK16_Encode` | 逐字符转为两个十六进制符号，累加 XOR 校验并在尾部追加两个校验符号。 |
| `FSK16_EstimateTime` | 按前导、每符号时长和结束段估算传输毫秒数。 |

## fsk4_encoder.c 与 fsk4_decoder.c

这两个文件实现旧的裸 4-FSK 编码/Goertzel 检测接口。当前 v5 链路改用 `voice_fec.c` 和 `voice_dsp.c`，但字符表和 DDS 频率表仍被 PWM 模块使用。

| 函数 | 具体内容及作用 |
|---|---|
| `FSK4_CharToIndex` | 把支持的 76 个字符编码为 0--75；非法字符返回 `255`。 |
| `FSK4_IndexToChar` | 将上述索引还原为 ASCII 字符，非法索引返回 `?`。 |
| `FSK4_GetFrequency` | 返回 digit 0--3 对应的 1500、1800、2100、2400 Hz。 |
| `FSK4_GetPhaseInc` | 返回 digit 对应的 16 kHz DDS 相位增量。 |
| `FSK4_Init` | 清零编码器，并以公式重算四个 DDS 相位增量。 |
| `FSK4_Encode` | 写入来源 ID 和目标掩码头部；文本转四进制符号，追加 `$` 终止符并补空格到 48 字符，最后写 XOR 校验。 |
| `FSK4_EstimateTime` | 按前导、30 ms 符号槽和后导估算时长。 |
| `FSK4_Decoder_GetFreqName` | 返回 digit 对应的频率名称，便于调试显示。 |
| `FSK4_Decoder_Init` | 保存窗口大小，计算四个 Goertzel 频点的整数 bin 和递推系数。 |
| `FSK4_Decoder_Reset` | 清零 Goertzel 的 `q1/q2`、采样计数及上次结果，准备下一个窗口。 |
| `FSK4_Decoder_ProcessSample` | 对一个去直流 ADC 样本并行执行四路 Goertzel 递推；窗口结束时计算各频点幅度平方。 |
| `FSK4_Decoder_DetectBlock` | 先检查峰峰值，再处理完整块，比较最强与次强频点及噪声阈值，输出 digit 或 `0xFF`。 |
| `FSK4_Decoder_GetMagnitudes` | 拷贝最近窗口的四个幅度平方值给调试调用者。 |

## keyboard.c

| 函数 | 具体内容及作用 |
|---|---|
| `Keyboard_Init` | 将 4 个行输出恢复为高电平，使矩阵键盘处于空闲状态。 |
| `ScanMatrix`（静态） | 逐行拉低、读取四列；探测到低电平立即恢复行线并返回映射后的键码。 |
| `Keyboard_Scan` | 对原始扫描结果执行 30 ms 去抖，并实现长按 200 ms 后、每 150 ms 一次的重复键事件。 |
| `Keyboard_IsPressed` | 扫描四列是否存在低电平，用于确认模式切换后按键是否已释放。 |
| `Keyboard_GetKeyName` | 将键码转换为 UI/调试用字符串；无效键返回 `?`。 |

## main.c

此文件是半双工 UI 与硬件编排中心。`HM_*` 函数在接收 ADC/DMA 和发送 PWM/定时器两组资源之间切换。

| 函数 | 具体内容及作用 |
|---|---|
| `Power_CutOff`（静态内联） | 直接写 PB8 的 BSRR 复位位，切断自保持电源。 |
| `SelectDeviceId`（静态） | 在 OLED 上循环显示 ID 选择界面，以数字键选择 1--9、发送键确认；侦测关机标志后停机。 |
| `HM_ShowRxFooter`（静态） | 绘制接收来源、字符数和右侧状态文本的底栏。 |
| `ShowTargetSelection`（静态） | 显示发送者 ID、当前广播/目标列表和选择说明。 |
| `HM_StopRxSampling`（静态） | 停 ADC DMA 与 TIM2，停止接收状态机并熄灭接收活动灯。 |
| `HM_StartRxSampling`（静态） | 启动 ADC 循环 DMA、TIM2 采样定时器和接收状态机。 |
| `HM_SwitchToRx`（静态） | 清理发送状态、关闭 PWM，恢复 ADC 接收；不清空编辑文本。 |
| `HM_SwitchToTxEdit`（静态） | 停止接收，进入文本编辑状态并设置 `Tx Ready`。 |
| `HM_SwitchToTx`（静态） | 停止接收，启动 PWM 和发送状态机，进入发送忙状态。 |
| `HM_SaveReceivedMessage`（静态） | 暂停采样后将刚收到的消息及来源 ID 写入外置 Flash。 |
| `HM_DeleteStoredMessage`（静态） | 仅在没有活动帧时停止采样、删除一条历史消息，再重新接收。 |
| `HAL_ADC_ConvHalfCpltCallback` | ADC DMA 前半缓冲完成回调；RX 模式下交给接收器处理前 400 个采样。 |
| `HAL_ADC_ConvCpltCallback` | ADC DMA 后半缓冲完成回调；RX 模式下处理后 400 个采样。 |
| `main` | 处理上电自保持，初始化 HAL/时钟/外设/OLED/键盘/收发与存储；随后运行 RX 浏览、编辑、目标选择、发送和省屏状态机。 |
| `SystemClock_Config` | 配置 HSI、PLL 和 AHB/APB 分频，建立所有外设的运行时钟。 |
| `MX_ADC1_Init` | 配置 ADC1 的单通道、外部 TIM2 触发及 DMA 接收参数。 |
| `MX_I2C1_Init` | 配置 I2C1，供 SSD1306 OLED 使用。 |
| `MX_SPI1_Init` | 配置 SPI1，供 PY25Q64 外置 Flash 使用。 |
| `MX_TIM1_Init` | 配置 TIM1 CH1 的 10 位 PWM，作为 DDS 模拟输出载体。 |
| `MX_TIM2_Init` | 配置 TIM2 以 16 kHz 触发 ADC 采样。 |
| `MX_TIM3_Init` | 配置 TIM3 的 16 kHz 更新中断，驱动发送 DDS。 |
| `MX_DMA_Init` | 使能 ADC DMA2 Stream0 时钟、中断及优先级。 |
| `MX_GPIO_Init` | 配置按键矩阵、LED、电源、Flash 片选和各 GPIO 的模式、上下拉及 EXTI。 |
| `HAL_GPIO_EXTI_Callback` | 收到电源按键 EXTI 时置 `g_power_off`；主循环随后安全关闭外设和显示。 |
| `Error_Handler` | 关闭全局中断并无限循环，作为 HAL 初始化失败的停止点。 |
| `assert_failed` | 在启用 `USE_FULL_ASSERT` 时接收断言文件与行号；当前不执行恢复或输出。 |

## oled.c

OLED 使用 8 页 x 128 列软件帧缓冲；除 `OLED_Refresh` 外的绘图函数均只修改内存。

| 函数 | 具体内容及作用 |
|---|---|
| `OLED_WriteCmd`（静态） | 在 I2C 控制字节 `0x00` 后发送一条 SSD1306 命令。 |
| `OLED_WriteDataBulk`（静态） | 在控制字节 `0x40` 后批量发送最多一页像素数据。 |
| `OLED_Init` | 保存 I2C 句柄，发送 SSD1306 初始化序列，清屏刷新，延时后打开显示。 |
| `OLED_Clear` | 将全部帧缓冲字节置零。 |
| `OLED_Fill` | 将帧缓冲全置零或全置一。 |
| `OLED_DrawPixel` | 带边界检查地设置/清除一个像素对应的页内 bit。 |
| `OLED_ShowChar` | 用 6x8 ASCII 字模在帧缓冲画正常字符。 |
| `OLED_ShowCharInvert` | 先填白 6x8 区域，再清除字模像素，以反白显示字符。 |
| `OLED_ShowString` | 连续绘制字符串，达到右边界时自动换行，到屏底停止。 |
| `OLED_ShowStringInvert` | 与上一函数相同，但使用反白字形。 |
| `OLED_DrawStartupChinese`（静态） | 将 12x12 中文启动字形的置位 bit 逐像素画入帧缓冲。 |
| `OLED_DrawStartupAscii`（静态） | 查找 12 像素高的启动页 ASCII 字形并绘制。 |
| `OLED_DrawStartupTitleChinese`（静态） | 绘制首行高对比度 16x16 中文标题字。 |
| `OLED_DrawStartupTitleAscii`（静态） | 绘制首行 16 像素高的 ASCII 标题字。 |
| `OLED_ShowStartupScreen` | 组合绘制“声语信使”标题、班级和三位成员姓名/学号，并刷新屏幕。 |
| `OLED_SetDisplay` | 发送显示开/关命令；由关转开时重新刷新完整帧缓冲。 |
| `OLED_IsDisplayEnabled` | 返回面板当前的逻辑开关状态。 |
| `OLED_Refresh` | 在面板开启时设置全屏地址范围，并逐页把 1024 字节帧缓冲写入 SSD1306。 |

## pwm_dds.c

| 函数 | 具体内容及作用 |
|---|---|
| `PWM_DDS_Init` | 保存 TIM1 句柄并启动 CH1 PWM；相位从零开始，默认频率为 digit 0。 |
| `PWM_DDS_SetFreq` | 将有效 FSK digit 对应的相位增量设为下一次 DDS 输出频率。 |
| `PWM_DDS_OutputMidscale` | 清相位和相位增量，立即将 PWM 比较值写为 512，输出 1.65 V 保护间隔/静态电平。 |
| `PWM_DDS_Shutdown` | PWM 已开启时停止 TIM1 CH1，关闭模拟音频输出。 |
| `PWM_DDS_Start` | PWM 已关闭时重新启动 TIM1 CH1 和高级定时器主输出。 |
| `PWM_DDS_Tick` | 每 62.5 us 累加 32 位相位，取高 10 位索引正弦表并写 TIM1 CCR1；生成连续正弦。 |

## py25q64.c

| 函数 | 具体内容及作用 |
|---|---|
| `cs_low` / `cs_high`（静态） | 拉低/拉高外置 Flash 的片选 GPIO。 |
| `send_command`（静态） | 在一次片选周期内发送一个无地址 SPI 命令。 |
| `wait_ready`（静态） | 轮询状态寄存器 BUSY 位直到空闲或超时。 |
| `write_enable`（静态） | 确认芯片空闲后发送写使能命令。 |
| `PY25Q64_Init` | 保存 SPI 句柄，退出掉电模式并读取 JEDEC ID 验证芯片型号。 |
| `PY25Q64_ReadJedecId` | 发送 `0x9F`，读取并拼接 3 字节厂商/器件 ID。 |
| `PY25Q64_Read` | 校验地址范围后发送读命令和 24 位地址，再接收指定长度。 |
| `PY25Q64_Write` | 按页边界拆分数据；每页写前使能写入，写后等待忙位清除。 |
| `PY25Q64_EraseSector` | 地址向下对齐到扇区，写使能后发送 4 KiB 扇区擦除并等待完成。 |
| `PY25Q64_EraseChip` | 写使能后发整片擦除命令，并以较长超时等待完成。 |

## receiver.c

| 函数 | 具体内容及作用 |
|---|---|
| `RX_CharToIndex` / `RX_IndexToChar` | 保留的字符表双向映射，供旧接口或兼容调用使用。 |
| `rx_sync_state`（静态） | 将 `VoiceRx` 的 DSP 内部状态映射成旧的 `RX_STATE_*` 公开状态。 |
| `rx_restart_listening`（静态） | 重新启动 DSP 接收、同步状态并复位收信 LED。 |
| `rx_on_frame_done`（静态） | 检查 FEC/CRC、解析 `[来源 ID, 目标掩码, 文本]`，只接受发给本机/广播的帧并填充显示缓冲。 |
| `RX_Init` | 初始化 VoiceRx、公开状态、消息/显示缓冲和诊断数据。 |
| `RX_Start` | 每次开始监听时重置帧数据和 done 标志，保留 DSP 噪声底学习结果。 |
| `RX_Stop` | 设置空闲状态、清 done 标志并熄灭接收灯。 |
| `RX_ProcessHalfBuffer` | 将一半 DMA 缓冲再切为 80 采样块送入 DSP；根据 DATA 进出控制 LED，帧完成时调用完成处理。 |
| `RX_IsBusy` | 在监听、前导或数据状态返回真。 |
| `RX_IsFrameActive` | 仅在前导检测和数据接收阶段返回真，供禁止存储操作使用。 |
| `RX_GetNoiseFloor` / `RX_GetEnergyThreshold` / `RX_GetLastEnergy` | 返回 DSP 的噪声底、活动阈值和最近块能量诊断值。 |
| `RX_IsDone` / `RX_GetState` / `RX_GetSymbolCount` | 分别返回帧完成标志、公开状态和已接收符号数量。 |
| `RX_ClearDone` | 清除完成标志并置空闲；主界面显示完消息后调用。 |
| `RX_GetStateName` | 将状态码转为可显示字符串。 |
| `RX_GetMessage` / `RX_GetMessageLength` | 返回本次成功接收文本及长度。 |
| `RX_GetSourceId` / `RX_GetTargetMask` | 返回本帧头部解析出的发送方和目标位图。 |
| `RX_GetDisplaySourceId` | 返回当前显示缓冲所属的发送方 ID。 |
| `RX_GetLastSymbol` | 输出最近判定 digit；可选地复制四个调试幅度。 |
| `RX_GetPilotHits` / `RX_GetEraseRun` / `RX_GetLastSNR` / `RX_GetDspSubState` | 暴露导频命中、连续擦除、最近置信度和 DSP 子状态，服务 LiveWatch 诊断。 |
| `RX_GetDisplayMessage` / `RX_GetDisplayLength` / `RX_GetScrollLine` | 返回 OLED 当前要显示的文本、长度和行滚动位置。 |
| `RX_ScrollUp` | 滚动位置大于零时向上一行。 |
| `RX_ScrollDown` | 仅在内容超过可视行且未到底部时向下一行。 |
| `RX_ScrollWrapUp` | 有溢出内容时向上滚动，到顶后回绕到底部。 |
| `RX_GetStatusString` | 将接收状态转换为底栏的 `Stand By`、`Incoming Data` 或 `Rx Complete`。 |

## transmitter.c

| 函数 | 具体内容及作用 |
|---|---|
| `tx_set_symbol`（静态） | 将一个 4-FSK digit 交给 PWM DDS 设定频率。 |
| `tx_guard`（静态） | 调用 DDS 中值输出，生成 10 ms 的直流保护间隔。 |
| `TX_Init` | 将发送状态机、计数器和完成标志复位为 IDLE。 |
| `TX_Start` | 校验来源/掩码，组装头部加文本 payload，经 FEC 变成符号流，启动 2 s 交替导频和 TIM3 中断。 |
| `TX_Tick` | 每个 16 kHz 中断推进前导、同步、数据、隔离尾和后导状态；在每符号 tone/guard 边界切换 DDS，结束时置完成标志。 |
| `TX_IsBusy` | 在 IDLE/DONE 以外状态返回真。 |
| `TX_IsDone` | 返回后导结束标志。 |
| `TX_ClearDone` | 复位到 IDLE，关闭 LED、输出中值，并停止 TIM3 中断。 |
| `TX_GetStateName` | 返回发送状态的人类可读名称。 |
| `HAL_TIM_PeriodElapsedCallback` | TIM3 更新中断回调，调用 `TX_Tick`；这是发送波形的实时驱动入口。 |

## voice_dsp.c

该文件是当前接收物理层：先做带通滤波和能量/导频检测，再一次性锁定同步音，按固定 30 ms 栅格判决变长数据符号。

| 函数 | 具体内容及作用 |
|---|---|
| `biquad_push`（静态） | 对一个二阶 IIR 节执行 Direct Form II 递推，并更新延迟状态。 |
| `bandpass_reset`（静态） | 清零高通和三级低通滤波器的状态，取消已建立的滤波记忆。 |
| `bandpass_sample`（静态） | ADC 样本去中心值后经高通和三级低通，首样本预置状态避免 DC 阶跃假瞬态。 |
| `goertzel_mag2`（静态） | 在一个窗口计算指定 DFT bin 的 Goertzel 幅度平方。 |
| `goertzel_band_mag2`（静态） | 累加中心 bin 及相邻两个 bin，容忍 TX/RX 时钟产生的频偏。 |
| `VoiceDSP_Classify` | 单窗口去 DC、计算四个频带能量与 SNR/频率支配比，输出最佳 digit 或擦除 `0xFF`。 |
| `VoiceDSP_ClassifyMulti` | 对三个重叠子窗口累加频带能量并判决；数据段用它提高混响和相位偏差下的稳定性。 |
| `VoiceDSP_DiffEnergy` | 累加相邻样本绝对差，得到对 DC 与低频干扰不敏感的活动能量。 |
| `ring_push`（静态） | 将滤波后样本写入按 2 的幂回绕的历史环形缓冲。 |
| `ring_extract`（静态） | 从绝对样本位置拷贝一段环形历史到连续工作窗口。 |
| `sync_window_mag2`（静态） | 对 5 ms 历史片段计算 1800 Hz 同步音能量。 |
| `sync_find_onset`（静态） | 回扫同步音附近，找到第一个达到峰值 80% 的窗口，从而确定符号栅格起点。 |
| `VoiceRx_Init` | 首次初始化 VoiceRx、滤波器及默认噪声底和导频统计。 |
| `VoiceRx_Start` | 为下一次监听清帧状态和环形缓存，但保留学到的噪声底。 |
| `data_store_symbol`（静态） | 对收满的 20 ms tone 做多窗口判决，保存软信息；收到长度前缀后算期望长度，收满后调用软 FEC。 |
| `VoiceRx_PushBlock` | 处理 80 个 ADC 样本并推动 LISTEN/PREAMBLE/DATA/DONE 状态机：校准噪声底、确认交替导频、锁同步、按栅格收符号。 |

## voice_fec.c

当前帧的可靠性编码：长度三重冗余；每半字节 Hamming(7,4)；按列交织；正文含 CRC-8。软判决路径会利用四个频点能量做 Chase 候选搜索。

| 函数 | 具体内容及作用 |
|---|---|
| `VoiceFEC_Crc8` | 用协议 CRC-8 多项式计算任意字节序列的校验值。 |
| `VoiceFEC_HammingEncode` | 将低 4 位半字节编码为带三个校验位的 7 bit Hamming 码。 |
| `VoiceFEC_HammingDecode` | 计算 syndrome、修正至多一位错误，并返回解出的半字节及是否修正。 |
| `VoiceFEC_ComputeLLR` | 由 bit=1/0 的能量比求对数似然比，供软判决可靠度使用。 |
| `byte_to_codewords`（静态） | 将一个字节的高、低半字节分别编码成两个 Hamming 码字。 |
| `interleave_codewords`（静态） | 将码字按列输出到位流，分散相邻的突发错误。 |
| `deinterleave_codewords`（静态） | 将传输位流按列反交织回各 Hamming 码字。 |
| `symbols_to_bits`（静态） | 将每个 4-FSK symbol 的两位展开为 bit 流。 |
| `bits_to_symbols`（静态） | 将成对 bit 重新组合为 0--3 的 4-FSK symbol。 |
| `symbol_bit_llr`（静态） | 由某个 symbol 的四个频带能量计算其中一个 bit 的 LLR。 |
| `chase_hamming_decode`（静态） | 以硬判决为起点，只翻转最不可靠位的候选组合，选择最可能的有效 Hamming 码字。 |
| `VoiceFEC_ParseDataSymbolsSoft` | 解三份长度并多数投票，反交织正文，用软 Chase 解码，最后验证 CRC；成功时输出 payload。 |
| `VoiceFEC_DataSymbolCount` | 根据 payload 字节数计算长度前缀和编码正文所需的总符号数。 |
| `VoiceFEC_BuildDataSymbols` | 构造三份冗余长度、`payload + CRC`，Hamming 编码并交织后转换为 FSK symbol。 |
| `VoiceFEC_DecodeLen` | 解码三份长度前缀，并按位多数投票得到 payload 长度。 |
| `VoiceFEC_ParseDataSymbols` | 硬判决兼容入口；实际转调软判决函数并传入空能量信息。 |

## stm32f4xx_hal_msp.c

CubeMX 生成的 MSP（硬件支持包）实现。它们不实现业务算法，而是在 HAL 初始化/反初始化时申请或释放外设时钟、GPIO、DMA 和 NVIC。

| 函数 | 具体内容及作用 |
|---|---|
| `HAL_MspInit` | 启用 SYSCFG/PWR 时钟，设置系统中断优先级分组。 |
| `HAL_ADC_MspInit` / `HAL_ADC_MspDeInit` | 为 ADC1 配置模拟输入、ADC 时钟、DMA2 Stream0 和 DMA 中断；反初始化时释放它们。 |
| `HAL_I2C_MspInit` / `HAL_I2C_MspDeInit` | 为 I2C1 配置 PB6/PB7 开漏复用和 I2C 时钟；反初始化时释放。 |
| `HAL_SPI_MspInit` / `HAL_SPI_MspDeInit` | 为 SPI1 配置 PA5/PA6/PA7 复用和 SPI 时钟；反初始化时释放。 |
| `HAL_TIM_Base_MspInit` / `HAL_TIM_Base_MspDeInit` | 启用/关闭 TIM1、TIM2、TIM3 时钟；TIM3 同时注册/注销 NVIC 中断。 |
| `HAL_TIM_MspPostInit` | 配置 TIM1_CH1 的 PA8 PWM 复用输出。 |

## stm32f4xx_it.c

| 函数 | 具体内容及作用 |
|---|---|
| `NMI_Handler` | 交给 HAL 处理 RCC NMI 后停在无限循环。 |
| `HardFault_Handler` | 出现硬错误时无限循环，保留现场以便调试。 |
| `MemManage_Handler` | 内存管理异常时无限循环。 |
| `BusFault_Handler` | 总线访问异常时无限循环。 |
| `UsageFault_Handler` | 非法指令或状态异常时无限循环。 |
| `SVC_Handler` / `DebugMon_Handler` / `PendSV_Handler` | 对应 Cortex-M 系统异常的空处理钩子。 |
| `SysTick_Handler` | 调用 `HAL_IncTick`，为 HAL 延时和超时提供系统毫秒计时。 |
| `EXTI1_IRQHandler` | 将电源按键 EXTI1 转交 HAL，最终触发 `HAL_GPIO_EXTI_Callback`。 |
| `TIM3_IRQHandler` | 将 TIM3 中断转交 HAL，最终触发发送的周期回调。 |
| `DMA2_Stream0_IRQHandler` | 将 ADC DMA 中断转交 HAL，最终触发两个半缓冲回调。 |

## syscalls.c 与 sysmem.c

这是 C 库在裸机环境所需的系统调用适配，不提供文件系统或进程。

| 函数 | 具体内容及作用 |
|---|---|
| `initialise_monitor_handles` | 空实现，满足库的半主机/监控初始化符号需求。 |
| `_getpid` | 固定返回伪进程 ID 1。 |
| `_kill` | 不支持信号，设 `EINVAL` 后失败。 |
| `_exit` | 调用 `_kill` 后无限循环，防止程序返回到不存在的 OS。 |
| `_read` | 逐字节调用弱符号 `__io_getchar`，为 `scanf` 等提供输入重定向点。 |
| `_write` | 逐字节调用弱符号 `__io_putchar`，为 `printf` 等提供输出重定向点。 |
| `_close` / `_open` | 不支持文件关闭或打开，返回失败。 |
| `_fstat` / `_stat` | 将对象伪装为字符设备，使标准 I/O 库可用。 |
| `_isatty` | 固定报告为终端。 |
| `_lseek` | 不支持真正定位，固定返回 0。 |
| `_wait` / `_times` / `_unlink` / `_link` / `_fork` / `_execve` | 分别是进程、时间和文件系统接口的裸机失败桩，按约定设置 errno 或返回 -1。 |
| `starm_putc`（仅 Picolibc，静态） | 将 Picolibc 的 `FILE` 单字符输出转到 `__io_putchar`。 |
| `starm_getc`（仅 Picolibc，静态） | 将 Picolibc 的 `FILE` 单字符输入转到 `__io_getchar`。 |
| `_sbrk`（`sysmem.c`） | 从链接器 `_end` 向上扩展堆，检查不会侵入预留 MSP 栈；是 `malloc` 的底层内存来源。 |

## system_stm32f4xx.c

这是 CMSIS/ST 设备启动文件的项目副本，负责在进入 `main` 前建立最小 MCU 系统状态。

| 函数 | 具体内容及作用 |
|---|---|
| `SystemInit` | 复位 RCC 时钟配置、关闭/清理中断和时钟源，并按芯片配置调用外部存储控制初始化。 |
| `SystemCoreClockUpdate` | 从 RCC 寄存器重新推导 `SystemCoreClock`，供依赖内核频率的库查询。 |
| `SystemInit_ExtMemCtl`（静态） | 在启用相关宏的构建下配置 FSMC 外部 SRAM/SDRAM 的 GPIO、时序和控制器；本板未使用时等效为空。 |

