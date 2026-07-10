/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body — 声语信使 半双工
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "oled.h"
#include "keyboard.h"
#include "editor.h"
#include "pwm_dds.h"
#include "transmitter.h"
#include "receiver.h"
#include "flash_store.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE  RX_DMA_BUF_SIZE  /* 800 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint8_t g_ready = 0;
volatile uint8_t g_power_off = 0;
static uint16_t adc_dma_buf[ADC_BUF_SIZE];

/* 半双工模式 */
#define HM_RX      0
#define HM_TX_EDIT 1
#define HM_TX_BUSY 2
static uint8_t  hm_mode = HM_RX;
static uint32_t last_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void HM_SwitchToRx(void);
static void HM_SwitchToTx(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline void Power_CutOff(void)
{
    GPIOB->BSRR = GPIO_BSRR_BR8;
}

/* ========================================================================== */
/*  半双工: RX/TX 模式切换                                                     */
/* ========================================================================== */

static void HM_SwitchToRx(void)
{
    /* 停止 TX 侧 */
    TX_ClearDone();                       /* 停 TIM3 ISR, PWM midscale */
    PWM_DDS_Shutdown();                   /* 停 TIM1 PWM */

    /* 启动 RX 侧 */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_BUF_SIZE);
    HAL_TIM_Base_Start(&htim2);           /* TIM2 → ADC trigger */
    RX_Start();

    hm_mode = HM_RX;
    /* 不清除编辑器 — 保留已编辑内容, 下次切回 TX 继续用 */
}

static void HM_SwitchToTx(void)
{
    /* 停止 RX 侧 */
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_TIM_Base_Stop(&htim2);
    RX_Stop();

    /* 启动 TX 侧 */
    PWM_DDS_Start();                      /* TIM1 PWM */
    TX_Start(Editor_GetBuffer());

    hm_mode = HM_TX_BUSY;
}

/* ── ADC DMA 回调 (RX 模式专用) ── */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && hm_mode == HM_RX) {
        RX_ProcessHalfBuffer(adc_dma_buf);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && hm_mode == HM_RX) {
        RX_ProcessHalfBuffer(adc_dma_buf + RX_BLOCK_SIZE);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  __DSB();
  GPIOB->MODER &= ~GPIO_MODER_MODER1;
  GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD1) | GPIO_PUPDR_PUPD1_0;
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER8) | GPIO_MODER_MODER8_0;
  for (volatile int i = 0; i < 200; i++) { __NOP(); }
  if ((GPIOB->IDR & GPIO_IDR_ID1) == 0) {
      GPIOB->BSRR = GPIO_BSRR_BS8;
  } else {
      GPIOB->BSRR = GPIO_BSRR_BR8;
      for (volatile int i = 0; i < 800000; i++) { __NOP(); }
  }

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);

  /* 冷启动延时: 等 OLED VDD 稳定 (手册要求 1-50ms) */
  HAL_Delay(50);

  OLED_Init(&hi2c1);
  Keyboard_Init();
  Editor_Init();

  PWM_DDS_Init(&htim1);
  TX_Init();
  FlashStore_Init();
  RX_Init();

  /* 首次启动 TIM3 (transmitter.c 的 TX_ClearDone 会停止它) */
  HAL_TIM_Base_Start_IT(&htim3);

  /* 开机画面 */
  OLED_ShowString(12, 0,  "Voice Messenger");
  OLED_ShowString(0,  16, " Half-Duplex");
  OLED_ShowString(0,  32, "Rx: Stand By");
  OLED_Refresh();
  HAL_Delay(500);

  /* 默认进入 RX 模式 */
  hm_mode = HM_RX;
  HM_SwitchToRx();

  while (HAL_GPIO_ReadPin(POWER_BUTTON_GPIO_Port, POWER_BUTTON_Pin) == GPIO_PIN_RESET) {
      HAL_Delay(10);
  }
  HAL_Delay(50);
  g_ready = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* ── RX 内部子模式 ── */
  #define LS_LISTENING    0   /* 显示上次接收消息 */
  #define LS_BROWSE_LIST  1   /* 已存储消息列表 */
  #define LS_BROWSE_VIEW  2   /* 查看单条存储消息 */

  uint8_t  ls_mode        = LS_LISTENING;
  uint8_t  browse_cursor  = 0;
  uint8_t  view_scroll    = 0;
  uint8_t  last_key_fn    = 0;
  uint8_t  last_key_del   = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_power_off) {
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop(&htim2);
        HAL_TIM_Base_Stop_IT(&htim3);
        RX_Stop();
        PWM_DDS_Shutdown();
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        OLED_Clear();
        OLED_Refresh();
        while (1) { __NOP(); }
    }

    uint8_t key = Keyboard_Scan();

    /* ── KEY_FN 边沿检测 ── */
    uint8_t key_fn_now  = (key == KEY_FN) ? 1 : 0;
    uint8_t key_fn_edge = key_fn_now && !last_key_fn;
    last_key_fn = key_fn_now;

    /* ═══════════════════════════════════════════════════════ */
    /*  RX 接收模式                                              */
    /* ═══════════════════════════════════════════════════════ */
    if (hm_mode == HM_RX) {

        /* ── KEY_FN: 退出浏览 / 切换到发送编辑 ── */
        if (key_fn_edge) {
            if (ls_mode != LS_LISTENING) {
                ls_mode = LS_LISTENING;
            } else {
                hm_mode = HM_TX_EDIT;
                Editor_SetTxStatus("Tx Ready");
            }
            last_tick = 0;
            continue;
        }

        /* ── KEY_SEND → 浏览已存储消息 / 退出浏览 ── */
        if (key == KEY_SEND) {
            if (ls_mode == LS_LISTENING) {
                ls_mode       = LS_BROWSE_LIST;
                browse_cursor = 0;
            } else if (ls_mode == LS_BROWSE_VIEW) {
                ls_mode     = LS_BROWSE_LIST;
                view_scroll = 0;
            }
        }

        /* ── T9 键 → 进入编辑发信 (退出浏览) ── */
        if (key >= KEY_0 && key <= KEY_9) {
            ls_mode = LS_LISTENING;
            hm_mode = HM_TX_EDIT;
            Editor_HandleKey(key);
            Editor_SetTxStatus("Tx Ready");
            continue;
        }

        /* ── 接收完成: 2s 显示 → 自动重启 ── */
        if (RX_IsDone()) {
            const char *rx_msg = RX_GetMessage();
            uint8_t    rx_len  = RX_GetMessageLength();
            uint8_t rls[50], rll[50], rtl = 0; uint16_t rp = 0;
            while (rp < rx_len) {
                rls[rtl] = (uint8_t)rp; uint8_t rl = 0;
                while (rp + rl < rx_len && rx_msg[rp + rl] != '\n' && rl < DISP_COLS) rl++;
                rll[rtl] = rl; rtl++; rp += rl;
                if (rp < rx_len && rx_msg[rp] == '\n') rp++;
            }
            if (rx_len > 0 && rx_msg[rx_len - 1] == '\n')
                { rls[rtl] = rx_len; rll[rtl] = 0; rtl++; }
            if (rtl == 0) { rtl = 1; rls[0] = 0; rll[0] = 0; }

            OLED_Clear();
            uint8_t sl = 0;
            if (rtl > VISIBLE_ROWS) sl = rtl - VISIBLE_ROWS;
            for (uint8_t row = 0; row < VISIBLE_ROWS; row++) {
                uint8_t li = sl + row; if (li >= rtl) break;
                uint8_t py = row * FONT_HEIGHT;
                for (uint8_t c = 0; c < rll[li]; c++)
                    OLED_ShowChar(c * FONT_WIDTH, py, rx_msg[rls[li] + c]);
            }
            char cbuf[10];
            snprintf(cbuf, sizeof(cbuf), "Count:%d", rx_len);
            OLED_ShowString(0, 7 * FONT_HEIGHT, cbuf);
            const char *done_status = "Rx Complete";
            OLED_ShowString(OLED_WIDTH - (uint8_t)strlen(done_status) * FONT_WIDTH,
                            7 * FONT_HEIGHT, done_status);
            OLED_Refresh();
            HAL_Delay(2000);
            RX_ClearDone();
            RX_Start();
            last_tick = 0;
        }

        /* ── 定时刷新显示 ── */
        uint32_t tick = HAL_GetTick();
        uint8_t  refresh = 0;
        if ((tick - last_tick) >= 200) refresh = 1;

        const char *msg = RX_GetDisplayMessage();
        uint8_t mlen = RX_GetDisplayLength();
        uint8_t ls[50], ll[50], tl = 0; uint16_t p = 0;
        while (p < mlen) {
            ls[tl] = (uint8_t)p; uint8_t l = 0;
            while (p + l < mlen && msg[p + l] != '\n' && l < DISP_COLS) l++;
            ll[tl] = l; tl++; p += l;
            if (p < mlen && msg[p] == '\n') p++;
        }
        if (mlen > 0 && msg[mlen - 1] == '\n')
            { ls[tl] = mlen; ll[tl] = 0; tl++; }
        if (tl == 0) { tl = 1; ls[0] = 0; ll[0] = 0; }

        if (key == KEY_LEFT)  RX_ScrollWrapUp(tl);
        if (key == KEY_RIGHT) RX_ScrollDown(tl);

        uint8_t cs = RX_GetScrollLine();
        static uint8_t lcs = 0xFF;
        if (refresh || cs != lcs) {
            lcs = cs;
            OLED_Clear();
            uint8_t sl2 = cs;
            if (tl > VISIBLE_ROWS && sl2 + VISIBLE_ROWS > tl)
                sl2 = tl - VISIBLE_ROWS;
            if (tl <= VISIBLE_ROWS) sl2 = 0;
            for (uint8_t row = 0; row < VISIBLE_ROWS; row++) {
                uint8_t li = sl2 + row; if (li >= tl) break;
                uint8_t py = row * FONT_HEIGHT;
                for (uint8_t c = 0; c < ll[li]; c++)
                    OLED_ShowChar(c * FONT_WIDTH, py, msg[ls[li] + c]);
            }
            const char *st = RX_GetStatusString();
            char cbuf2[10];
            snprintf(cbuf2, sizeof(cbuf2), "Count:%d", mlen);
            OLED_ShowString(0, 7 * FONT_HEIGHT, cbuf2);
            OLED_ShowString(OLED_WIDTH - (uint8_t)strlen(st) * FONT_WIDTH,
                            7 * FONT_HEIGHT, st);
            OLED_Refresh();
        }

        /* ═══════════════════════════════════════════════════ */
        /*  LS_BROWSE_LIST: 已存储消息列表                       */
        /* ═══════════════════════════════════════════════════ */
        else if (ls_mode == LS_BROWSE_LIST) {
            uint8_t count = FlashStore_GetCount();

            uint8_t nav_changed = 0;
            if (key == KEY_LEFT || key == KEY_RIGHT) {
                if (count > 0) {
                    if (key == KEY_RIGHT) {
                        browse_cursor++;
                        if (browse_cursor >= count) browse_cursor = 0;
                    } else {
                        if (browse_cursor == 0) browse_cursor = count - 1;
                        else browse_cursor--;
                    }
                    nav_changed = 1;
                }
            }

            uint8_t key_del_now  = (key == KEY_DELETE) ? 1 : 0;
            uint8_t key_del_edge = key_del_now && !last_key_del;
            last_key_del = key_del_now;
            if (key_del_edge && count > 0) {
                FlashStore_DeleteMessage(browse_cursor);
                count = FlashStore_GetCount();
                if (count > 0 && browse_cursor >= count)
                    browse_cursor = count - 1;
            }

            if (key >= KEY_0 && key <= KEY_9 && count > 0) {
                ls_mode     = LS_BROWSE_VIEW;
                view_scroll = 0;
            }

            if (refresh || nav_changed) {
                OLED_Clear();
                char hdr[22];
                snprintf(hdr, sizeof(hdr), "Stored Msgs (%d)", count);
                OLED_ShowString(0, 0, hdr);
                if (count == 0) {
                    OLED_ShowString(0, 2 * FONT_HEIGHT, "(no messages)");
                } else {
                    for (uint8_t i = 0; i < count && i < 5; i++) {
                        const FlashStore_MsgSlot *slot = FlashStore_GetMessage(i);
                        uint8_t ry = (uint8_t)(1 + i) * FONT_HEIGHT;
                        char pr[4];
                        snprintf(pr, sizeof(pr), "%c%d:",
                                 (i == browse_cursor) ? '>' : ' ', i + 1);
                        OLED_ShowString(0, ry, pr);
                        if (slot && slot->valid) {
                            char preview[19]; uint8_t plen = slot->length;
                            if (plen > 14) plen = 14;
                            memcpy(preview, slot->data, plen);
                            if (slot->length > 14) { memcpy(preview + 14, "...", 3); plen += 3; }
                            preview[plen] = '\0';
                            OLED_ShowString(4 * FONT_WIDTH, ry, preview);
                        }
                    }
                }
                OLED_ShowString(0, 7 * FONT_HEIGHT,
                                "[英/数]Back [发送]View");
                OLED_Refresh();
            }
        }

        /* ═══════════════════════════════════════════════════ */
        /*  LS_BROWSE_VIEW: 查看单条消息                         */
        /* ═══════════════════════════════════════════════════ */
        else if (ls_mode == LS_BROWSE_VIEW) {
            const FlashStore_MsgSlot *slot = FlashStore_GetMessage(browse_cursor);
            if (!slot || !slot->valid) {
                ls_mode = LS_BROWSE_LIST; view_scroll = 0;
            } else {
                uint8_t vls[50], vll[50], vtl = 0; uint16_t vp = 0;
                while (vp < slot->length) {
                    vls[vtl] = (uint8_t)vp; uint8_t vl = 0;
                    while (vp + vl < slot->length && slot->data[vp + vl] != '\n'
                           && vl < DISP_COLS) vl++;
                    vll[vtl] = vl; vtl++; vp += vl;
                    if (vp < slot->length && slot->data[vp] == '\n') vp++;
                }
                if (slot->length > 0 && slot->data[slot->length - 1] == '\n')
                    { vls[vtl] = slot->length; vll[vtl] = 0; vtl++; }
                if (vtl == 0) { vtl = 1; vls[0] = 0; vll[0] = 0; }

                if (key == KEY_LEFT || key == KEY_RIGHT) {
                    if (vtl > VISIBLE_ROWS) {
                        if (key == KEY_RIGHT) {
                            if (view_scroll < vtl - VISIBLE_ROWS) view_scroll++;
                        } else {
                            if (view_scroll > 0) view_scroll--;
                        }
                    }
                }

                if (key == KEY_DELETE) {
                    ls_mode     = LS_BROWSE_LIST;
                    view_scroll = 0;
                }

                if (refresh || key == KEY_LEFT || key == KEY_RIGHT) {
                    OLED_Clear();
                    char vhdr[16];
                    snprintf(vhdr, sizeof(vhdr), "Msg %d/%d",
                             browse_cursor + 1, FlashStore_GetCount());
                    OLED_ShowString(0, 0, vhdr);
                    for (uint8_t row = 0; row < VISIBLE_ROWS; row++) {
                        uint8_t li = view_scroll + row; if (li >= vtl) break;
                        uint8_t py = (uint8_t)(1 + row) * FONT_HEIGHT;
                        for (uint8_t c = 0; c < vll[li]; c++)
                            OLED_ShowChar(c * FONT_WIDTH, py, slot->data[vls[li] + c]);
                    }
                    OLED_ShowString(0, 7 * FONT_HEIGHT,
                                    "[发送]Back [L/R]Scroll");
                    OLED_Refresh();
                }
            }
        }

        if (refresh) last_tick = tick;
        HAL_Delay(10);
    }

    /* ═══════════════════════════════════════════════════════ */
    /*  TX 编辑模式                                              */
    /* ═══════════════════════════════════════════════════════ */
    else if (hm_mode == HM_TX_EDIT) {

        /* ── 光标在末尾悬空 + KEY_RIGHT → 切回收信 ── */
        if (key == KEY_RIGHT && Editor_IsCursorAtEnd()) {
            HM_SwitchToRx();
            last_tick = 0;
            continue;
        }

        /* ── 其他按键 ── */
        if (key != KEY_NONE) {
            if (key == KEY_SEND) {
                if (Editor_GetLength() > 0) {
                    HM_SwitchToTx();
                    continue;
                }
            }
            else {
                Editor_HandleKey(key);
            }
        }

        Editor_SetTxStatus("Tx Ready");

        uint32_t now = HAL_GetTick();
        if ((now - last_tick) >= 50) { last_tick = now; Editor_Tick(); }
        Editor_UpdateDisplay();
        HAL_Delay(10);
    }

    /* ═══════════════════════════════════════════════════════ */
    /*  TX 发送中                                                */
    /* ═══════════════════════════════════════════════════════ */
    else if (hm_mode == HM_TX_BUSY) {
        if (TX_IsDone()) {
            Editor_SetTxStatus("Tx Complete");
            Editor_UpdateDisplay();
            HAL_Delay(1500);
            /* 发完回到编辑模式, 保留编辑器内容 (与发送端单工一致) */
            TX_ClearDone();
            PWM_DDS_Shutdown();
            hm_mode = HM_TX_EDIT;
            Editor_SetTxStatus("Tx Ready");
            last_tick = 0;
            continue;
        }
        Editor_SetTxStatus("Tx Active");
        Editor_UpdateDisplay();
        HAL_Delay(10);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1023;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 512;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3124;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 624;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, KB_ROW0_Pin|KB_ROW1_Pin|KB_ROW2_Pin|KB_ROW3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_CTRL_GPIO_Port, POWER_CTRL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KB_ROW0_Pin KB_ROW1_Pin KB_ROW2_Pin KB_ROW3_Pin */
  GPIO_InitStruct.Pin = KB_ROW0_Pin|KB_ROW1_Pin|KB_ROW2_Pin|KB_ROW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KB_COL0_Pin KB_COL1_Pin KB_COL2_Pin KB_COL3_Pin */
  GPIO_InitStruct.Pin = KB_COL0_Pin|KB_COL1_Pin|KB_COL2_Pin|KB_COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_BUTTON_Pin */
  GPIO_InitStruct.Pin = POWER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(POWER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_CTRL_Pin */
  GPIO_InitStruct.Pin = POWER_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_CTRL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == POWER_BUTTON_Pin && g_ready) {
        g_power_off = 1;
        Power_CutOff();
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
