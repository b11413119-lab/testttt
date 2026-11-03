/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 電子寵物 - NUCLEO-G474RE(完整版)
  *                   眼睛:PA0 紅 / PA1 綠 / PA4 藍
  *                   尾巴:28BYJ-48 + ULN2003 -> PB3~PB6
  *                   按鈕1:PC0(換眼色,軟體去彈跳)
  *                   按鈕3:PA10(EXTI 緊急切換)
  *                   UART :USART2 115200 8N1(環形緩衝接收)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>   // rand()、srand()
#include <string.h>   // strlen()
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 眼睛顏色:藍、綠往復切換
typedef enum {
  EYE_BLUE = 0,
  EYE_GREEN
} EyeColor_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEP_PERIOD_MS    3      // 步進馬達每 3ms 走一步(固定速度)
#define WAG_STEPS         400    // 單邊搖晃步數(半步模式),可自行調整
#define BTN1_DEBOUNCE_MS  20     // 按鈕 1 需穩定 20ms 才算有效
#define BTN3_CONFIRM_MS   20     // 按鈕 3 中斷後,需持續為低 20ms 才確認「真的按下」
#define BTN3_RELEASE_MS   30     // 按鈕 3 需持續為高 30ms 才確認「已放開」,之後才能再次觸發
#define RX_RING_SIZE      128    // 環形緩衝大小(必須為 2 的次方)
#define LINE_MAX_LEN      64     // 一行最多接受的字元數

// 若 CubeMX 的 User Label 沒設定,這裡提供備用定義,避免編譯錯誤
#ifndef LED_R_Pin
#define LED_R_Pin        GPIO_PIN_0
#define LED_R_GPIO_Port  GPIOA
#endif
#ifndef LED_G_Pin
#define LED_G_Pin        GPIO_PIN_1
#define LED_G_GPIO_Port  GPIOA
#endif
#ifndef LED_B_Pin
#define LED_B_Pin        GPIO_PIN_4
#define LED_B_GPIO_Port  GPIOA
#endif
#ifndef IN1_Pin
#define IN1_Pin          GPIO_PIN_3
#define IN1_GPIO_Port    GPIOB
#endif
#ifndef IN2_Pin
#define IN2_Pin          GPIO_PIN_4
#define IN2_GPIO_Port    GPIOB
#endif
#ifndef IN3_Pin
#define IN3_Pin          GPIO_PIN_5
#define IN3_GPIO_Port    GPIOB
#endif
#ifndef IN4_Pin
#define IN4_Pin          GPIO_PIN_6
#define IN4_GPIO_Port    GPIOB
#endif
#ifndef BTN1_Pin
#define BTN1_Pin         GPIO_PIN_0
#define BTN1_GPIO_Port   GPIOC
#endif
#ifndef BTN3_Pin
#define BTN3_Pin         GPIO_PIN_10
#define BTN3_GPIO_Port   GPIOA
#endif

// 強制使用下列腳位,不管 main.h 裡的 User Label 設在哪個腳,避免 Label 設錯導致讀錯腳
#undef LED_R_Pin
#undef LED_R_GPIO_Port
#undef LED_G_Pin
#undef LED_G_GPIO_Port
#undef LED_B_Pin
#undef LED_B_GPIO_Port
#undef IN1_Pin
#undef IN1_GPIO_Port
#undef IN2_Pin
#undef IN2_GPIO_Port
#undef IN3_Pin
#undef IN3_GPIO_Port
#undef IN4_Pin
#undef IN4_GPIO_Port
#undef BTN1_Pin
#undef BTN1_GPIO_Port
#undef BTN3_Pin
#undef BTN3_GPIO_Port
#define LED_R_Pin        GPIO_PIN_0     // 紅眼 PA0
#define LED_R_GPIO_Port  GPIOA
#define LED_G_Pin        GPIO_PIN_1     // 綠眼 PA1
#define LED_G_GPIO_Port  GPIOA
#define LED_B_Pin        GPIO_PIN_4     // 藍眼 PA4
#define LED_B_GPIO_Port  GPIOA
#define IN1_Pin          GPIO_PIN_3     // 馬達 IN1 PB3
#define IN1_GPIO_Port    GPIOB
#define IN2_Pin          GPIO_PIN_4     // 馬達 IN2 PB4
#define IN2_GPIO_Port    GPIOB
#define IN3_Pin          GPIO_PIN_5     // 馬達 IN3 PB5
#define IN3_GPIO_Port    GPIOB
#define IN4_Pin          GPIO_PIN_6     // 馬達 IN4 PB6
#define IN4_GPIO_Port    GPIOB
#define BTN1_Pin         GPIO_PIN_0     // 按鈕1 PC0(Nucleo 排針 A5)
#define BTN1_GPIO_Port   GPIOC
#define BTN3_Pin         GPIO_PIN_10    // 按鈕3 PA10(Nucleo 排針 D2)
#define BTN3_GPIO_Port   GPIOA

// 診斷開關:1 = 透過 UART 印出按鈕腳位電位與狀態變化;確認功能正常後請改成 0
#define BTN_DIAG          1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// ---------- 狀態旗標 ----------
static volatile EyeColor_t g_eye_color = EYE_BLUE;  // 目前眼睛顏色(藍/綠)
static volatile uint8_t    g_emergency = 0;         // 緊急狀態旗標(1=緊急)

// 按鈕 3 去彈跳狀態機:0=待命(等 EXTI)、1=確認按下中、2=等待放開
static volatile uint8_t    btn3_state = 0;
static volatile uint8_t    btn3_cnt   = 0;

// ---------- 步進馬達(28BYJ-48 半步序列,對應 IN1~IN4)----------
static const uint8_t half_step[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};
static uint8_t  step_phase = 0;   // 目前在半步序列的第幾相(0~7)
static uint8_t  step_dir   = 1;   // 方向:1=正轉、0=反轉
static uint16_t step_count = 0;   // 本方向已走的步數
static uint8_t  coil_off   = 0;   // 線圈是否已斷電(避免重複寫入)

// ---------- UART 環形緩衝(接收)----------
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;   // 寫入位置(中斷內更新)
static volatile uint16_t rx_tail = 0;   // 讀取位置(主迴圈更新)
static uint8_t rx_byte;                 // HAL 單一位元組接收用

// ---------- 對話 ----------
static uint8_t  seeded   = 0;           // 亂數種子是否已設定
static uint16_t line_len = 0;           // 目前這一行已收字元數

// 四種隨機回覆(英文)
static const char *replies[4] = {
  "Pet: I'm happy! :)\r\n",
  "Pet: I'm sad... :(\r\n",
  "Pet: I'm angry! >:(\r\n",
  "Pet: I'm sleepy... zzz\r\n"
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================= 眼睛 LED ================= */
// 依目前狀態點亮對應顏色:緊急=紅,否則藍或綠(同時只亮一色)
static void Eye_Update(void)
{
  GPIO_PinState r = GPIO_PIN_RESET;
  GPIO_PinState g = GPIO_PIN_RESET;
  GPIO_PinState b = GPIO_PIN_RESET;

  if (g_emergency) {
    r = GPIO_PIN_SET;                 // 緊急狀態:紅眼
  } else if (g_eye_color == EYE_BLUE) {
    b = GPIO_PIN_SET;                 // 藍眼
  } else {
    g = GPIO_PIN_SET;                 // 綠眼
  }

  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, r);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, g);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, b);
}

/* ================= 步進馬達 ================= */
// 依序列輸出一組線圈狀態到 PB3~PB6
static void Stepper_Write(uint8_t phase)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, half_step[phase][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, half_step[phase][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, half_step[phase][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, half_step[phase][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// 全部線圈斷電(緊急停止時使用,避免馬達持續發熱)
static void Stepper_Release(void)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

// 走一步:正轉走滿 WAG_STEPS 後換成反轉,如此往復 = 尾巴搖晃
static void Tail_Step(void)
{
  if (step_dir) {
    step_phase = (step_phase + 1) & 0x07;   // 正轉:相位 +1
  } else {
    step_phase = (step_phase + 7) & 0x07;   // 反轉:相位 -1(等同 +7 取模 8)
  }
  Stepper_Write(step_phase);
  coil_off = 0;

  if (++step_count >= WAG_STEPS) {          // 走滿一邊,換方向
    step_count = 0;
    step_dir ^= 1;
  }
}

/* ================= UART 環形緩衝 ================= */
// 放入一個位元組(在接收中斷內呼叫);緩衝滿了就丟棄新資料
static void Ring_Push(uint8_t b)
{
  uint16_t next = (rx_head + 1) & (RX_RING_SIZE - 1);
  if (next != rx_tail) {
    rx_ring[rx_head] = b;
    rx_head = next;
  }
}

// 取出一個位元組(在主迴圈呼叫);成功回傳 1,沒資料回傳 0
static uint8_t Ring_Pop(uint8_t *b)
{
  if (rx_head == rx_tail) return 0;
  *b = rx_ring[rx_tail];
  rx_tail = (rx_tail + 1) & (RX_RING_SIZE - 1);
  return 1;
}

/* ================= 對話 ================= */
// 傳送字串(阻塞式傳送,字串很短不影響其他功能)
static void Uart_Print(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

// 主迴圈呼叫:處理環形緩衝中收到的資料
static void Pet_UartTask(void)
{
  uint8_t c;

  while (Ring_Pop(&c)) {
    // 第一次收到字元時,用當下的 tick 當亂數種子
    if (!seeded) {
      srand(HAL_GetTick());
      seeded = 1;
    }

    if (c == '\r' || c == '\n') {
      // 收到換行:若這行有內容,回覆一句隨機的話
      if (line_len > 0) {
        line_len = 0;
        Uart_Print("\r\n");
        Uart_Print(replies[rand() % 4]);   // 隨機選 happy/sad/angry/sleep
      }
    } else if (line_len < LINE_MAX_LEN) {
      line_len++;
      HAL_UART_Transmit(&huart2, &c, 1, 10);   // 回顯(echo)
    }
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
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Eye_Update();                                  // 開機先亮藍眼
  Uart_Print("Pet ready! Say something and press Enter.\r\n");

  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);     // 啟動 UART 中斷接收(一次 1 byte)
  HAL_TIM_Base_Start_IT(&htim6);                 // 啟動 TIM6 的 1ms 中斷
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Pet_UartTask();   // 處理 UART 收到的資料並回覆
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration(HSI 16MHz -> PLL -> 170MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;     // 16MHz / 4 = 4MHz
  RCC_OscInitStruct.PLL.PLLN = 85;                // 4MHz x 85 = 340MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;     // 340MHz / 2 = 170MHz
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM6 Initialization Function(1kHz = 1ms 中斷)
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;                     // 170MHz / 170 = 1MHz
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;                        // 1MHz / 1000 = 1kHz
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function(115200 8N1)
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* 輸出腳位初始電平:全部 Low */
  HAL_GPIO_WritePin(GPIOA, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /* 眼睛 LED:PA0 / PA1 / PA4 推挽輸出 */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 步進馬達:PB3~PB6 推挽輸出 */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 按鈕 1:PC0 輸入 + 上拉(用 TIM6 輪詢去彈跳) */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /* 按鈕 3:PA10 外部中斷(下降緣)+ 上拉 */
  GPIO_InitStruct.Pin = BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI line[15:10] 中斷:最高優先權 */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/* ---------- TIM6 每 1ms 中斷一次 ---------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6) return;

  /* (1) 按鈕 1 軟體去彈跳:需連續 BTN1_DEBOUNCE_MS 毫秒與穩定狀態不同,才算真的變化 */
  static uint8_t  btn1_stable = GPIO_PIN_SET;   // 穩定狀態(pull-up,放開=高)
  static uint8_t  btn1_cnt    = 0;
  uint8_t raw = (uint8_t)HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin);

  if (raw != btn1_stable) {
    if (++btn1_cnt >= BTN1_DEBOUNCE_MS) {
      btn1_stable = raw;
      btn1_cnt = 0;
      if (btn1_stable == GPIO_PIN_RESET) {      // 確認「按下」才切換顏色
        g_eye_color = (g_eye_color == EYE_BLUE) ? EYE_GREEN : EYE_BLUE;
      }
    }
  } else {
    btn1_cnt = 0;                               // 狀態相同,計數歸零
  }

  /* (1b) 按鈕 3 狀態機:EXTI 只負責「喚醒」,真正的確認與放開判斷在這裡 */
  if (btn3_state == 1) {                        // 確認按下中
    if (HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == GPIO_PIN_RESET) {
      if (++btn3_cnt >= BTN3_CONFIRM_MS) {      // 持續為低 -> 真的按下
        g_emergency ^= 1;                       // 切換緊急(進入/解除)
        btn3_state = 2;                         // 進入等待放開
        btn3_cnt = 0;
      }
    } else {                                    // 中途變回高電位 -> 只是雜訊,放棄
      btn3_state = 0;
      btn3_cnt = 0;
    }
  } else if (btn3_state == 2) {                 // 等待放開
    if (HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin) == GPIO_PIN_SET) {
      if (++btn3_cnt >= BTN3_RELEASE_MS) {      // 持續為高 -> 真的放開,重新待命
        btn3_state = 0;
        btn3_cnt = 0;
      }
    } else {
      btn3_cnt = 0;                             // 還在按著或放開時彈跳,重新計時
    }
  }

  /* (2) 尾巴:固定速度,每 STEP_PERIOD_MS 走一步;緊急時停止並斷電 */
  static uint16_t step_timer = 0;
  if (g_emergency) {
    if (!coil_off) {                            // 只在剛進入緊急時斷電一次
      Stepper_Release();
      coil_off = 1;
    }
    step_timer = 0;
  } else {
    if (++step_timer >= STEP_PERIOD_MS) {
      step_timer = 0;
      Tail_Step();
    }
  }

  /* (3) 依狀態更新眼睛顏色 */
  Eye_Update();
}

/* ---------- 按鈕 3(PA10)EXTI 處理(共用)---------- */
static void Btn3_OnExti(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BTN3_Pin) {
    // 只在待命狀態才接受;確認與切換交給 TIM6 的狀態機,放開前的彈跳會被忽略
    if (btn3_state == 0) {
      btn3_cnt = 0;
      btn3_state = 1;
    }
  }
}

// 新版 HAL(有分上升緣/下降緣回呼)
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  Btn3_OnExti(GPIO_Pin);
}

// 舊版 HAL(只有單一回呼)的相容寫法,新版不會呼叫它,留著無害
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Btn3_OnExti(GPIO_Pin);
}


/* ---------- UART 接收完成中斷:把收到的 byte 存入環形緩衝 ---------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    Ring_Push(rx_byte);                          // 存入環形緩衝
    HAL_UART_Receive_IT(huart, &rx_byte, 1);     // 重新啟動下一次接收
  }
}

/* ---------- UART 錯誤中斷(例如 Overrun):清除旗標並重新接收 ---------- */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(huart, &rx_byte, 1);
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

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
