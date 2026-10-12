/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // 수신 패킷 파싱(sscanf)용
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 이 시간(ms) 동안 유효 패킷이 한 번도 오지 않으면 링크 두절로 판단한다.
 * 파이는 약 20Hz(50ms)로 송신하므로 300ms는 6프레임 연속 유실에 해당한다. */
#define RPI_LINK_TIMEOUT_MS   300U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vMotorTask */
osThreadId_t vMotorTaskHandle;
const osThreadAttr_t vMotorTask_attributes = {
  .name = "vMotorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vEmergencyTask */
osThreadId_t vEmergencyTaskHandle;
const osThreadAttr_t vEmergencyTask_attributes = {
  .name = "vEmergencyTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myEmergencySem */
osSemaphoreId_t myEmergencySemHandle;
const osSemaphoreAttr_t myEmergencySem_attributes = {
  .name = "myEmergencySem"
};

/* USER CODE BEGIN PV */
volatile uint8_t is_emergency = 0; // 비상 정지 플래그 (0: 정상 구동, 1: 비상정지 발동)

/* 라즈베리파이 UART 수신 버퍼 변수 */
uint8_t rx_data;          // HAL이 1바이트씩 수신해 채우는 버퍼
char rx_buf[30];          // 개행까지 한 줄을 조립하는 버퍼
uint8_t rx_idx = 0;       // rx_buf 내 다음 저장 위치

/* 실시간 픽셀 오차 보관용 전역 변수 */
volatile int32_t rpi_err_x = 0;
volatile int32_t rpi_err_y = 0;

/* 타겟 유효성 및 링크 상태 (USART1 ISR에서 write, 모터 태스크에서 read) */
volatile uint8_t  rpi_target_valid = 0;   // 1: 현재 프레임에서 사람 검출, 0: 미검출
volatile uint32_t rpi_last_rx_tick = 0;   // 마지막 유효 패킷 수신 시각 (HAL tick)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartMotorTask(void *argument);
void StartEmergencyTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  // 주변장치 초기화 완료 표시 (LED on)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  // UART 수신 인터럽트 최초 무장. 이후에는 수신 콜백이 스스로 재무장한다.
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the semaphore(s) */
  myEmergencySemHandle = osSemaphoreNew(1, 0, &myEmergencySem_attributes);

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  vMotorTaskHandle = osThreadNew(StartMotorTask, NULL, &vMotorTask_attributes);
  vEmergencyTaskHandle = osThreadNew(StartEmergencyTask, NULL, &vEmergencyTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1)
  {
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* TIM3는 APB1(PCLK1 = 42MHz)에 연결되어 있으나, APB1 프리스케일러가 1이 아니면
   * 타이머 클럭은 PCLK의 2배가 되므로 실제 TIM3 클럭은 84MHz이다.
   *   84MHz / (83 + 1)  = 1MHz        -> 1틱 = 1us
   *   (19999 + 1)틱     = 20,000us    -> 20ms = 50Hz (표준 서보 주기)          */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // 채널 1 (PA6) : 팬(X축)
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500; // 1500us = 90도 (중립)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // 채널 2 (PA7) : 틸트(Y축)
  sConfigOC.Pulse = 1500;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART1 Initialization Function
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 (사용자 버튼 B1, 비상정지) */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 (LD2 보드 내장 LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI 인터럽트 활성화.
   * 우선순위 5는 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY와 같은 값으로,
   * ISR에서 FreeRTOS API를 호출할 수 있는 가장 높은(숫자가 작은) 우선순위다.
   * 이보다 숫자가 작으면 커널 임계구역에 막히지 않아 세마포어 반환이 위험해진다. */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    // ISR에서는 세마포어 반환만 하고 실제 처리는 vEmergencyTask에 위임한다.
    osSemaphoreRelease(myEmergencySemHandle);
  }
}

/* USART1 1바이트 수신 완료 콜백. 개행을 만나면 한 줄을 파싱한다. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    // 수신 활성 표시
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

    if(rx_data == '\n') // 개행 = 패킷 한 개 완성
    {
      rx_buf[rx_idx] = '\0'; // 널 종단

      int32_t parsed_x = 0;
      int32_t parsed_y = 0;

      int32_t parsed_d = 1;   // D 필드가 없는 v1 패킷은 '검출됨'으로 간주

      // [프로토콜 v2] X<오차>Y<오차>D<0|1>   D: 사람 검출 여부
      if(sscanf(rx_buf, "X%dY%dD%d", &parsed_x, &parsed_y, &parsed_d) == 3)
      {
        rpi_err_x        = parsed_x;
        rpi_err_y        = parsed_y;
        rpi_target_valid = (parsed_d != 0) ? 1 : 0;
        rpi_last_rx_tick = HAL_GetTick();
      }
      // [프로토콜 v1 하위 호환] X<오차>Y<오차>
      else if(sscanf(rx_buf, "X%dY%d", &parsed_x, &parsed_y) == 2)
      {
        rpi_err_x        = parsed_x;
        rpi_err_y        = parsed_y;
        rpi_target_valid = 1;
        rpi_last_rx_tick = HAL_GetTick();
      }
      rx_idx = 0; // 다음 줄을 위해 초기화
    }
    else
    {
      if(rx_idx < sizeof(rx_buf) - 1) // 오버플로우 방지
      {
        rx_buf[rx_idx++] = rx_data;
      }
    }

    // 다음 1바이트 수신을 위해 재무장. 이 호출이 빠지면 수신이 멈춘다.
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
  }
}

/* UART 에러 콜백.
 *
 * HAL_UART_Receive_IT()는 1바이트를 받으면 수신을 종료하고 gState를 READY로
 * 되돌리므로, 콜백에서 다시 무장해야 수신이 이어진다. 그런데 콜백 처리가 늦어진
 * 사이에 다음 바이트가 도착하면 ORE(Overrun Error)가 발생하고, HAL은
 * RxCpltCallback 대신 이 콜백으로 진입한다. 여기서 재무장하지 않으면
 * 수신이 영구히 멈춘다.
 *
 * 플래그를 먼저 지우는 이유: 에러 플래그가 남아 있으면 인터럽트가 즉시
 * 재발생해 무한 루프에 빠진다. 따라서 클리어 -> 재무장 순서를 지킨다.       */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);

    HAL_UART_Receive_IT(&huart1, &rx_data, 1); // 수신 재무장
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
/* USER CODE END Header_StartDefaultTask */

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief 2축(팬/틸트) 서보 제어 태스크. 50Hz 고정 주기로 동작한다.
*/
void StartMotorTask(void *argument)
{
  // 서보 중립(90도)에서 시작
  double x_current = 90.0;
  double x_error = 0.0;
  double x_prev_error = 0.0;
  double x_derivative = 0.0;
  double x_control = 0.0;

  double y_current = 90.0;
  double y_error = 0.0;
  double y_prev_error = 0.0;
  double y_derivative = 0.0;
  double y_control = 0.0;

  // 제어 게인. 진동이 발생하지 않는 범위에서 보수적으로 설정 (실측 튜닝 전)
  double Kp_x = 0.0008;   double Kd_x = 0.0002;
  double Kp_y = 0.0006;   double Kd_y = 0.0002;

  // 1: 추적 중 / 0: 타겟 소실 또는 링크 두절 → 현재 각도 유지(hold)
  uint8_t tracking = 0;

  uint32_t pwm_motor1 = 1500;
  uint32_t pwm_motor2 = 1500;

  // PWM 출력 시작
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  for(;;)
  {
    if (is_emergency == 1) { osDelay(10); continue; }

    /* ── 타겟/링크 상태 판정 ────────────────────────────────────────
     * 서로 다른 두 가지 실패 모드를 구분한다.
     *   (1) 타겟 소실 : 링크는 정상이나 프레임에 사람이 없음  (D0 수신)
     *   (2) 링크 두절 : 파이 다운·케이블 탈락 등으로 패킷 자체가 끊김
     *
     * 두 경우 모두 마지막 오차가 그대로 남아 x_current에 계속 누적되면
     * 카메라가 클램프 한계까지 밀려가는 드리프트가 발생한다.
     * 따라서 오차를 0으로 만들고 미분기까지 리셋해 제어 출력을 0으로 만들어
     * 현재 각도를 그대로 유지시킨다.
     *
     * ※ HAL_GetTick() - rpi_last_rx_tick 은 부호 없는 뺄셈이므로
     *   49.7일 후 tick 오버플로우가 나도 차이값은 정상 동작한다.
     */
    if ((HAL_GetTick() - rpi_last_rx_tick) > RPI_LINK_TIMEOUT_MS)
    {
      tracking = 0;                  // (2) 링크 두절
    }
    else
    {
      tracking = rpi_target_valid;   // (1) 타겟 검출 여부에 따름
    }

    if (tracking)
    {
      x_error = -(double)rpi_err_x;  // 팬 축은 부호 반전
      y_error =  (double)rpi_err_y;
    }
    else
    {
      // 오차 0 + 미분기 리셋 → 제어 출력 0 → 현재 각도에서 정지
      x_error = 0.0;  x_prev_error = 0.0;
      y_error = 0.0;  y_prev_error = 0.0;
    }

    /* X축(팬) 제어.
     * 제어 출력을 각도에 누적(x_current +=)하는 속도형(velocity form) 구조이므로
     * 거동은 적분기에 가깝다. 오차가 0이 되면 더 이상 움직이지 않는다. */
    x_derivative = x_error - x_prev_error;
    x_control = (Kp_x * x_error) + (Kd_x * x_derivative);
    x_current += x_control;
    x_prev_error = x_error;

    /* Y축(틸트) 제어. X축과 동일한 구조, 게인만 다르다. */
    y_derivative = y_error - y_prev_error;
    y_control = (Kp_y * y_error) + (Kd_y * y_derivative);
    y_current += y_control;
    y_prev_error = y_error;

    // 기구적 가동 범위 제한 (5 ~ 175도)
    if (x_current < 5.0)   x_current = 5.0;
    if (x_current > 175.0) x_current = 175.0;
    if (y_current < 5.0)   y_current = 5.0;
    if (y_current > 175.0) y_current = 175.0;

    /* 각도 -> 펄스폭(us) 변환. 기울기 = (2500 - 500) / 180 = 11.111
     * 타이머가 1us/틱으로 설정되어 있어 us 값을 그대로 CCR에 넣을 수 있다. */
    pwm_motor1 = (uint32_t)((x_current * 11.111) + 500.0);
    pwm_motor2 = (uint32_t)((y_current * 11.111) + 500.0);

    // 매크로 실체는 TIM3->CCR1 / CCR2 직접 대입
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_motor1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_motor2);

    // 20ms = 50Hz. 미분항이 일정한 샘플링 주기를 전제하므로 고정한다.
    osDelay(20);
  }
}
/* USER CODE END Header_StartMotorTask */

/* USER CODE BEGIN Header_StartEmergencyTask */
/**
* @brief 비상정지 태스크. 세마포어 대기 중에는 Blocked 상태로 CPU를 점유하지 않는다.
*/
void StartEmergencyTask(void *argument)
{
  for(;;)
  {
    if (osSemaphoreAcquire(myEmergencySemHandle, osWaitForever) == osOK)
    {
      is_emergency = 1;

      // 타이머 출력을 정지시켜 펄스 자체를 차단
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

      // 비상정지 상태 표시 (LED off)
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
  }
}
/* USER CODE END Header_StartEmergencyTask */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */