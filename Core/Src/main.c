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
#include <math.h> // 모터를 sin파로 제어하기 위해 수학 라이브러리 추가
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

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
uint8_t is_emergency = 0; // 비상 정지 플래그 (0: 정상 구동, 1: 비상정지 발동)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // [초기 가동 테스트] 커널 가동 전에 PA5 보드 초록 LED를 무조건 점등시킵니다.
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Create the semaphore(s) */
  myEmergencySemHandle = osSemaphoreNew(1, 0, &myEmergencySem_attributes);

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // CubeMX가 코드를 재생성해도 함수가 삭제되지 않도록 슬롯 안에 정확히 배치
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

  // 정밀 외부 크리스탈(HSE Bypass) 기반 84MHz 메인 심장 세팅
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
  
  // ⚙️ 채널 1 세팅 (PA6 핀 매핑)
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  
  // ⚙️ 채널 2 세팅 (PA7 핀 매핑)
  sConfigOC.Pulse = 1500;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief USART2 Initialization Function
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /*Configure GPIO pin : PC13 (사용자 푸시 파란 버튼) */
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

  /* EXTI 인터럽트 인터페이스 활성화 */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13) 
  {
    // 파란 버튼 클릭 시 비상정지 세마포어 전송 (락 해제)
    osSemaphoreRelease(myEmergencySemHandle);
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
* @brief 2개의 서보모터를 각자 sin파로 제어 및 개별 오차값 보정 연산
*/
void StartMotorTask(void *argument)
{
  double degree = 0.0;     // 사람이 읽기 쉬운 각도 변수 (0 ~ 360도)
  double radian = 0.0;     // sin() 함수 주입용 호도법 라디안 변수
  
  double calc_motor1 = 0.0;
  double calc_motor2 = 0.0;

  uint32_t pwm_motor1 = 2000;
  uint32_t pwm_motor2 = 2000;
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  for(;;)
  {
    if (is_emergency == 1)
    {
      osDelay(10);
      continue; 
    }

    // 각도를 라디안 단위로 완벽하게 변환 (sin 함수 필수 요건)
    radian = degree * (3.141592653589793 / 180.0);
    
    // 1. 실수(double) 상태에서 음수 펄스 계산을 안전하게 먼저 수행합니다.
    calc_motor1 = 1500.0 + (1000.0 * sin(radian));
    calc_motor2 = 1500.0 + (1000.0 * -sin(radian));
    
    // 2. 최종 계산된 '양수 결과값'만 안전하게 정수형으로 변환하여 대입 (0~180도 전구간 복구)
    pwm_motor1 = (uint32_t)calc_motor1;
    pwm_motor2 = (uint32_t)calc_motor2;
    
    // 3. 하드웨어 레벨에서 두 개의 채널 값을 나란히 연속 갱신 (동시 구동 실현)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_motor1); 
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm_motor2); 

    // 각도 전진
    degree += 2.0; 
    if(degree > 360.0) degree = 0.0;
    
    osDelay(20); 
  }
}
/* USER CODE END Header_StartMotorTask */

/* USER CODE BEGIN Header_StartEmergencyTask */
/**
* @brief 비상 버튼 입력 시 무조건 최우선 순위로 모터를 즉시 긴급 정지시키는 태스크
*/
void StartEmergencyTask(void *argument)
{
  for(;;)
  {
    // 세마포어 무전이 오기 전까지는 완벽히 대기 상태를 유지
    if (osSemaphoreAcquire(myEmergencySemHandle, osWaitForever) == osOK)
    {
      is_emergency = 1; // 메인 루트 구동 셧다운 플래그 활성화
      
      // 모터 1번, 2번 출력으로 가는 펄스를 하드웨어 레벨에서 즉각 차단!
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
      
      // 사용자에게 비상 상태임을 경고하기 위해 보드의 LED 불을 꺼버림 (상태 변화 반전)
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