# 구현 상세

[README](../README.md)가 **"왜 그렇게 설계했는가"** 를 다룬다면, 이 문서는 **"어떻게 동작하는가"** 를 다룹니다.
레지스터와 계산식 수준까지 내려가 각 기능의 구현을 정리했습니다.

- [1. 초기화 순서](#1-초기화-순서)
- [2. 클럭 트리](#2-클럭-트리)
- [3. UART 설정](#3-uart-설정)
- [4. EXTI 설정](#4-exti-설정)
- [5. 인터럽트와 태스크](#5-인터럽트와-태스크)
- [6. PWM](#6-pwm)
- [7. 제어 루프](#7-제어-루프)
- [8. 라즈베리파이 파이프라인](#8-라즈베리파이-파이프라인)

---

## 1. 초기화 순서

[`Core/Src/main.c`](../Core/Src/main.c)의 `main()`은 초기화만 수행하고, 실제 동작은 전부 태스크에서 일어납니다.

```
HAL_Init()                     // NVIC 우선순위 그룹 설정, TIM1을 1ms 타임베이스로 기동
SystemClock_Config()           // HSE 8MHz -> PLL -> SYSCLK 84MHz
MX_GPIO_Init()                 // PA5(LED) 출력, PC13(버튼) EXTI
MX_TIM3_Init()                 // 50Hz PWM 2채널
MX_USART1_UART_Init()          // 115200 8N1
HAL_UART_Receive_IT(.., 1)     // 수신 인터럽트 최초 무장
osKernelInitialize()
osSemaphoreNew(1, 0, ..)       // 바이너리 세마포어 (최대 1, 초기값 0)
osThreadNew() x 3
osKernelStart()                // 스케줄러 기동 - 이후 main()으로 돌아오지 않음
```

`HAL_UART_Receive_IT()`를 커널 시작 **전에** 한 번 호출해 두는 것이 핵심입니다.
이후로는 수신 콜백이 스스로를 재무장하며 수신이 무한히 이어집니다.

### 태스크 구성

| 태스크 | 우선순위 | 스택 | 역할 |
|---|---|---|---|
| `defaultTask` | `osPriorityNormal` | 512 B | 미사용 (CubeMX 기본 생성) |
| `vMotorTask` | `osPriorityNormal` | 512 B | 50Hz 서보 제어 루프 |
| `vEmergencyTask` | `osPriorityHigh` | 512 B | 비상정지 처리 |

FreeRTOS는 `heap_4` 방식을 사용하며, 태스크 스택·TCB·세마포어가 모두
`configTOTAL_HEAP_SIZE`(15,360 B)에서 할당됩니다.

---

## 2. 클럭 트리

[`main.c` `SystemClock_Config()`](../Core/Src/main.c)

```
HSE = 8MHz  (Nucleo는 ST-Link가 공급하는 8MHz를 받으므로 RCC_HSE_BYPASS)
  |
  +-- PLLM = 4    ->  8MHz / 4   = 2MHz      (VCO 입력, 1~2MHz 권장)
  +-- PLLN = 84   ->  2MHz x 84  = 168MHz    (VCO 출력)
  +-- PLLP = 2    ->  168MHz / 2 = 84MHz     <- SYSCLK

AHB  Prescaler = 1   ->  HCLK  = 84MHz
APB1 Prescaler = 2   ->  PCLK1 = 42MHz       (APB1 최대 42MHz 제약)
APB2 Prescaler = 1   ->  PCLK2 = 84MHz
```

`FLASH_LATENCY_2` — 84MHz에서는 플래시 액세스가 CPU를 따라가지 못하므로
대기 사이클 2를 설정해야 합니다.

### 타이머 클럭 2배 규칙

TIM3는 APB1에 연결되어 있어 42MHz로 보이지만, 실제 타이머 클럭은 **84MHz**입니다.

> APB 프리스케일러가 1이 아니면, 해당 버스의 타이머 클럭은 PCLK의 2배가 된다.

APB1 프리스케일러가 2이므로 TIM3 클럭 = 42MHz × 2 = 84MHz.
이 규칙을 놓치면 PWM 주기가 정확히 절반으로 틀어집니다.

USART1은 APB2에 있으므로 84MHz를 그대로 사용합니다.

---

## 3. UART 설정

설정이 세 계층으로 나뉘어 있습니다.

| 계층 | 위치 | 내용 |
|---|---|---|
| 통신 파라미터 | `main.c` `MX_USART1_UART_Init()` | 보레이트, 8N1, 오버샘플링 |
| 하드웨어 배선 | `stm32f4xx_hal_msp.c` `HAL_UART_MspInit()` | 페리페럴 클럭, GPIO AF, NVIC |
| 인터럽트 진입점 | `stm32f4xx_it.c` `USART1_IRQHandler()` | 벡터 테이블이 호출하는 함수 |

```c
__HAL_RCC_USART1_CLK_ENABLE();      // 페리페럴 클럭 인가 (없으면 레지스터 쓰기가 무시됨)
__HAL_RCC_GPIOA_CLK_ENABLE();

GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;    // Alternate Function, Push-Pull
GPIO_InitStruct.Alternate = GPIO_AF7_USART1;    // AF7 = USART1
GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;

HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(USART1_IRQn);
```

### GPIO 대체 기능(AF)

STM32의 핀 하나는 여러 페리페럴이 공유합니다. PA9는 GPIO로도, USART1_TX로도,
TIM1_CH2로도 쓸 수 있습니다. `AFR` 레지스터에 **AF7**을 기록해야 USART1에 연결됩니다.
AF 번호는 데이터시트의 Alternate function mapping 표에서 확인합니다.

### 보레이트가 레지스터로 변환되는 과정

`huart1.Init.BaudRate = 115200` 한 줄은 실제로 `BRR` 레지스터 값 계산입니다.
(OVER8 = 0, 16배 오버샘플링)

```
USARTDIV = fCK / (16 x Baud)
         = 84,000,000 / (16 x 115,200)
         = 45.5729...

Mantissa = 45                      -> BRR[15:4]
Fraction = 0.5729 x 16 = 9.17 -> 9 -> BRR[3:0]

BRR = (45 << 4) | 9 = 729 = 0x2D9

실제 보레이트 = 84,000,000 / 729 = 115,226 bps
오차 = 0.023%   (UART 허용 오차 약 ±2~3% 대비 충분한 여유)
```

---

## 4. EXTI 설정

```c
GPIO_InitStruct.Pin  = GPIO_PIN_13;
GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;   // 입력이 아니라 인터럽트 모드
GPIO_InitStruct.Pull = GPIO_NOPULL;           // Nucleo B1은 외부 풀업이 이미 존재
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
```

`GPIO_MODE_IT_RISING` 한 줄이 레지스터 수준에서 수행하는 일:

| 순서 | 레지스터 | 동작 |
|---|---|---|
| 1 | `SYSCFG->EXTICR[3]` | EXTI13 라인의 소스를 포트 C로 선택 |
| 2 | `EXTI->IMR` bit 13 | 인터럽트 마스크 해제 |
| 3 | `EXTI->RTSR` bit 13 | 상승 엣지 트리거 설정 |
| 4 | NVIC | `EXTI15_10_IRQn` 활성화 |

**EXTI 라인은 핀 번호 단위로 묶입니다.** 13번 라인은 PA13·PB13·PC13 중
하나만 연결할 수 있고, 10~15번 라인은 NVIC 벡터 하나(`EXTI15_10_IRQn`)를
공유합니다. 그래서 핸들러 안에서 어느 핀인지 다시 판별합니다.

---

## 5. 인터럽트와 태스크

### 인터럽트가 사용자 콜백까지 도달하는 경로

두 인터럽트 모두 동일한 3단 구조입니다.

```
[하드웨어]       [벡터테이블 / it.c]           [HAL 내부]                     [사용자 콜백]

PC13 엣지   ->  EXTI15_10_IRQHandler()   ->  HAL_GPIO_EXTI_IRQHandler()  ->  HAL_GPIO_EXTI_Callback()
                                             (EXTI->PR 플래그 클리어)         main.c

USART1 수신 ->  USART1_IRQHandler()      ->  HAL_UART_IRQHandler()       ->  HAL_UART_RxCpltCallback()
                                             (SR/DR 처리, 상태머신)           main.c
```

`stm32f4xx_it.c`의 핸들러는 CubeMX가 생성한 껍데기이고, HAL이 플래그 처리를 대신한 뒤
`__weak`로 선언된 콜백을 호출합니다. 사용자는 그 콜백을 재정의해 자신의 코드를 넣습니다.

### 비상정지: ISR은 세마포어만 반환

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
    osSemaphoreRelease(myEmergencySemHandle);   // ISR은 여기서 종료
}
```

```c
void StartEmergencyTask(void *argument)
{
  for(;;)
  {
    if (osSemaphoreAcquire(myEmergencySemHandle, osWaitForever) == osOK)
    {
      is_emergency = 1;
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
  }
}
```

이 패턴을 **deferred interrupt processing**(지연 인터럽트 처리)이라고 합니다.

- 태스크는 `osWaitForever`로 **Blocked** 상태 — CPU를 전혀 점유하지 않음
- ISR이 세마포어를 반환하면 태스크가 Ready로 전환되고, `osPriorityHigh`이므로 즉시 선점 실행
- 무거운 처리(PWM 정지 등)는 태스크 컨텍스트에서 수행되어 ISR은 짧게 유지됨

### 인터럽트 우선순위 5의 의미

`Core/Inc/FreeRTOSConfig.h`:

```c
#define configPRIO_BITS                              4    // 우선순위 16단계 (0~15)
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
```

USART1과 EXTI15_10은 모두 우선순위 **5**로 설정되어 있습니다.

Cortex-M은 **숫자가 작을수록 높은 우선순위**입니다. FreeRTOS는 임계구역에서
인터럽트를 전부 비활성화하지 않고, `BASEPRI` 레지스터에
`configMAX_SYSCALL_INTERRUPT_PRIORITY`(= 5 << 4 = 0x50)를 설정해
**숫자가 5 이상인 인터럽트만 차단**합니다.

| 우선순위 | 커널 임계구역에 차단되는가 | `*FromISR` API 호출 |
|---|---|---|
| 0 ~ 4 | 차단되지 않음 (임계구역을 무시하고 진입) | **금지** — 커널 자료구조가 손상됨 |
| 5 ~ 15 | 차단됨 | 허용 |

즉 **우선순위 5는 FreeRTOS API를 호출할 수 있는 가장 높은 우선순위**이고,
이 프로젝트의 두 ISR은 정확히 그 경계에 놓여 있습니다.
세마포어를 반환하려면 5 이상이어야 하고, 응답성을 위해서는 가능한 한 숫자가
작아야 하므로 5가 두 조건을 동시에 만족하는 값입니다.

`HAL_Init()`이 `NVIC_PRIORITYGROUP_4`를 설정하므로 구현된 4비트 전부가
선점 우선순위로 쓰이고 서브 우선순위는 0비트입니다. 그래서
`HAL_NVIC_SetPriority()`의 세 번째 인자는 항상 0입니다.

`osSemaphoreRelease()`는 CMSIS-RTOS v2 래퍼로, 내부에서 `IS_IRQ()`로 컨텍스트를
판별해 ISR이면 `xSemaphoreGiveFromISR()`을 호출합니다. 따라서 태스크와 ISR에서
같은 함수를 쓸 수 있습니다.

### 커널 관련 인터럽트 우선순위

| 인터럽트 | 우선순위 |
|---|---|
| USART1, EXTI15_10 | 5 |
| TIM1 (HAL 타임베이스, `TICK_INT_PRIORITY`) | 15 |
| PendSV | 15 |

커널 관련 인터럽트를 최하위로 두어 응용 인터럽트가 항상 먼저 처리되게 합니다.

---

## 6. PWM

[`main.c` `MX_TIM3_Init()`](../Core/Src/main.c)

```
TIM3 클럭 = 84MHz   (2배 규칙 적용)

Prescaler = 83      ->  84MHz / (83 + 1)  = 1MHz       -> 1틱 = 1us
Period    = 19999   ->  (19999 + 1)틱     = 20,000us   -> 20ms = 50Hz
```

`+1`이 붙는 이유는 카운터가 0부터 세기 때문입니다. `ARR = 19999`면 0~19999까지 20,000틱입니다.

### 각도에서 펄스폭으로

```c
pwm_motor1 = (uint32_t)((x_current * 11.111) + 500.0);
```

```
기울기 = (2500 - 500) / 180 = 11.111

0°   -> 500us
90°  -> 1500us   (중립)
180° -> 2500us
```

프리스케일러를 1MHz로 맞춰 **1틱 = 1µs**가 되게 했으므로, 계산된 마이크로초 값을
그대로 CCR에 넣을 수 있습니다. 단위 환산이 필요 없어집니다.

```c
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_motor1);
// 매크로의 실체: TIM3->CCR1 = pwm_motor1;
```

함수 호출 오버헤드 없이 레지스터에 직접 대입하는 매크로입니다.

PWM 출력 핀은 `HAL_TIM_MspPostInit()`에서 **AF2(TIM3)** 로 설정됩니다 (PA6, PA7).

---

## 7. 제어 루프

[`main.c` `StartMotorTask()`](../Core/Src/main.c)

```c
x_derivative = x_error - x_prev_error;
x_control    = (Kp_x * x_error) + (Kd_x * x_derivative);
x_current   += x_control;          // 각도에 누적
x_prev_error = x_error;
```

`x_current +=` 가 구조의 핵심입니다. 제어 출력을 각도에 그대로 대입하지 않고
**누적**하므로, 엄밀히는 위치형 PD가 아니라 **속도형(velocity form)** 제어기입니다.

- 제어 출력 = "이번 주기에 얼마나 더 회전시킬지"
- 오차가 남아 있으면 계속 누적되어 거동이 적분기에 가까움
- 오차가 0이 되면 더 이상 움직이지 않고 그 각도를 유지

### 주기를 20ms로 고정한 이유

```c
osDelay(20);   // 50Hz
```

1. 미분항 `error - prev_error`는 dt가 일정해야 의미가 있습니다 (dt는 게인에 흡수)
2. 서보 자체의 갱신 주기가 50Hz이므로 더 빈번히 갱신해도 효과가 없습니다

### 게인 규모

```
최대 오차 320px x Kp_x(0.0008) = 사이클당 0.256°
0.256° x 50Hz                  = 최대 약 13°/s
```

실측 튜닝을 거친 값이 아니라 진동이 발생하지 않는 선에서 보수적으로 설정한 값입니다.

### 추적 상태 판정

제어 루프 진입 전에 두 가지 실패 모드를 구분합니다.

```c
if ((HAL_GetTick() - rpi_last_rx_tick) > RPI_LINK_TIMEOUT_MS)
  tracking = 0;                  // 링크 두절
else
  tracking = rpi_target_valid;   // 타겟 검출 여부

if (tracking) {
  x_error = -(double)rpi_err_x;  // 팬 축은 부호 반전
  y_error =  (double)rpi_err_y;
} else {
  x_error = 0.0;  x_prev_error = 0.0;   // 미분기까지 리셋
  y_error = 0.0;  y_prev_error = 0.0;
}
```

누적형 구조에서 오차가 남아 있으면 카메라가 클램프 한계까지 밀려가므로,
오차를 0으로 만들어 제어 출력을 0으로 수렴시켜 현재 각도를 유지시킵니다.
`prev_error`까지 리셋하는 것은 오차가 급변할 때 미분항이 튀는 것(derivative kick)을
막기 위함입니다.

`HAL_GetTick() - rpi_last_rx_tick`은 부호 없는 뺄셈이므로 약 49.7일 후
틱 카운터가 오버플로우해도 차이값은 정상적으로 계산됩니다.

---

## 8. 라즈베리파이 파이프라인

[`raspberry_pi/tracker.py`](../raspberry_pi/tracker.py) — 4개 스레드와 Flask 메인 루프.

| 스레드 | 주기 | 역할 |
|---|---|---|
| T1 `capture_thread` | 카메라 속도 | `libcamera-vid` 파이프에서 MJPEG 프레임 분리 |
| T2 `inference_thread` | 가변 | YOLOv4-tiny 추론, 추적 상태 갱신 |
| T3 `uart_tx_thread` | 고정 20Hz | 패킷 송신 (링크 keepalive) |
| T4 `rtsp_thread` | 고정 15Hz | ffmpeg로 H.264 인코딩 후 RTSP 송출 |
| Flask (메인) | 약 30fps | 디버그용 MJPEG over HTTP |

### MJPEG 프레임 분리

MJPEG 스트림에는 프레임 경계가 명시되지 않으므로 JPEG 마커로 직접 잘라냅니다.

| 마커 | 바이트 |
|---|---|
| SOI (Start of Image) | `FF D8` |
| EOI (End of Image) | `FF D9` |

```python
soi = buf.find(b"\xff\xd8")
eoi = buf.find(b"\xff\xd9", soi + 2) if soi != -1 else -1
if soi != -1 and eoi != -1:
    jpg = buf[soi:eoi + 2]
    buf = buf[eoi + 2:]
```

마커를 찾지 못한 채 버퍼가 4MB를 넘으면 스트림이 깨진 것으로 보고 버퍼를 비웁니다.

### 공유 상태

프레임은 **항상 새 객체로 교체**하고 소비자는 in-place 수정하지 않는다는 규칙을 둬서,
락 구간을 참조 교체로만 한정했습니다. 프레임 복사는 락 밖에서 수행됩니다.

### 패킷 송신

```python
packet = "X{}Y{}D{}\n".format(err_x, err_y, 1 if detected else 0)
```

검출 여부와 무관하게 고정 20Hz로 송신합니다. 패킷의 도착 자체가 링크 생존 신호이므로,
송신 주기가 추론 속도에 따라 흔들리면 STM32가 "추론이 느린 상태"와 "파이가 정지한 상태"를
구분할 수 없게 됩니다.
