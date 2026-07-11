/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   차량 시스템 전체 GPIO(General Purpose Input/Output) 초기화 및 설정
  * @details 모터 제어(L298N), 초음파 센서(HC-SR04), 엔코더(포토 인터럽터) 등
  * 엑추에이터와 센서의 전기적 특성에 맞춘 핀 모드(Push-Pull, EXTI, Input) 구성.
  * FreeRTOS와의 충돌을 방지하기 위한 인터럽트 우선순위(NVIC) 설정 포함.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable (AHB1 버스 클럭 활성화로 전력 소모 최적화) */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* ------------------------------------------------------------------------
   * [초기 상태 설정] 시스템 부팅 시 모터 및 센서 오작동(Floating) 방지
   * ------------------------------------------------------------------------ */
  /* 모터 방향 제어 핀(L298N IN1~IN4) 초기화: LOW(0V) 출력으로 모터 강제 정지 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /* 초음파 센서 트리거(Trigger) 핀 초기화: 초음파 미발사 상태(LOW) 유지 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /* ------------------------------------------------------------------------
   * [모터 드라이버 제어 핀] Push-Pull 출력
   * ------------------------------------------------------------------------ */
  /* Configure GPIO pins : PC0 PC1 PC2 PC3 (모터 방향 제어) */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-Pull 모드: 확실한 High/Low 논리 레벨 보장
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 고속 스위칭이 필요 없으므로 Low 설정 (EMI 노이즈 저감)
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ------------------------------------------------------------------------
   * [휠 엔코더(Odometry) 센서 핀] 외부 인터럽트(EXTI) 설정
   * ------------------------------------------------------------------------ */
  /* Configure GPIO pins : PC4 PC5 (좌/우 포토 인터럽터 센서) */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Rising Edge 감지: 빛이 차단되었다가 다시 들어오는 순간을 1 틱(Tick)으로 인식
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* ------------------------------------------------------------------------
   * [초음파 센서 트리거(Trigger) 핀] MCU -> Sensor (출력)
   * ------------------------------------------------------------------------ */
  /* Configure GPIO pins : PB12 PB14 (좌측, 전방 트리거) */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pin : PA8 (우측 트리거) */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ------------------------------------------------------------------------
   * [초음파 센서 에코(Echo) 핀] Sensor -> MCU (입력)
   * ------------------------------------------------------------------------ */
  /* Configure GPIO pins : PB13 PB15 (좌측, 우측 에코) */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 단순 입력 감지 (타이머 폴링 방식으로 펄스 폭 측정)
  GPIO_InitStruct.Pull = GPIO_NOPULL;     // HC-SR04 내부 회로를 신뢰하여 Pull-up/down 생략
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure GPIO pin : PA9 (전방 에코) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ------------------------------------------------------------------------
   * [외부 인터럽트(NVIC) 설정 및 FreeRTOS 호환성 확보]
   * ------------------------------------------------------------------------ */
  /* EXTI4_IRQn (PC4: 좌측 바퀴 센서) 
   * EXTI9_5_IRQn (PC5: 우측 바퀴 센서)
   * * [핵심] 우선순위를 5로 설정한 이유: 
   * FreeRTOS의 configMAX_SYSCALL_INTERRUPT_PRIORITY가 보통 5로 설정됨.
   * 인터럽트 내부에서 OS API(예: 세마포어, 큐 전달)를 안전하게 호출하기 위해
   * 우선순위를 5(낮은 하드웨어 우선순위, 높은 논리 우선순위)로 제한함.
   */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
