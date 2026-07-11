/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   인터럽트 서비스 루틴(ISR) 및 Cortex-M4 시스템 예외 처리기
  * @details 
  * - 외부 인터럽트(EXTI): 휠 엔코더(포토 인터럽터)의 Edge 감지 (Odometry)
  * - 통신 인터럽트(CAN1_RX0): Edge AI(라즈베리파이)의 제어 명령 실시간 수신
  * - 타이머 인터럽트(TIM5): FreeRTOS 환경에서의 HAL Timebase 제공
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim5;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief Non-Maskable Interrupt (NMI) 처리기
  */
void NMI_Handler(void)
{
   while (1)
  {
  }
}

/**
  * @brief HardFault 예외 처리기
  * @note  메모리 침범, 잘못된 포인터 참조, 스택 오버플로우 등 
  * 치명적인 시스템 오류 발생 시 진입하는 보호 루틴. (디버깅 시 Call Stack 추적용)
  */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief Memory Management Fault 처리기
  */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief Bus Fault 처리기
  */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief Usage Fault 처리기
  */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/**
  * @brief Debug Monitor 핸들러
  */
void DebugMon_Handler(void)
{
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief  EXTI Line 4 인터럽트 처리 (PC4: 좌측 바퀴 엔코더)
  * @note   좌측 포토 인터럽터의 Rising Edge 발생 시 하드웨어적으로 즉각 진입하여
  * main.c의 HAL_GPIO_EXTI_Callback()으로 실행 흐름을 넘김.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief  EXTI Line [9:5] 인터럽트 처리 (PC5: 우측 바퀴 엔코더)
  * @note   우측 포토 인터럽터의 상태 변화를 감지하여 주행 거리(Odometry) 갱신 트리거.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief  CAN1 RX0(수신 FIFO 0) 인터럽트 처리
  * @note   라즈베리파이에서 NCNN 비전 추론 후 전송하는 상태 프레임(0x100)이 
  * 도착하는 즉시 폴링(Polling) 대기 없이 비동기적으로 데이터를 파싱함.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief  TIM5 글로벌 인터럽트 처리 (HAL Timebase)
  * @note   [중요] FreeRTOS가 컨텍스트 스위칭 및 스케줄링을 위해 ARM 코어의 
  * SysTick 타이머를 전용(Exclusive)으로 사용하므로, STM32 HAL 라이브러리의 
  * 내부 딜레이(HAL_Delay) 및 타임아웃 계산을 위해 TIM5를 1ms 주기의 대체 Timebase로 사용함.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}
