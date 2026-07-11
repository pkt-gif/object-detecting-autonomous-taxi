/**
  ******************************************************************************
  * @file    can.c
  * @brief   차량 제어용 CAN(Controller Area Network) 통신 드라이버
  * @details 라즈베리파이(Edge AI)에서 판단한 주행 상태(STOP, SLOW, GO)를 
  * 수신하기 위한 CAN1 인터페이스 초기화 및 하드웨어(GPIO, NVIC) 설정
  ******************************************************************************
  */
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/**
  * @brief CAN1 초기화 및 비트 타이밍(Bit Timing) 설정
  * @note  목표 Baudrate: 500 kbps 
  * - APB1 Clock: 45 MHz (STM32F446 기준)
  * - Prescaler: 9  => CAN Clock = 45MHz / 9 = 5 MHz
  * - 1 Bit Time (10 TQ) = Sync(1) + BS1(7) + BS2(2)
  * - 최종 통신 속도 = 5 MHz / 10 TQ = 500 kbps
  */
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  
  // 1. 통신 속도 제어 파라미터 (500kbps 동기화)
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;         // 정상 송수신 모드
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;    // 클럭 오차 보정 허용치
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;         // 샘플링 포인트 설정 (Bit Segment 1)
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;         // 샘플링 포인트 설정 (Bit Segment 2)
  
  // 2. 통신 안정성 및 에러 처리 기능 설정
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;           // Bus-Off 상태 시 자동 복구 비활성화 (소프트웨어적 제어)
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;   // 자동 재전송 비활성화 (실시간성 확보 목적)
  hcan1.Init.ReceiveFifoLocked = DISABLE;    // FIFO 오버런 시 덮어쓰기 허용 (최신 데이터 유지)
  hcan1.Init.TransmitFifoPriority = DISABLE; // ID 기반 송신 우선순위 사용
  
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 하드웨어 자원(MSP: MCU Specific Package) 초기화
  * @param canHandle CAN 핸들러 포인터
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  if(canHandle->Instance==CAN1)
  {
    /* 1. 페리퍼럴 클럭 활성화 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    /** CAN1 GPIO Configuration (Alternate Function)
    * PA11 ------> CAN1_RX (수신)
    * PA12 ------> CAN1_TX (송신)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;           // 푸시-풀 대체 기능 모드
    GPIO_InitStruct.Pull = GPIO_NOPULL;               // CAN 트랜시버에 풀업/풀다운 위임
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;// 고속 통신을 위한 GPIO 응답 속도 최대화
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;        // PA11, PA12를 CAN1으로 연결(AF9)
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 2. 수신 인터럽트(NVIC) 설정
     * 데이터를 놓치지 않고 실시간으로 즉각 반응하기 위해 FIFO0 RX 인터럽트 활성화
     */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0); // FreeRTOS 환경에 맞춘 적절한 우선순위(5) 할당
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}

/**
  * @brief CAN1 하드웨어 자원 해제
  * @param canHandle CAN 핸들러 포인터
  */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{
  if(canHandle->Instance==CAN1)
  {
    /* 클럭 비활성화 */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /* GPIO 핀 설정 해제 */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* 인터럽트 비활성화 */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  }
}
