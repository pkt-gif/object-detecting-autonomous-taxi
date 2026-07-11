/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   하드웨어 타이머 인스턴스 초기화 및 레지스터 설정
  * @details 
  * 1) TIM3: 1kHz 주파수의 PWM 신호 생성 (L298N 모터 드라이버 ENA/ENB 제어)
  * 2) TIM4: 1us(마이크로초) 분해능의 Free-running 카운터 생성 (초음파 센서 측정)
  ******************************************************************************
  */
/* USER CODE END Header */
#include "tim.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/**
  * @brief TIM3 초기화 (모터 구동용 하드웨어 PWM)
  * @note  APB1 Timer Clock = 90MHz 기준 설정
  * - Prescaler(PSC) = 90 - 1  => 90MHz / 90 = 1MHz 카운터 클럭
  * - Period(ARR)    = 1000 - 1 => 1MHz / 1000 = 1kHz PWM 주파수
  * DC 모터 구동에 최적화된 1kHz 주파수를 도출하여 고주파 노이즈(고주음)를 방지함.
  */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 90-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1; // Duty Cycle을 0~999 해상도로 세밀하게 제어 가능
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
  
  // PWM 모드 1 설정: CNT < CCR 일 때 High 레벨 출력
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0; // 초기 Duty Cycle 0% (안전 정지)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  // 채널 1 (왼쪽 바퀴 ENA), 채널 2 (오른쪽 바퀴 ENB) 활성화
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief TIM4 초기화 (초음파 센서용 1us 정밀 타이머)
  * @note  APB1 Timer Clock = 90MHz 기준 설정
  * - Prescaler(PSC) = 90 - 1 => 1MHz 카운터 클럭 (1 Tick = 1us)
  * - Period(ARR)    = 65535  => 16-bit 최대값 설정. 
  * 카운터 오버플로우까지 65.5ms가 걸리며, 이는 초음파 센서 최대 응답 시간(약 30ms)을 
  * 안전하게 커버할 수 있는 측정 구간을 보장함.
  */
void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 90-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /** TIM3 GPIO Configuration (PWM Output)
    PA6 ------> TIM3_CH1 (모터 A 속도 제어)
    PA7 ------> TIM3_CH2 (모터 B 속도 제어)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    
    // PWM 신호 출력을 위해 Alternate Function(대체 기능) 모드 적용
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }
  else if(tim_baseHandle->Instance==TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
  }
}
