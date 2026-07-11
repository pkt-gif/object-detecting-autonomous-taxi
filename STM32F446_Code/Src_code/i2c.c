/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   I2C(Inter-Integrated Circuit) 페리퍼럴 초기화 및 하드웨어 설정
  * @details 외부 I2C LCD 모듈(PCF8574)과의 안정적인 통신을 위해 
  * Open-Drain 구조의 핀 설정 및 100kHz Standard-Mode 클럭을 구성합니다.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "i2c.h"

I2C_HandleTypeDef hi2c1;

/**
  * @brief I2C1 초기화 및 통신 파라미터 설정
  * @note  점퍼선 연결로 인한 노이즈 및 기생 정전용량(Capacitance)을 고려하여 
  * Fast-Mode(400kHz) 대신 안정적인 Standard-Mode(100kHz)로 통신 속도를 제한함.
  */
void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;                     // 100kHz 클럭 스피드 (Standard)
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;// 표준 7-bit 주소 체계 사용
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;   // Clock Stretching 허용 (슬레이브 지연 방어)
  
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 하드웨어 자원(GPIO, 클럭) 초기화
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /** I2C1 GPIO Configuration
    PB8 ------> I2C1_SCL (클럭)
    PB9 ------> I2C1_SDA (데이터)
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    
    // I2C 버스의 전기적 특성인 Open-Drain(OD) 모드 적용
    // 통신 라인의 High 상태는 외부 풀업(Pull-up) 저항에 의해 결정됨
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{
  if(i2cHandle->Instance==I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
  }
}
