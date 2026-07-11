/**
  ******************************************************************************
  * @file    RCC_Motor.c
  * @brief   L298N 듀얼 H-Bridge 모터 드라이버 및 차동 구동(Differential Drive) 제어
  * @details TIM3의 PWM(Pulse Width Modulation) 채널을 이용한 속도 제어 및
  * GPIO 논리 상태를 통한 전/후진 방향 제어(Current Direction Control) 구현
  ******************************************************************************
  */
#include "RCC_Motor.h"
#include "tim.h"

extern TIM_HandleTypeDef htim3;

/**
  * @brief  모터 제어 시스템 초기화
  * @note   타이머 기반의 하드웨어 PWM 신호 출력을 시작하고, 
  * 시스템 부팅 시 모터가 오작동하는 것을 방지하기 위해 초기 상태를 정지(Stop)로 강제함.
  */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // 왼쪽 바퀴 PWM (ENA) 활성화
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // 오른쪽 바퀴 PWM (ENB) 활성화

    Motor_Stop();
}

/**
  * @brief  양륜 모터 정지 및 브레이크 제어
  * @note   H-Bridge 회로의 IN1~IN4 양단 전위를 모두 LOW(0V)로 설정하여
  * 모터 코일에 흐르는 전류를 차단하고 관성에 의한 Free-run 상태로 안전 정지.
  */
void Motor_Stop(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT2, GPIO_PIN_RESET);

    Motor_SetSpeed(0, 0); // PWM Duty Cycle 0% 인가
}

/**
  * @brief  양륜 정방향(Forward) 주행
  * @note   IN1/IN3(RESET)과 IN2/IN4(SET) 조합으로 모터에 정방향 전위차를 형성.
  */
void Motor_Forward(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT2, GPIO_PIN_SET);
}

/**
  * @brief  양륜 역방향(Backward) 주행
  * @note   전진 모드와 전류의 흐름을 반대로 인가(SET/RESET 반전)하여 역회전 유도.
  */
void Motor_Backward(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT2, GPIO_PIN_RESET);
}

/**
  * @brief  제자리 좌회전 (Point Turn Left)
  * @note   차동 구동(Differential Drive) 역학 적용:
  * 왼쪽 바퀴는 후진, 오른쪽 바퀴는 전진시켜 차체의 중심축을 기준으로 최소 반경 회전 수행.
  */
void Motor_Left(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT2, GPIO_PIN_SET);
}

/**
  * @brief  제자리 우회전 (Point Turn Right)
  * @note   왼쪽 바퀴는 전진, 오른쪽 바퀴는 후진시켜 우측 방향으로 회전 토크 발생.
  */
void Motor_Right(void)
{
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_LEFT2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_GPIO, MOTOR_RIGHT2, GPIO_PIN_RESET);
}

/**
  * @brief  모터 회전 속도 제어 (PWM Duty Cycle 조절)
  * @param  left_speed  왼쪽 모터 PWM Duty 값 (0 ~ Period 값)
  * @param  right_speed 오른쪽 모터 PWM Duty 값 (0 ~ Period 값)
  * @note   HAL 라이브러리의 오버헤드를 줄이기 위해 매크로 함수를 사용하여
  * 타이머의 CCR(Capture/Compare Register) 값을 하드웨어 레벨에서 직접(Direct) 갱신함.
  * 이를 통해 자율주행 P 제어 루프에서 실시간 조향 지연시간(Latency)을 최소화.
  */
void Motor_SetSpeed(uint16_t left_speed, uint16_t right_speed)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_speed);
}
