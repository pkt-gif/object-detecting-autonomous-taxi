/**
  ******************************************************************************
  * @file    taxi.c
  * @brief   자율주행 택시 요금 미터기 및 Odometry(주행 기록) UI 시스템
  * @details 좌/우 휠 엔코더(포토 인터럽터)의 Tick 데이터를 기반으로 
  * 양륜 평균 이동 거리를 산출하고, 실시간 요금을 I2C LCD에 비동기적으로 출력함.
  ******************************************************************************
  */
#include "taxi.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "I2C_LCD.h" // I2C 기반 HD44780 LCD 제어 헤더

/* 글로벌 공유 변수: 외부 인터럽트(EXTI) 루틴에서 갱신되므로 volatile 선언 */
volatile uint32_t left_ticks = 0;
volatile uint32_t right_ticks = 0;

/**
  * @brief 미터기 및 LCD 초기화
  */
void TaxiMeter_Init(void)
{
    i2c_lcd_init(); // LCD 하드웨어 초기화 시퀀스 실행
    left_ticks = 0;
    right_ticks = 0;
}

/**
  * @brief  좌측 휠 엔코더 Tick 증가 (EXTI ISR에서 호출)
  * @note   인터럽트 서비스 루틴 내부에서 실행되므로 연산을 최소화함.
  */
void TaxiMeter_AddLeftTick(void)
{
    left_ticks++;
}

/**
  * @brief  우측 휠 엔코더 Tick 증가 (EXTI ISR에서 호출)
  */
void TaxiMeter_AddRightTick(void)
{
    right_ticks++;
}

/**
  * @brief  요금 계산 및 UI 갱신 태스크 (FreeRTOS Task)
  * @note   자율주행 조향(DriveTask)과 분리된 독립 스레드로 동작.
  * I2C 통신의 Blocking 현상이 차량 제어에 영향을 주지 않도록 설계됨.
  */
void TaxiMeter_Run(void)
{
    TaxiMeter_Init();

    float total_distance_cm = 0.0f;
    int current_fare = BASE_FARE;
    
    // 버퍼 오버플로우 방지를 위해 충분한 크기의 로컬 버퍼 할당 (16 Characters + Null)
    char lcd_buffer_row1[24]; 
    char lcd_buffer_row2[24];

    for(;;)
    {
        // 1. Odometry (추측 항법) 연산: 차동 구동 로봇의 회전 오차 상쇄
        // 좌/우 바퀴의 이동 거리가 코너링 시 달라지는 점을 고려하여 양륜 평균 틱(Tick) 적용
        float avg_ticks = (left_ticks + right_ticks) / 2.0f;
        total_distance_cm = avg_ticks * DIST_PER_TICK_CM;

        // 2. 요금(Fare) 산출 로직
        // 정수형 캐스팅(int)을 통해 소수점 단위 거리를 버림 처리하여 요금 인상 구간 계산
        int additional_units = (int)(total_distance_cm / DISTANCE_UNIT_CM);
        current_fare = BASE_FARE + (additional_units * FARE_PER_UNIT);

        // 3. UI 문자열 포매팅 (Defensive Programming 적용)
        // sprintf 대신 snprintf를 사용하여 메모리 침범(Buffer Overflow) 원천 차단
        snprintf(lcd_buffer_row1, sizeof(lcd_buffer_row1), "Dist: %5.2f m   ", total_distance_cm / 100.0f);
        snprintf(lcd_buffer_row2, sizeof(lcd_buffer_row2), "Fare: %5d W   ", current_fare);

        /* * [디버깅 흔적] 하드웨어 캘리브레이션 시 센서 불량/노이즈를 
         * 확인하기 위해 좌우 Tick을 개별 모니터링했던 테스트 코드
         */
//        snprintf(lcd_buffer_row1, sizeof(lcd_buffer_row1), "L: %lu   ", left_ticks);
//        snprintf(lcd_buffer_row2, sizeof(lcd_buffer_row2), "R: %lu   ", right_ticks);

        // 4. I2C 통신을 통한 LCD 디스플레이 출력
        move_cusor(0, 0); // DDRAM 1행 1열 주소 세팅
        lcd_string(lcd_buffer_row1);

        move_cusor(1, 0); // DDRAM 2행 1열 주소 세팅
        lcd_string(lcd_buffer_row2);

        // 5. Task Scheduling & I2C Bus Optimization
        // 화면 주사율(Refresh Rate)을 5Hz(200ms)로 제한.
        // 너무 잦은 I2C 통신은 버스 트래픽 과부하 및 LCD 컨트롤러(HD44780) 먹통 현상을 유발함.
        osDelay(200);
    }
}
