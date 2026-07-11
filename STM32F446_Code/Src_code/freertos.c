/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file          : freertos.c
 * @brief         : FreeRTOS 기반 자율주행 멀티스레딩(Multi-threading) 시스템
 * @details       : 센서 데이터 수집(High Priority), 주행 제어(Normal Priority), 
 * 요금 UI 갱신(BelowNormal) 태스크를 분리하여 실시간성(Real-time) 확보.
 * 공유 자원(센서 데이터) 접근 시 Mutex를 사용하여 Race Condition 방지.
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "taxi.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RCC_Motor.h"
#include "tim.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 외부 통신(CAN) 및 인터럽트를 통해 갱신되는 글로벌 제어 변수 (최적화 방지를 위해 volatile 선언)
extern volatile uint8_t AI_State;
extern volatile uint16_t AI_Speed;

// 다중 태스크(Sensor, Drive)에서 접근하는 공유 자원 (초음파 거리 데이터)
extern volatile uint16_t distance_Left;
extern volatile uint16_t distance_Front;
extern volatile uint16_t distance_Right;
/* USER CODE END Variables */

/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh, // 환경 인지가 가장 중요하므로 최우선순위(High) 할당
};
/* Definitions for DriveTask */
osThreadId_t DriveTaskHandle;
const osThreadAttr_t DriveTask_attributes = {
  .name = "DriveTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal, // 센서 데이터를 기반으로 동작하므로 Normal 할당
};
/* Definitions for TaxiTask */
osThreadId_t TaxiTaskHandle;
const osThreadAttr_t TaxiTask_attributes = {
  .name = "TaxiTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal, // UI 갱신은 주행 실시간성에 영향을 주지 않도록 낮게 설정
};
/* Definitions for sensorMutex */
osMutexId_t sensorMutexHandle;
const osMutexAttr_t sensorMutex_attributes = {
  .name = "sensorMutex" // 공유 자원 보호를 위한 Mutex 객체
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint16_t HCSR04_Read(GPIO_TypeDef *trigPort, uint16_t trigPin,
        GPIO_TypeDef *echoPort, uint16_t echoPin);
static void RS(uint16_t *l, uint16_t *f, uint16_t *r);
/* USER CODE END FunctionPrototypes */

void StartSensorTask(void *argument);
void StartDriveTask(void *argument);
void StartTaxiTask(void *argument);

void MX_FREERTOS_Init(void); 

void MX_FREERTOS_Init(void) {
  /* creation of sensorMutex */
  sensorMutexHandle = osMutexNew(&sensorMutex_attributes);

  /* creation of Threads */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);
  DriveTaskHandle = osThreadNew(StartDriveTask, NULL, &DriveTask_attributes);
  TaxiTaskHandle = osThreadNew(StartTaxiTask, NULL, &TaxiTask_attributes);
}

/* USER CODE BEGIN Application */
/**
 * @brief  센서 데이터 안전 읽기 함수 (Critical Section 보호)
 * @note   DriveTask가 센서 값을 읽어갈 때, SensorTask가 값을 덮어쓰는 
 * Data Tearing 현상을 방지하기 위해 Mutex 단위로 Lock/Unlock 수행
 */
static void RS(uint16_t *l, uint16_t *f, uint16_t *r)
{
    osMutexAcquire(sensorMutexHandle, osWaitForever);
    *l = distance_Left;
    *f = distance_Front;
    *r = distance_Right;
    osMutexRelease(sensorMutexHandle);
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief  환경 인지 태스크 (SensorTask - Priority: High)
 * @note   3개의 초음파 센서를 지속적으로 Polling하여 전방/좌/우 거리를 갱신함
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
    for (;;)
    {
        // 물리적 음파 간섭(Cross-talk) 방지를 위해 각 센서 트리거 사이에 40ms 딜레이(Context Switching) 부여
        uint16_t l = HCSR04_Read(GPIOB, GPIO_PIN_12, GPIOB, GPIO_PIN_13);
        osDelay(40);
        uint16_t f = HCSR04_Read(GPIOB, GPIO_PIN_14, GPIOA, GPIO_PIN_9);
        osDelay(40);
        uint16_t r = HCSR04_Read(GPIOA, GPIO_PIN_8,  GPIOB, GPIO_PIN_15);
        osDelay(40);

        // 측정 완료 후 Mutex를 획득하여 전역 변수 일괄 업데이트 (원자성 보장)
        osMutexAcquire(sensorMutexHandle, osWaitForever);
        distance_Left  = l;
        distance_Front = f;
        distance_Right = r;
        osMutexRelease(sensorMutexHandle);
    }
}

/* USER CODE BEGIN Header_StartDriveTask */
/**
 * @brief 자율주행 제어 태스크 (DriveTask - Priority: Normal)
 * @note  Edge AI(라즈베리파이)의 CAN 명령 상태(AI_State)와 초음파 데이터를 융합하여 
 * 조향 및 속도를 결정하는 핵심 알고리즘(P-Control 기반) 구동
 */
/* USER CODE END Header_StartDriveTask */
void StartDriveTask(void *argument)
{
    osDelay(100); // 초기화 안정화 대기

    for (;;)
    {
        // 1. 상태 기계(State Machine)에 따른 동적 속도 프로파일 설정
        int base_spd = 450;
        int max_spd = 650;  // 코너링 시 바깥쪽 바퀴에 강한 토크 인가 (차체 관성 극복)
        int min_spd = 200;  // 조향 시 안쪽 바퀴의 감속을 보장하여 선회 반경 최소화

        if (AI_State == 0) // STOP 상태 (예: 횡단보도, 적색불 인식 시)
        {
            Motor_Stop();
            osDelay(10);
            continue;
        }
        else if (AI_State == 1) // SLOW 상태 (예: 스쿨존 인식 시)
        {
            // 베이스 속도를 350으로 낮추되, 조향 여유폭(max-min)은 크게 유지하여 코너링 성능 보장
            base_spd = 350;
            max_spd = 500;  
            min_spd = 200;  
        }

        uint16_t dist_L, dist_F, dist_R;
        RS(&dist_L, &dist_F, &dist_R); // Mutex로 보호된 최신 센서 데이터 Load

        // 센서 미인식(에러값 0) 시 무한대(안전 거리 40cm 이상)로 간주하는 예외 처리
        if (dist_F == 0) dist_F = 40;
        if (dist_L == 0) dist_L = 40;
        if (dist_R == 0) dist_R = 40;

        // ----------------------------------------------------
        // [알고리즘 1] 전방 장애물 감지 및 회전 (Quick Turn)
        // ----------------------------------------------------
        if (dist_F <= 37) // 제동 관성을 고려한 전방 정지 임계값 (37cm)
        {
            Motor_Stop();
            osDelay(50); // 모터 역기전력(Reverse EMF) 방지 대기

            // 좌/우 여유 공간을 비교하여 회전 방향 결정
            uint8_t turn_left = (dist_L > dist_R) ? 0 : 1;
            uint32_t turnStart = osKernelGetTickCount();

            // 최대 1.5초(1500ms) 타임아웃을 가진 Point Turn 수행
            while (osKernelGetTickCount() - turnStart < 1500)
            {
                if (turn_left) {
                    Motor_Left();
                    Motor_SetSpeed(530, 530);
                } else {
                    Motor_Right();
                    Motor_SetSpeed(530, 530);
                }
                osDelay(30);

                RS(&dist_L, &dist_F, &dist_R);

                // 정면 45cm 확보 및 측면 클리어런스(15cm) 만족 시 안전한 코너 탈출로 판단
                if (dist_F >= 45 && dist_L >= 15 && dist_R >= 15) {
                    osDelay(20);
                    break;
                }
            }
            Motor_Stop();
            osDelay(50);
        }
        // ----------------------------------------------------
        // [알고리즘 2] 측면 긴급 회피 (Emergency Avoidance)
        // ----------------------------------------------------
        // 측면 10cm 이내 접근 시 차체 무게를 고려하여 극단적인 비대칭 토크(550:150) 인가
        else if (dist_L <= 10)
        {
            Motor_Forward();
            Motor_SetSpeed(550, 150);
        }
        else if (dist_R <= 10)
        {
            Motor_Forward();
            Motor_SetSpeed(150, 550);
        }
        // ----------------------------------------------------
        // [알고리즘 3] 비례 제어(Proportional Control) 기반 트랙 중앙 유지 (Center Keeping)
        // ----------------------------------------------------
        else
        {
            Motor_Forward();

            // 좌우 거리 편차(Error) 연산
            int diff = (int)dist_L - (int)dist_R;

            // 조향 헌팅(Ping-pong) 현상 방지를 위한 1cm 데드존(Deadzone) 설정
            if (diff > -1 && diff < 1) {
                diff = 0; 
            }
            // 적분 누적(Windup) 방지 및 최대 오차 허용치 클램핑(Clamping)
            if (diff > 35) diff = 35;
            if (diff < -35) diff = -35;

            // P-Gain 설정: 하드웨어 마찰력과 반응성을 고려한 최적 튜닝값(5)
            int p_gain = 5;
            int offset = diff * p_gain;

            // 오차에 따른 보상 속도 분배 (Differential Speed)
            int left_spd = base_spd - offset;
            int right_spd = base_spd + offset;

            // PWM Duty 인가 전 속도 상/하한선 제한 (Saturation 방지)
            if (left_spd > max_spd) left_spd = max_spd;
            if (left_spd < min_spd) left_spd = min_spd;
            if (right_spd > max_spd) right_spd = max_spd;
            if (right_spd < min_spd) right_spd = min_spd;

            Motor_SetSpeed((uint16_t)left_spd, (uint16_t)right_spd);
        }

        osDelay(10); // 태스크 Yield (다른 우선순위 태스크에 CPU 점유율 양보)
    }
}

/* USER CODE BEGIN Header_StartTaxiTask */
/**
* @brief  UI 및 요금 결제 태스크 (TaxiTask - Priority: BelowNormal)
* @note   엔코더 센서 기반의 요금 갱신 및 I2C 통신 LCD 출력 전담.
* I2C 통신의 딜레이가 조향(DriveTask)에 영향을 미치지 않도록 낮은 우선순위 할당
*/
/* USER CODE END Header_StartTaxiTask */
void StartTaxiTask(void *argument)
{
  /* USER CODE BEGIN StartTaxiTask */
    TaxiMeter_Run(); // 내부 무한 루프로 동작하여 요금 실시간 갱신
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaxiTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
