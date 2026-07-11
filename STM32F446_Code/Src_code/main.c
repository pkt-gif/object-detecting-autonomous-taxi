/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : 자율주행 및 요금 처리 시스템 메인 컨트롤러 (Master MCU)
 * @details        : STM32F446RE (180MHz) 기반의 중앙 제어 시스템.
 * - Edge AI (Raspberry Pi)와의 CAN 통신을 통한 상태(State) 수신
 * - TIM4의 1us 분해능을 활용한 다채널 초음파 센서 정밀 거리 측정
 * - EXTI 인터럽트 기반의 휠 엔코더(Odometry) 카운팅 및 소프트웨어 디바운싱
 * - 하드웨어 브링업(Bring-up) 테스트 후 FreeRTOS 커널로 제어권 이관
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "taxi.h"
#include "RCC_Motor.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* * [글로벌 공유 상태 변수]
 * 인터럽트(ISR)와 여러 RTOS 태스크에서 동시 접근하므로, 
 * 컴파일러의 레지스터 최적화를 방지하고 항상 메모리에서 직접 읽도록 volatile 선언.
 */
volatile uint8_t AI_State = 0;   // 자율주행 상태 기계 (0: 정지, 1: 감속, 2: 정속)
volatile uint16_t AI_Speed = 0;  // 동적 속도 제어용 파라미터 (0 ~ 255)

volatile uint16_t distance_Left  = 0;
volatile uint16_t distance_Front = 0;
volatile uint16_t distance_Right = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  초음파 센서(HC-SR04) 정밀 제어 및 거리 산출 (Microsecond Polling)
 * @note   HAL_Delay()는 1ms 분해능이므로, 12us의 짧은 트리거 펄스를 생성하기 위해 
 * TIM4 하드웨어 카운터 레지스터를 직접 조작(Direct Register Access)함.
 * @param  trigPort 트리거 핀 포트
 * @param  trigPin  트리거 핀 번호
 * @param  echoPort 에코 핀 포트
 * @param  echoPin  에코 핀 번호
 * @retval 측정된 거리 (cm 단위, 타임아웃 시 0 반환)
 */
uint16_t HCSR04_Read(GPIO_TypeDef *trigPort, uint16_t trigPin, GPIO_TypeDef *echoPort, uint16_t echoPin)
{
    // 1. Trigger Pulse 생성: 데이터시트 규격(최소 10us)을 만족하는 12us High 신호 인가
    HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while(__HAL_TIM_GET_COUNTER(&htim4) < 2); // 핀 상태 안정화 대기

    HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while(__HAL_TIM_GET_COUNTER(&htim4) < 12); // 180MHz 클럭 환경에서도 정확히 12us 펄스 유지

    HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);

    // 2. Echo 신호 응답 대기 (초음파 발사 대기)
    // 노이즈나 센서 미응답으로 인한 무한 루프(Deadlock)를 방지하기 위해 5ms 타임아웃 설정
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while (HAL_GPIO_ReadPin(echoPort, echoPin) == GPIO_PIN_RESET) {
        if (__HAL_TIM_GET_COUNTER(&htim4) > 5000) return 0;
    }

    // 3. Echo 펄스 폭(Pulse Width) 측정 (초음파가 되돌아오는 시간)
    // 최대 측정 거리 5m 왕복 시간인 약 30ms를 타임아웃으로 설정하여 시스템 지연 방지
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while (HAL_GPIO_ReadPin(echoPort, echoPin) == GPIO_PIN_SET) {
        if (__HAL_TIM_GET_COUNTER(&htim4) > 30000) return 0;
    }

    // 4. Time-of-Flight(ToF) 방식 거리 환산
    // 음속(340m/s)을 기준으로 왕복 거리를 고려한 상수 58을 나누어 cm 단위 산출
    return (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) / 58);
}
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
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	Motor_Init();
	HAL_TIM_Base_Start(&htim4); // 초음파 센서 제어용 1MHz(1us 분해능) 타이머 구동

	/* CAN 통신 필터링(Mask) 설정: 현재는 모든 ID의 메시지를 수신(0x0000)하도록 개방 */
	CAN_FilterTypeDef canFilterConfig;
	canFilterConfig.FilterBank = 0;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilterConfig.FilterIdHigh = 0x0000;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x0000;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilterConfig.FilterActivation = ENABLE;
	canFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig); // 필터 적용
	HAL_CAN_Start(&hcan1);                          // CAN 버스 활성화
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // 데이터 수신 시 외부 인터럽트 트리거 설정

	/* * [Pre-RTOS 하드웨어 검증 루틴 (Hardware Calibration Loop)]
     * FreeRTOS 커널이 구동되기 전, 엑추에이터(L298N 및 모터)의 물리적 결선 상태와
     * 파워 서플라이의 출력 한계를 점검하기 위한 순수 베어메탈(Bare-metal) 테스트 코드.
     * 역기전력 방지(0.5초 딜레이)를 준수하며 4방향 거동을 1회 사이클 검증함.
     * (실제 자율주행 통합 시 해당 while문을 주석 처리하여 RTOS로 제어권 이관)
     */
	while (1)
	  {
	      // 1. 최고속도 전진 (2초)
	      Motor_Forward();
	      Motor_SetSpeed(700, 700);
	      HAL_Delay(2000);

	      // 모터 드라이버(H-Bridge) 보호를 위한 방향 전환 전 데드타임(Dead-time) 0.5초 부여
	      Motor_Stop();
	      HAL_Delay(500);

	      // 2. 최고속도 후진 (2초)
	      Motor_Backward();
	      Motor_SetSpeed(700, 700);
	      HAL_Delay(2000);

	      Motor_Stop();
	      HAL_Delay(500);

	      // 3. 최고속도 제자리 좌회전 (2초)
	      Motor_Left();
	      Motor_SetSpeed(700, 700);
	      HAL_Delay(2000);

	      Motor_Stop();
	      HAL_Delay(500);

	      // 4. 최고속도 제자리 우회전 (2초)
	      Motor_Right();
	      Motor_SetSpeed(700, 700);
	      HAL_Delay(2000);

	      Motor_Stop();
	      HAL_Delay(2000); 
	  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

/**
  * @brief  CAN 통신 수신 인터럽트 콜백 함수 (Event-driven)
  * @note   라즈베리파이(Edge AI)에서 전송한 비전 처리 결과(0x100)를 실시간 파싱함.
  * 인터럽트 루틴이므로 연산을 최소화하고 상태 변수(State) 갱신만 수행하여 오버헤드 방지.
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
		if (RxHeader.StdId == 0x100) // 약속된 파이썬 브릿지 ID 검증
		{
			AI_State = RxData[0]; // 주행 상태 머신 (0, 1, 2)
			AI_Speed = RxData[1]; // 동적 속도 (현재 미사용, 확장성 고려)
		}
	}
}

uint32_t last_left_tick_time = 0;
uint32_t last_right_tick_time = 0;

/**
  * @brief  외부 인터럽트(EXTI) 콜백 - 휠 엔코더(Odometry) 처리
  * @note   포토 인터럽터 센서의 물리적 채터링(Chattering) 및 고속 모터 노이즈를 
  * 방지하기 위한 소프트웨어 디바운싱(Software Debouncing) 알고리즘 적용.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 현재 시스템 시간 (1ms 단위) 획득
    uint32_t current_time = HAL_GetTick();

    // PC4 (좌측 바퀴 센서 엣지 감지)
    if (GPIO_Pin == GPIO_PIN_4)
    {
        // 디바운스 필터링: 마지막 유효 틱으로부터 2ms(블랭킹 타임) 이내의 노이즈 신호 무시
        if (current_time - last_left_tick_time >= 2)
        {
            TaxiMeter_AddLeftTick(); // 주행 거리 누적
            last_left_tick_time = current_time;
        }
    }
    // PC5 (우측 바퀴 센서 엣지 감지)
    else if (GPIO_Pin == GPIO_PIN_5)
    {
        if (current_time - last_right_tick_time >= 2)
        {
            TaxiMeter_AddRightTick();
            last_right_tick_time = current_time;
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
