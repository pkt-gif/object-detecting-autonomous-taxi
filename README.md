# 🚖 Autonomous Driving Taxi & Edge AI System

On-Device AI Vision(Raspberry Pi)과 RTOS 기반 임베디드 제어 시스템(STM32)을 결합한 실시간 자율주행 택시 플랫폼입니다. 도로 환경(스쿨존, 횡단보도, 신호등)을 실시간으로 인식하여 차량의 주행 상태를 판단하고, P-Control 기반의 조향 제어와 오도메트리(Odometry)를 통한 택시 과금 비즈니스 로직을 완벽하게 통합 수행합니다.

---

## 🛠 Tech Stack
* **Edge AI**: Python, NCNN, YOLO11n, OpenCV
* **Firmware (MCU)**: C, STM32F446RE, FreeRTOS, HAL Library
* **Hardware**: L298N (Motor Driver), HC-SR04 (Ultrasonic), Photo Interrupter (Encoder), I2C LCD
* **Communication**: CAN 2.0 (SocketCAN), I2C, PWM

---

## 🚀 System Architecture

### 1. Edge AI Vision (Raspberry Pi 4)
* **Model Optimization**: 최신 객체 인식 모델인 YOLO11n(320x320 Input)을 NCNN 프레임워크로 변환하여 엣지 환경에서의 실시간 추론(Inference) FPS를 극대화했습니다.
* **Finite State Machine (FSM)**: 6개의 클래스(스쿨존 진입/해제, 횡단보도, 3색 신호등) 인식 결과를 바탕으로 자율주행 상태 기계(STOP, SLOW, GO)를 설계 및 운용합니다.
* **In-Vehicle Network**: 판단된 주행 상태(State)는 SocketCAN 인터페이스를 통해 STM32로 0.05초 주기로 실시간 전송됩니다.

### 2. Embedded Control System (STM32F446RE)
* **RTOS Task Scheduling**: FreeRTOS 기반 멀티스레딩 아키텍처를 적용하여 센서 인지(High Priority), 주행 제어(Normal Priority), 요금 UI 갱신(BelowNormal Priority)을 분리하여 자원 점유 최적화 및 실시간성을 확보했습니다.
* **Proportional Control (비례 제어)**: 3채널 초음파 센서의 양측면 데이터를 P-제어 알고리즘으로 연산하여 차동 구동(Differential Drive) 모터의 좌우 PWM Duty Cycle을 동적으로 분배, 팽팽한 트랙 중앙 유지(Center Keeping)를 구현했습니다.
* **Odometry & UI**: 휠 엔코더의 펄스를 외부 인터럽트(EXTI)로 획득하여 물리적 주행 거리(1 Tick = 1.021cm)를 추산하고, 비동기적으로 I2C LCD에 요금을 갱신합니다.

---

## 📁 Directory Structure
```text
.
├── AI_code/                             # 라즈베리파이 Edge AI 시스템
│   ├── bridge_code.py                   # 카메라 스레딩, NCNN 추론 및 CAN 전송 메인 로직
│   └── best_ncnn_model/                 # 최적화된 YOLO11n 가중치 및 파라미터 파일
│
├── STM32/                               # 마이크로컨트롤러 펌웨어
│   ├── Core/Src/
│   │   ├── main.c                       # 타이머 폴링, 인터럽트 디바운싱, CAN 수신 핸들러
│   │   ├── freertos.c                   # 스레드 스케줄링 및 자율주행 P-Control 로직
│   │   └── taxi.c                       # 오도메트리 기반 주행 요금 산출 로직
│   └── Core/Inc/                        # 하드웨어 제어 API 명세 헤더 파일 모음
│
└── README.md
