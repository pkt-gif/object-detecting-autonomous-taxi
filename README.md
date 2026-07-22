# Object-Detecting Autonomous Taxi

> **YOLO11n NCNN과 STM32F446RE를 활용한 객체 탐지 기반 자율주행 RC 택시**

Raspberry Pi 4의 USB 웹캠으로 어린이보호구역 표지판, 횡단보도, 신호등을 인식하고, 판단 결과를 CAN 통신으로 STM32F446RE에 전달하여 차량의 **정지·감속·주행**을 제어하는 프로젝트입니다.

STM32F446RE는 AI 주행 상태와 초음파 센서 값을 함께 판단해 모터를 제어하며, 포토인터럽터로 이동 거리와 택시 요금을 계산합니다.

---

## 프로젝트 정보

- 개발 형태: 팀 프로젝트
- 담당 역할:
  - 전체 자율주행 시스템 아키텍처 설계
  - 교통 객체 6종 데이터셋 구축
  - 초음파 센서 데이터 융합 로직 구현
  - 차동 구동 기반 주행 및 조향 알고리즘 설계

## 주요 기능

- YOLO11n 기반 교통 객체 6종 인식
- Raspberry Pi 4용 `320 × 320` NCNN 모델 적용
- CAN 통신 기반 Raspberry Pi 4 ↔ STM32F446RE 연동
- 초음파 센서 기반 장애물 회피 및 중앙 주행
- FreeRTOS 기반 모터·센서·요금 기능 분리
- 포토인터럽터 기반 거리 및 택시 요금 계산
- 객체 탐지 결과 영상 녹화

---

## 시스템 구성

```text
USB Webcam
    ↓
Raspberry Pi 4
- YOLO11n NCNN 객체 인식
- STOP / SLOW / GO 상태 판단
- CAN 메시지 송신
    ↓
STM32F446RE
- 초음파 센서 측정
- 모터 속도·방향 제어
- 이동 거리·택시 요금 계산
- I2C LCD 출력
```

---

## 인식 클래스

| 클래스 | 차량 동작 |
|---|---|
| `child_protect` | 어린이보호구역 감속 시작 |
| `child_protect_end` | 일반 주행 복귀 |
| `crosswalk` | 3초 정지 |
| `red_light` | 정지 |
| `yellow_light` | 감속 |
| `green_light` | 주행 |

```text
STOP = 0
SLOW = 1
GO   = 2
```

---

## 저장소 구조

```text
Object-detecting-autonomous-taxi/
├── AI_code/              # 학습 설정, 가중치, NCNN 모델, 결과
├── Bridge_code/          # 객체 인식, FSM, CAN, 영상 녹화
├── STM32F446_Code/       # FreeRTOS, 센서, 모터, LCD 제어
└── README.md
```

---

## 사용 기술

`YOLO11n` · `NCNN` · `Raspberry Pi 4` · `OpenCV` · `STM32F446RE` · `FreeRTOS` · `CAN` · `HC-SR04` · `L298N` · `I2C LCD`

---

## 프로젝트 결과

📑 [노션(Notion)](https://app.notion.com/p/4-390ef63b7a3a809d99a7c79ee05ec624)

🎬 [유튜브(YouTube) 객체 탐지 자율주행 택시 영상](https://www.youtube.com/watch?v=wJaPBZJurDA)

📊 [프로젝트 발표 PPT](https://www.miricanvas.com/v2/ko/design2/18f4ac1a-1930-4d8b-ab69-c4c35f988cd2)

