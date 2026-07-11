/**
  ******************************************************************************
  * @file    I2C_LCD.c
  * @brief   I2C 기반 HD44780 LCD 제어 드라이버 (PCF8574 I2C Expander 사용)
  * @details 4-bit 모드로 동작하며, MCU의 GPIO 핀 낭비 없이 I2C 통신만으로
  * LCD 화면에 센서 및 자율주행 상태 데이터를 실시간으로 출력합니다.
  ******************************************************************************
  */
#include "I2C_LCD.h"

extern I2C_HandleTypeDef hi2c1;

/**
  * @brief  LCD에 명령어(Command)를 전송합니다. (RS = 0)
  * @param  command 8-bit 명령어 데이터
  */
void lcd_command(uint8_t command)
{
    uint8_t high_nibble, low_nibble;
    uint8_t i2c_buffer[4];
    
    // 4-bit 모드: 8bit 데이터를 High/Low Nibble로 분할하여 전송
    high_nibble = command & 0xf0;
    low_nibble = (command << 4) & 0xf0;

    // 데이터 전송 시퀀스: EN(Enable) 핀을 High -> Low로 토글하여 데이터 Latch
    // 제어 비트 구성: [Data 4bit] | [Backlight(0x08)] | [EN(0x04)] | [RW(0x02)] | [RS(0x01)]
    i2c_buffer[0] = high_nibble | 0x04 | 0x08;  // en=1, rs=0, rw=0, backlight=1
    i2c_buffer[1] = high_nibble | 0x00 | 0x08;  // en=0, rs=0, rw=0, backlight=1 (Latch)
    i2c_buffer[2] = low_nibble  | 0x04 | 0x08;  // en=1, rs=0, rw=0, backlight=1
    i2c_buffer[3] = low_nibble  | 0x00 | 0x08;  // en=0, rs=0, rw=0, backlight=1 (Latch)

    // I2C 버스를 통해 4바이트 연속 전송 (Polling 방식으로 통신 완료 보장)
    while(HAL_I2C_Master_Transmit(&hi2c1, I2C_LCD_ADDRESS, i2c_buffer, 4, 100) != HAL_OK)
    {
        // 통신 안정화 대기 (필요 시 활성화)
        //HAL_Delay(1);
    }
}

/**
  * @brief  LCD에 출력할 문자 데이터(Data)를 전송합니다. (RS = 1)
  * @param  data 8-bit 아스키코드 데이터
  */
void lcd_data(uint8_t data)
{
    uint8_t high_nibble, low_nibble;
    uint8_t i2c_buffer[4];
    
    // 4-bit 모드 데이터 분할
    high_nibble = data & 0xf0;
    low_nibble = (data << 4) & 0xf0;

    // 명령어 전송과 동일하나, 문자 출력을 위해 RS(Register Select) 비트를 1(0x01)로 설정
    i2c_buffer[0] = high_nibble | 0x05 | 0x08;  // en=1, rs=1, rw=0, backlight=1
    i2c_buffer[1] = high_nibble | 0x01 | 0x08;  // en=0, rs=1, rw=0, backlight=1 (Latch)
    i2c_buffer[2] = low_nibble  | 0x05 | 0x08;  // en=1, rs=1, rw=0, backlight=1
    i2c_buffer[3] = low_nibble  | 0x01 | 0x08;  // en=0, rs=1, rw=0, backlight=1 (Latch)

    while(HAL_I2C_Master_Transmit(&hi2c1, I2C_LCD_ADDRESS, i2c_buffer, 4, 100) != HAL_OK)
    {
        //HAL_Delay(1);
    }
}

/**
  * @brief  LCD 초기화 시퀀스 (HD44780 데이터시트 기준 4-bit 초기화 규격 준수)
  */
void i2c_lcd_init()
{
    HAL_Delay(50);           // 전원 인가 후 시스템 안정화 대기 (>40ms)
    lcd_command(0x33);       // 8-bit 모드 초기화 시퀀스 (강제 리셋)
    HAL_Delay(5);
    lcd_command(0x32);       // 8-bit -> 4-bit 모드로 전환
    HAL_Delay(5);
    lcd_command(0x28);       // Function Set: 4-bit 모드, 2 Line 표시, 5x8 폰트 사용
    HAL_Delay(5);
    lcd_command(DISPLAY_ON); // Display 제어: 화면 ON, 커서 OFF, 깜빡임 OFF
    HAL_Delay(5);
    lcd_command(0x06);       // Entry Mode Set: 문자 출력 후 커서를 자동으로 우측 이동
    HAL_Delay(5);
    lcd_command(CLEAR_DISPLAY); // 화면 전체 지우기 및 커서 홈(0,0) 복귀
    HAL_Delay(2);
}

/**
  * @brief  문자열을 LCD에 연속으로 출력합니다.
  * @param  str 출력할 널 종료(Null-terminated) 문자열 포인터
  */
void lcd_string(char *str)
{
    // 널 문자('\0')를 만날 때까지 포인터를 증가시키며 한 글자씩 전송
    while(*str) lcd_data(*str++); 
}

/**
  * @brief  LCD 커서 위치를 원하는 좌표로 이동시킵니다.
  * @param  row 이동할 행 (0: 첫 번째 줄, 1: 두 번째 줄)
  * @param  col 이동할 열 (0 ~ 15)
  */
void move_cusor(uint8_t row, uint8_t col)
{
    // DDRAM(Display Data RAM) 주소 설정 명령어는 최상위 비트가 1(0x80)이어야 함
    // 16x2 LCD에서 두 번째 줄(row=1)의 시작 주소는 0x40이므로, 
    // 비트 시프트 연산(row << 6)을 활용하여 주소를 동적으로 계산
    lcd_command(0x80 | row << 6 | col);
}
