#include "main.h"
#include "LCD_JLX1602A.h"

extern I2C_HandleTypeDef hi2c1;

#define JLX_ADDR (0x3C << 1)

static void LCD_SendCommand(uint8_t cmd)
{
    uint8_t data[2];
    data[0] = 0x00;   // Control byte: command
    data[1] = cmd;    // Command
    HAL_I2C_Master_Transmit(&hi2c1, JLX_ADDR, data, 2, HAL_MAX_DELAY);
    HAL_Delay(2);
}

static void LCD_SendData(uint8_t dat)
{
    uint8_t data[2];
    data[0] = 0x40;   // Control byte: data
    data[1] = dat;    // Data byte
    HAL_I2C_Master_Transmit(&hi2c1, JLX_ADDR, data, 2, HAL_MAX_DELAY);
}

// 초기화
void LCD_Init(void)
{
    HAL_Delay(50);  // 전원 안정화 대기

    // JLX1602A 전용 초기화 명령 시퀀스
    LCD_SendCommand(0x38); // Function set: 8bit, 2line, 5x8 dots
    LCD_SendCommand(0x39); // Function set: extended instruction
    LCD_SendCommand(0x14); // Internal OSC frequency
    LCD_SendCommand(0x70); // Contrast set (low bits)
    LCD_SendCommand(0x56); // Power/ICON/Contrast high bits
    LCD_SendCommand(0x6C); // Follower control (boost on)
    HAL_Delay(200);

    LCD_SendCommand(0x38); // Function set: normal instruction
    LCD_SendCommand(0x0C); // Display ON, cursor off
    LCD_SendCommand(0x01); // Clear display
    HAL_Delay(2);
}

// 커서 설정
void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_SendCommand(address);
}

// 문자열 출력
void LCD_Print(char *str)
{
    while (*str)
    {
        LCD_SendData((uint8_t)*str++);
    }
}

void LCD_Clear(void)
{
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}
