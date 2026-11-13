#ifndef __LCD_JLX1602A_H__
#define __LCD_JLX1602A_H__

#include "stm32g4xx_hal.h"

void LCD_Init(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void LCD_Clear(void);
#endif
