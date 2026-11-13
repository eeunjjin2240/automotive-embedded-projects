#ifndef KEYPAD_H
#define KEYPAD_H

int keypad(void);
void SEG_CLEAR(void);
void DIG_OFF_ALL(void);
void keypad_task(void);
void buzzer_task(void);
void blink_task(void);

#endif
