# 키패드와 디스플레이

- [F411 키패드](firmware/f411-keypad/) · 매트릭스 키패드, 타이머 인터럽트를 이용한 7세그먼트 갱신, 부저 피드백을 구현했습니다. 코드에 있는 PIN은 실습 예시값입니다.
- [G474 LCD](firmware/g474-lcd/) · JLX1602A LCD 초기화, 커서 위치 지정, 문자열 출력 코드를 포함합니다.

---

## English

## Keypad and Multiplexed Display

Built a four-digit keypad prototype on a Nucleo-F411RE. A timer interrupt refreshed the seven-segment display, while the application handled matrix keypad input, visual feedback, and a piezo buzzer.

The lab demonstrated keypad entry, display updates, and distinct feedback for accepted and rejected input.

The [F411 keypad firmware](firmware/f411-keypad/) uses a demonstration PIN. The [G474 LCD firmware](firmware/g474-lcd/) includes the JLX1602A initialization and display routines.
