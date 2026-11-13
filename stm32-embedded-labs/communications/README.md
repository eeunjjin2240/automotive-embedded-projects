# 유·무선 통신 실습

| 펌웨어 | 보드 | 실습 내용 |
| --- | --- | --- |
| [UART](uart-f411/) | Nucleo-F411RE | 폴링과 수신 인터럽트를 사용한 시리얼 통신 |
| [Bluetooth UART](bluetooth-uart-h533/) | Nucleo-H533RE | Bluetooth 모듈과 PC 터미널 사이의 메시지 전달 |
| [ESP8266 Wi-Fi](esp8266-wifi-h533/) | Nucleo-H533RE | ESP8266 AT 명령을 사용한 LED 제어와 상태 교환 |
| [I²C 슬레이브](i2c-slave-g474/) | Nucleo-G474RE | DMA 수신 |
| [SPI 슬레이브](spi-slave-g474/) | Nucleo-G474RE | SPI 수신 데이터에 따른 LED 제어 |

Wi-Fi 예제의 `AP_SSID`와 `AP_PW`는 실습용 예시값입니다. 실행 전 `Core/Src/main.c`에서 원하는 값으로 바꾸면 됩니다. CAN·RFID도 실습했으며 여기에는 위 5개의 펌웨어 소스를 포함했습니다.

---

## English

## Wired and Wireless Communications

- Implemented UART, SPI, I²C, CAN, RFID, Wi-Fi, and Bluetooth exercises.
- In the CAN exercise, verified that a button press on either STM32 board changed the LED timing on both boards. Captured the CAN waveform as bench evidence.
- Tested bidirectional Bluetooth data transfer using polling and interrupt-driven UART. Verified Wi-Fi command and status exchange with an app.

| Firmware | Board |
| --- | --- |
| [UART](uart-f411/) | Nucleo-F411RE |
| [Bluetooth UART](bluetooth-uart-h533/) | Nucleo-H533RE |
| [ESP8266 Wi-Fi](esp8266-wifi-h533/) | Nucleo-H533RE |
| [I²C slave](i2c-slave-g474/) | Nucleo-G474RE |
| [SPI slave](spi-slave-g474/) | Nucleo-G474RE |

Set `AP_SSID` and `AP_PW` in the Wi-Fi project's `Core/Src/main.c` before use. This directory contains the five firmware projects listed above.
