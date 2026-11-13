# STM32 임베디드 실습

STM32 Nucleo 보드로 통신, 디지털 필터, 키패드·디스플레이를 구현한 개인 실습입니다.

| 프로젝트 | 코드 |
| --- | --- |
| [유·무선 통신](communications/) | UART, Bluetooth, ESP8266 Wi-Fi, I²C·SPI 슬레이브 |
| [디지털 필터](digital-filters/) | STM32G474 펌웨어 2개, C 필터 계산, MATLAB 계수 산출 |
| [키패드와 디스플레이](keypad-display/) | STM32F411 키패드·7세그먼트, STM32G474 LCD 펌웨어 |

각 프로젝트의 `.ioc`와 `Core/`에 보드 설정과 소스가 있습니다. STM32CubeIDE에서 각 폴더를 기존 프로젝트로 가져와 빌드합니다. Drivers/에는 HAL·CMSIS가 포함되어 있습니다. .ioc를 수정해 다시 생성할 때는 지정된 Cube 펌웨어 패키지와 사용자 코드 보존 옵션을 사용합니다. 보드별 연결 핀은 각 프로젝트의 .ioc와 Core/Inc/main.h에 정의되어 있습니다.

---

## English

## STM32 Embedded Labs

Individual laboratory work on STM32 Nucleo boards.

| Lab | Scope |
| --- | --- |
| [Wired and wireless communications](communications/) | CAN, UART, SPI, I²C, Wi-Fi, Bluetooth, and RFID |
| [Digital filter verification](digital-filters/) | MATLAB coefficient design, C filter steps, and STM32 output checks |
| [Keypad and multiplexed display](keypad-display/) | Matrix keypad scanning, timed display refresh, and buzzer feedback |

The `.ioc` and `Core/` directories contain the board configuration and source. Import each folder as an existing STM32CubeIDE project and build it. Drivers/ includes HAL and CMSIS. When regenerating from the .ioc, use the specified Cube package and preserve user code. Board pin assignments are defined in the project .ioc and Core/Inc/main.h.
