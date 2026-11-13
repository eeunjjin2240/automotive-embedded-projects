# STM32 임베디드 실습

STM32 Nucleo 보드로 통신, 디지털 필터, 키패드·디스플레이를 구현한 개인 실습입니다.

| 프로젝트 | 코드 |
| --- | --- |
| [유·무선 통신](communications/) | UART, Bluetooth, ESP8266 Wi-Fi, I²C·SPI 슬레이브 |
| [디지털 필터](digital-filters/) | STM32G474 펌웨어 2개, C 필터 계산, MATLAB 계수 산출 |
| [키패드와 디스플레이](keypad-display/) | STM32F411 키패드·7세그먼트, STM32G474 LCD 펌웨어 |

각 프로젝트의 `.ioc`와 `Core/`에 보드 설정과 소스가 있습니다. STM32CubeIDE에서 프로젝트를 가져온 다음 `.ioc`에 지정된 Cube 펌웨어 패키지를 설치하고, 사용자 코드 보존을 켠 상태로 코드를 생성해 `Drivers/`를 준비합니다. ARM GCC로 원본 SDK 헤더를 사용해 C 소스 컴파일 검사를 했습니다. 보드 플래싱과 IDE 빌드는 이번 정리에서 재실행하지 않았습니다.

---

## English

## STM32 Embedded Labs

Individual laboratory work on STM32 Nucleo boards.

| Lab | Scope |
| --- | --- |
| [Wired and wireless communications](communications/) | CAN, UART, SPI, I²C, Wi-Fi, Bluetooth, and RFID |
| [Digital filter verification](digital-filters/) | MATLAB coefficient design, C filter steps, and STM32 output checks |
| [Keypad and multiplexed display](keypad-display/) | Matrix keypad scanning, timed display refresh, and buzzer feedback |

The `.ioc` and `Core/` directories contain the board configuration and source. In STM32CubeIDE, import the project, install the Cube firmware package identified in the `.ioc`, and regenerate with user-code preservation enabled to obtain `Drivers/`. The C sources passed ARM GCC checks against the original SDK headers. IDE builds and board flashing were not repeated.
