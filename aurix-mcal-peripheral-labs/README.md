# AURIX MCAL 주변장치 실습

Infineon AURIX TC234LP 교육용 ECU에서 EB tresos로 제공 드라이버를 설정하고 애플리케이션에서 각 API를 연동했습니다.

| 실습 | 주요 작업 |
| --- | --- |
| 타이밍과 입출력 | GPT, ADC, Port, DIO 설정 |
| 센서와 차량 통신 | SPI 6축 센서 읽기와 CAN 송수신 |
| 비휘발성 메모리 | FLS와 FEE 연동, Trace32로 가비지 컬렉션 전후 데이터 이동 관찰 |
| 감시와 계측 | Watchdog 설정과 XCP 기본 계측 |

소스 아카이브는 교육용 예제와 공급사 드라이버를 포함합니다. 개인 구현 범위를 확인할 수 있는 파일이 확보되면 추가할 수 있습니다.

---

## English

## AURIX MCAL Peripheral Labs

Hardware exercises on the Infineon AURIX TC234LP training ECU. I used EB tresos to configure provided drivers and connected their APIs to the training applications.

| Lab area | Work |
| --- | --- |
| Timing and I/O | Configured GPT, ADC, Port, and DIO for periodic tasks and analog/digital signals. |
| Sensor and vehicle communication | Used SPI to read a six-axis sensor and exercised CAN transmission and reception. |
| Nonvolatile memory | Worked with the Flash MCAL driver and the Flash EEPROM Emulation layer. Used Trace32 to observe storage movement during garbage collection. |
| Monitoring | Exercised watchdog configuration and basic XCP measurement. |

**Environment** · AURIX Development Studio, EB tresos, and Trace32.

The source archive contains course examples and vendor drivers. Individual contributions need to be identified before publication.
