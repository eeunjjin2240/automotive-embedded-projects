# CAN/UDS ECU 부트로더

HL만도·HL클레무브 Intelligent Vehicle School에서 Infineon AURIX TC234LP 교육용 ECU의 CAN/UDS 리프로그래밍 부트로더를 개발했습니다.

- 진단 세션과 접근 처리, 펌웨어 수신, 수신 이미지 검증을 포함한 ECU 측 프로그래밍 흐름을 구현했습니다.
- 다운로드 중단에 대비해 수신 영역과 실행 영역을 분리하고, 검증된 이미지를 반영하는 흐름을 적용했습니다.
- EB tresos에서 CAN·Flash MCAL을 설정하고, FEE를 비휘발성 데이터 저장에 활용했습니다.

2026년 6월 부트로더·소프트웨어 바이너리 무결성 프로젝트에서 우수상을 받았습니다. 교육용 기본 코드와 직접 구현한 부분의 공개 범위를 확인 중입니다.

---

## English

## ECU Bootloader and Firmware Integrity

I worked on a CAN/UDS reprogramming bootloader for an Infineon AURIX TC234LP training ECU in the HL Mando and HL Klemove Intelligent Vehicle School.

### Design and contribution

- Worked on the ECU-side programming flow, including diagnostic session and access handling, firmware transfer, and validation of the received image.
- Used separate staging and application regions so a download could be checked before replacing the application image. An interrupted download motivated the design.
- Configured CAN and flash MCAL components using EB tresos. Used Flash EEPROM Emulation (FEE) for nonvolatile storage in the training environment.

### Result

Received an Excellence Award for the bootloader and software binary integrity project in June 2026.

The project focused on ECU-side reprogramming over CAN. The update design addressed interruption during the download phase.

The training base and personal changes are still being reviewed for public release.
