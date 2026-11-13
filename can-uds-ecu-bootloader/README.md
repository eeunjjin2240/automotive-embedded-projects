# ECU Bootloader and Firmware Integrity

I worked on a CAN/UDS reprogramming bootloader for an Infineon AURIX TC234LP training ECU in the HL Mando and HL Klemove Intelligent Vehicle School.

## Design and contribution

- Worked on the ECU-side programming flow, including diagnostic session and access handling, firmware transfer, and validation of the received image.
- Used separate staging and application regions so a download could be checked before replacing the application image. An interrupted download motivated the design.
- Configured CAN and flash MCAL components using EB tresos. Used Flash EEPROM Emulation (FEE) for nonvolatile storage in the training environment.

## Result

Received an Excellence Award for the bootloader and software binary integrity project in June 2026.

The project focused on ECU-side reprogramming over CAN. The update design addressed interruption during the download phase.
