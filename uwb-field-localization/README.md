# UWB Localization for Agricultural Machinery

In an industry-sponsored capstone, I was responsible for embedded software for UWB localization. The goal was to provide position data for a prototype agricultural machine.

## System and contribution

```mermaid
flowchart LR
    A[Four UWB anchors] --> B[DWM3000 tag]
    B -->|SPI and interrupt| C[STM32]
    C -->|UART| D[Raspberry Pi]
    C -->|UART| E[ESP8266]
    E -->|Wi-Fi| F[Android display]
```

I extended a vendor reference implementation of double-sided two-way ranging to a multi-anchor system, worked on tag-side position estimation, and integrated live position output to a Raspberry Pi and an Android prototype.

## Field observations

The summer project report documents position updates within a roughly 50 × 50 m test area. Communication became less stable beyond that area depending on antenna orientation. With the tag stationary, the reported position fluctuated by up to roughly 5 cm in the observed test. That figure describes stationary variation. Absolute position error across the full field still needed a reference measurement.

I documented hardware connections and setup variants for the next person bringing up the system.
