# Electronics Subsystem: BaseCode V0.1 Firmware Documentation

## Overview & Objectives
BaseCode V0.1 is an isolated hardware verification harness designed for bench-testing dead-wheel quadrature encoders on the KARMA base for RoboCup SML/@work 2026. This version strips out high-level actuators and IMU routines to validate timer hardware decoding, pulse counting accuracy, and phase ordering across three encoder channels before full subsystem integration.

Key engineering objectives:
* Configure STM32F411 timers TIM2, TIM3, and TIM4 in Encoder Mode (X4 decoding on TI1 and TI2).
* Verify 4096 counts per revolution (CPR) under manual rotational calibration.
* Output raw encoder tick counts over USB CDC in a low-overhead CSV format (`E,e1,e2,e3\r\n`) at 20 Hz.

## Technical Specifications & Bill of Materials (BOM)

### System Specifications
* **Microcontroller:** STM32F411CEU6 (WeAct Black Pill), ARM Cortex-M4 @ 96 MHz.
* **Firmware Scope:** Bench validation harness (Encoders + USB CDC only).
* **Sampling Rate:** 20 Hz (50 ms polling loop).
* **Communication Interface:** USB CDC Virtual COM Port (115200 baud).

### Bill of Materials (BOM)

| Component | Part Number / Model | Manufacturer | Key Specification | Quantity | Reference Designator |
| :--- | :--- | :--- | :--- | :--- | :--- |
| MCU Board | STM32F411CEU6 | WeAct Studio | Cortex-M4, 96 MHz, 512 KB Flash | 1 | U1 |
| Optical Encoders | Incremental Optical Kit | Broadcom / Avago | 1024 CPR (4096 counts/rev in X4 mode) | 3 | ENC1–ENC3 |
| Tracking Assembly | Dead Wheel Omni Rig | Custom RoboManipal | Radius $R = 35.0\text{ mm}$, Ground-sprung assembly | 3 | DW1–DW3 |

## Design & Architecture

### Encoder Hardware Timer Configuration
Each encoder interface is driven directly by hardware timer peripherals configured in Encoder Mode (both edges of TI1 and TI2), delivering 4x resolution:

```{code-block} c
/* Encoder initialization from main.c */
encoder_init(&htim2, &htim3, &htim4);

/* Periodic 50 ms polling loop */
while (1) {
    if (HAL_GetTick() - s_last_tick_50ms >= 50) {
        s_last_tick_50ms = HAL_GetTick();
        encoder_update();
        int elen = snprintf(txbuf, sizeof(txbuf), "E,%ld,%ld,%ld\r\n",
                             (long)encoder_get_count(0),
                             (long)encoder_get_count(1),
                             (long)encoder_get_count(2));
        CDC_Transmit_FS((uint8_t*)txbuf, elen);
    }
}
```

### Pulse Calculation Formula
For a single full rotation of the $35.0\text{ mm}$ radius tracking wheel:
```{math}
\text{Expected Ticks} = 1024\text{ lines} \times 4 = 4096\text{ ticks}
```
Linear surface distance traveled per full wheel turn:
```{math}
C = 2 \pi R = 2 \pi \times 0.035\text{ m} \approx 0.2199\text{ m}
```

## Software/Hardware Interface

### Hardware Pin Mapping Table

| Encoder Identifier | Timer Channel | Pin | Alternate Function | Phase Signal |
| :--- | :--- | :--- | :--- | :--- |
| Encoder 1 (Left) | TIM4_CH1 | PB6 | AF2 (TIM4) | Channel A |
| Encoder 1 (Left) | TIM4_CH2 | PB7 | AF2 (TIM4) | Channel B |
| Encoder 2 (Right) | TIM3_CH1 | PB4 | AF2 (TIM3) | Channel A |
| Encoder 2 (Right) | TIM3_CH2 | PB5 | AF2 (TIM3) | Channel B |
| Encoder 3 (Back) | TIM2_CH1 | PA15 | AF1 (TIM2) | Channel B |
| Encoder 3 (Back) | TIM2_CH2 | PB3 | AF1 (TIM2) | Channel A |
| Serial Stream | USB_FS | PA11, PA12 | AF10 (USB_OTG_FS) | Virtual COM Port |

### Communication Protocol
* **Format:** ASCII CSV string terminated by CRLF.
* **Payload Example:** `E,4096,-2048,12\r\n`
* **Interpretation:** Encoder 1 has completed +1.0 rev (+4096 ticks), Encoder 2 has rotated -0.5 rev (-2048 ticks), Encoder 3 has minimal jitter (+12 ticks).

## Future Work / Known Limitations
* **Scope Restriction:** BaseCode V0.1 does not incorporate motor control, IMU reads, or kinematic pose integration.
* **Status:** Deprecated bench validation firmware; incorporated into BaseCode V1.0 and V2.0.
