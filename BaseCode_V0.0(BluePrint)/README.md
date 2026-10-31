# Electronics Subsystem: BaseCode V0.0 Firmware Documentation

## Overview & Objectives
BaseCode V0.0 represents the initial architectural blueprint for the KARMA base firmware on the STM32F411CEU6 microcontroller for RoboCup SML/@work 2026. This version evaluated basic pin assignments and peripheral allocations, attempting simultaneous operation of 3 quadrature encoder timer channels, 4 motor PWM outputs, BNO085 I2C communication, and USB CDC telemetry.

Key engineering objectives:
* Establish baseline CubeMX peripheral initialization for TIM2, TIM3, TIM4, TIM5, I2C3, and USB_OTG_FS.
* Verify BNO085 SH-2 firmware boot communication and probe response over I2C3.
* Map physical pins to identify timer channel conflicts before board manufacturing.

## Technical Specifications & Bill of Materials (BOM)

### System Specifications
* **Microcontroller:** STM32F411CEU6, ARM Cortex-M4 @ 96 MHz, 512 KB Flash.
* **Firmware Topology:** Bare-metal proof of concept.
* **Timer Mapping (Blueprint):**
  * Encoders: TIM2, TIM3, TIM5.
  * Motors: TIM4 PWM.
* **IMU Interface:** I2C3 (PA8 SCL, PB8 SDA) with external RST and INT control pins.

### Bill of Materials (BOM)

| Component | Model | Manufacturer | Key Specification | Quantity | Reference Designator |
| :--- | :--- | :--- | :--- | :--- | :--- |
| MCU Board | STM32F411CEU6 (Black Pill) | WeAct Studio | Cortex-M4F, 96 MHz, 128 KB SRAM | 1 | U1 |
| Motor Drivers | MD30C | Cytron Technologies | 30 A DC Motor Driver | 4 | MD1–MD4 |
| IMU Sensor | BNO085 | Hillcrest Labs | 9-DOF SiP Sensor Hub | 1 | IMU1 |
| Optical Encoders | Incremental Optical | Broadcom / Avago | 1024 CPR Quadrature | 3 | ENC1–ENC3 |

## Design & Architecture

### Pin Conflict Discovery & Analysis
In the initial blueprint configuration:
* TIM5 was assigned to dead-wheel encoders, leaving TIM4 for motor PWM generation.
* However, TIM4 only possesses 4 output compare channels, which prevented independent PWM routing due to board trace routing conflicts with adjacent I2C/SPI pins.
* Furthermore, TIM5 is a full 32-bit timer with 4 independent PWM channels on PA0..PA3, making it better suited for 4-wheel independent PWM generation.

### Legacy Initialization Snippet
From `BaseCode_V0.0(BluePrint)/Core/Src/main.c`:

```{code-block} c
/* Peripheral initialization in V0.0 blueprint */
bno085_init(&hi2c3, BNO_RST_GPIO_Port, BNO_RST_Pin, BNO_INT_GPIO_Port, BNO_INT_Pin);
bno085_hw_reset();

uint8_t maj, min;
if (bno085_probe(&maj, &min)) {
    snprintf(txbuf, sizeof(txbuf), "BNO085 alive, SH-2 fw v%u.%u\r\n", maj, min);
}

encoder_init(&htim2, &htim3, &htim5);
motor_init(&htim4);
motor_stop_all();
```

## Software/Hardware Interface

### Initial Blueprint Pin Table

| Peripheral | Assigned Pin | Function in V0.0 | Conflict / Reroute in V1.0+ |
| :--- | :--- | :--- | :--- |
| TIM5_CH1..CH2 | PA0, PA1 | Encoder 3 Inputs | Reassigned to Motor 1 & Motor 2 PWM (TIM5) |
| TIM4_CH1..CH4 | PB6..PB9 | Motor PWM Outputs | Reassigned: PB6/PB7 to Encoder 1, PB8 to I2C3_SDA |
| I2C3 | PA8, PB8 | BNO085 SCL / SDA | Preserved in subsequent revisions |
| USB OTG FS | PA11, PA12 | Host Telemetry Link | Preserved in subsequent revisions |

## Future Work / Known Limitations
* **Architectural Infeasibility:** Timer routing in V0.0 created pin contention between motor PWM and encoder lines.
* **Resolution:** Superseded by BaseCode V1.0, where TIM5 was dedicated to 4x motor PWM (PA0..PA3) and encoders were reassigned to TIM4, TIM3, and TIM2.
