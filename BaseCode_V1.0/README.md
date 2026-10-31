# Electronics Subsystem: BaseCode V1.0 Firmware Documentation

## Overview & Objectives
BaseCode V1.0 serves as the baseline bare-metal firmware for the autonomous 4-wheel mobile base of RoboManipal's RoboCup SML/@work 2026 entry (KARMA platform). The primary objective of this version is integrating hardware peripherals into a single sequential superloop (`robot_loop()`) running on an STM32F411CEU6 MCU to capture dead-wheel odometry, read inertial orientation, drive four brushed DC motors, and stream JSON telemetry to an NVIDIA Jetson Orin companion computer over USB CDC at 50 Hz.

Key engineering goals:
* Sample three unpowered spring-loaded dead-wheel quadrature encoders using hardware timer encoder modes.
* Query a BNO085 9-DOF IMU over I2C3 using Hillcrest Sensor Hub Transport Protocol (SHTP) for orientation and linear acceleration.
* Generate four independent 20 kHz PWM channels via TIM5 coupled with directional GPIOs to control Cytron MD30C motor drivers.
* Provide bidirectional USB CDC serial communication parsing ASCII JSON command payloads and responding with periodic state telemetry frames.

## Technical Specifications & Bill of Materials (BOM)

### System Specifications
* **Microcontroller:** STM32F411CEU6 (WeAct Black Pill), ARM Cortex-M4 @ 96 MHz, 512 KB Flash, 128 KB SRAM.
* **Core Operating Voltage:** 3.3 V DC logic level.
* **Actuator Power Bus:** 24.0 V DC nominal LiPo/LiFePO4 battery pack.
* **Control Loop Topology:** Bare-metal polling superloop with `HAL_GetTick()` software millisecond timers.
* **Telemetry Sampling Rate:** 50 Hz (nominal 20 ms interval).
* **Communication Link:** USB 2.0 Full Speed CDC Virtual COM Port (12 Mbps PHY, 115200 baud terminal configuration).

### Bill of Materials (BOM)

| Component | Part Number / Model | Manufacturer | Key Specification | Quantity | Reference Designator |
| :--- | :--- | :--- | :--- | :--- | :--- |
| MCU Board | STM32F411CEU6 (Black Pill) | WeAct Studio | 32-bit Cortex-M4F, 96 MHz, USB-C | 1 | U1 |
| Motor Driver | MD30C | Cytron Technologies | 5 V–30 V DC, 30 A continuous (80 A peak), 20 kHz PWM | 4 | MD1–MD4 |
| IMU Module | BNO085 | CEVA / Hillcrest Labs | 9-DOF SiP with Cortex-M0+ running SH-2 sensor hub | 1 | IMU1 |
| Tracking Wheel | Dead Wheel Omni Assembly | Custom RoboManipal | Radius $R = 35.0\text{ mm}$, Track separation $W = 150.0\text{ mm}$ | 3 | DW1–DW3 |
| Encoders | Optical Quadrature Kit | Broadcom / Avago | Incremental AB phase, 1024 CPR (4096 counts/rev X4) | 3 | ENC1–ENC3 |
| Companion PC | Jetson Orin Nano / NX | NVIDIA | Ubuntu 22.04 LTS, ROS 2 Humble | 1 | COMP1 |

## Design & Architecture

### Mechanical & Kinematic Parameters
* **Tracking Wheel Radius ($R$):** $35.0\text{ mm}$ ($0.035\text{ m}$)
* **Tracking Wheelbase Width ($W$):** $150.0\text{ mm}$ ($0.150\text{ m}$)
* **Encoder Counts Per Revolution ($\text{CPR}$):** $4096\text{ counts/rev}$ (X4 hardware decoding)
* **Linear Distance Resolution per Tick ($k_s$):**
  ```{math}
  k_s = \frac{2 \pi R}{\text{CPR}} = \frac{2 \pi \times 0.035}{4096} \approx 5.371 \times 10^{-5}\text{ m/tick}
  ```

### Kinematics & Dead Reckoning Formulation
Local pose integration computes the midpoint displacement between left and right tracking dead wheels across discrete sampling intervals $\Delta t$:

```{math}
\Delta s = \frac{\Delta s_{\text{left}} + \Delta s_{\text{right}}}{2} = \frac{k_s (\Delta e_{\text{left}} + \Delta e_{\text{right}})}{2}
```

```{math}
\Delta \theta = \frac{k_s (\Delta e_{\text{right}} - \Delta e_{\text{left}})}{W}
```

The global position tuple $(x, y, \theta)$ updates through Runge-Kutta 2nd order (midpoint arc) approximation:

```{math}
x_{k+1} = x_k + \Delta s \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)
```

```{math}
y_{k+1} = y_k + \Delta s \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)
```

```{math}
\theta_{k+1} = \theta_k + \Delta \theta
```

### BaseCode V1.0 Polling Control Flow
BaseCode V1.0 executes inside an unbounded `while (1)` superloop in `main.c` invoking `robot_loop()`. All sub-modules (encoders, IMU SHTP processing, USB reception, command execution, and telemetry transmission) are polled sequentially:

```{mermaid}
graph TD
    A[main while loop] --> B[robot_loop]
    B --> C{BNO085 Ready?}
    C -- Yes --> D[BNO085_Update x8 polling]
    C -- No --> E[Service Pending USB ACK]
    D --> E
    E --> F{HAL_GetTick - last_telemetry >= 20ms?}
    F -- No --> B
    F -- Yes --> G[encoder_update: Sample TIM4, TIM3, TIM2]
    G --> H[BNO085_GetGameRotation: Read Yaw]
    H --> I[BNO085_GetLinearAccel: Read Accel]
    I --> J[snprintf: Serialize JSON Telemetry Frame]
    J --> K[CDC_Transmit_FS: Transmit to Jetson]
    K --> B
```

### Polling Implementation Snippets
Periodic telemetry formatting and non-blocking transmit invocation from `robot.c`:

```{code-block} c
void robot_loop(void) {
    /* 1. Process pending BNO085 SHTP packets */
    if (s_bno_ready) {
        for (int i = 0; i < 8; i++) {
            if (BNO085_Update(&s_bno) != BNO_PORT_OK) break;
        }
    }

    /* 2. Service command ACK if pending */
    if (s_pending_ack) {
        if (CDC_Transmit_FS((uint8_t *)s_ack_buf, (uint16_t)strlen(s_ack_buf)) == USBD_OK) {
            s_pending_ack = false;
        }
    }

    /* 3. Periodic JSON Telemetry Output (50 Hz / 20 ms) */
    if (!s_pending_ack && (HAL_GetTick() - s_last_telemetry_tick >= TELEMETRY_INTERVAL_MS)) {
        s_last_telemetry_tick = HAL_GetTick();

        encoder_update();
        int32_t e1 = encoder_get_count(0);
        int32_t e2 = encoder_get_count(1);
        int32_t e3 = encoder_get_count(2);

        BNO_Quaternion q;
        float yaw = 0.0f;
        if (BNO085_GetGameRotation(&s_bno, &q) || BNO085_GetRotation(&s_bno, &q)) {
            yaw = BNO085_GetEulerYaw(&q);
        }

        BNO_Vector3 accel = {0.0f, 0.0f, 0.0f};
        if (!BNO085_GetLinearAccel(&s_bno, &accel)) {
            BNO085_GetAccel(&s_bno, &accel);
        }

        int len = snprintf(s_tx_buf, sizeof(s_tx_buf),
            "{\"yaw\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"e1\":%ld,\"e2\":%ld,\"e3\":%ld,\"bno_ok\":%d,\"pwm\":[%d,%d,%d,%d]}\r\n",
            yaw, accel.x, accel.y, accel.z, (long)e1, (long)e2, (long)e3,
            s_bno_ready ? 1 : 0, s_motor_pwm[0], s_motor_pwm[1], s_motor_pwm[2], s_motor_pwm[3]);

        if (len > 0) {
            CDC_Transmit_FS((uint8_t *)s_tx_buf, (uint16_t)len);
        }
    }
}
```

## Software/Hardware Interface

### STM32 Hardware Pin Allocations

| Peripheral | Function | STM32 Pin | Alternate Function | Connected Device / Channel | Logic Level |
| :--- | :--- | :--- | :--- | :--- | :--- |
| TIM5 | PWM Output | PA0 | AF2 (TIM5_CH1) | MD30C Motor 1 PWM | 3.3 V CMOS |
| TIM5 | PWM Output | PA1 | AF2 (TIM5_CH2) | MD30C Motor 2 PWM | 3.3 V CMOS |
| TIM5 | PWM Output | PA2 | AF2 (TIM5_CH3) | MD30C Motor 3 PWM | 3.3 V CMOS |
| TIM5 | PWM Output | PA3 | AF2 (TIM5_CH4) | MD30C Motor 4 PWM | 3.3 V CMOS |
| GPIO | Direction Output | PB0 | GPIO Output | MD30C Motor 1 DIR | 3.3 V CMOS |
| GPIO | Direction Output | PB1 | GPIO Output | MD30C Motor 2 DIR | 3.3 V CMOS |
| GPIO | Direction Output | PB2 | GPIO Output | MD30C Motor 3 DIR | 3.3 V CMOS |
| GPIO | Direction Output | PB12 | GPIO Output | MD30C Motor 4 DIR | 3.3 V CMOS |
| TIM4 | Encoder CH1 | PB6 | AF2 (TIM4_CH1) | Dead-Wheel Encoder 1 Channel A | 3.3 V / 5 V FT |
| TIM4 | Encoder CH2 | PB7 | AF2 (TIM4_CH2) | Dead-Wheel Encoder 1 Channel B | 3.3 V / 5 V FT |
| TIM3 | Encoder CH1 | PB4 | AF2 (TIM3_CH1) | Dead-Wheel Encoder 2 Channel A | 3.3 V / 5 V FT |
| TIM3 | Encoder CH2 | PB5 | AF2 (TIM3_CH2) | Dead-Wheel Encoder 2 Channel B | 3.3 V / 5 V FT |
| TIM2 | Encoder CH1 | PA15 | AF1 (TIM2_CH1) | Dead-Wheel Encoder 3 Channel B | 3.3 V / 5 V FT |
| TIM2 | Encoder CH2 | PB3 | AF1 (TIM2_CH2) | Dead-Wheel Encoder 3 Channel A | 3.3 V / 5 V FT |
| I2C3 | Clock (SCL) | PA8 | AF4 (I2C3_SCL) | BNO085 SCL (400 kHz Fast-Mode) | 3.3 V OD (PU) |
| I2C3 | Data (SDA) | PB8 | AF9 (I2C3_SDA) | BNO085 SDA (400 kHz Fast-Mode) | 3.3 V OD (PU) |
| USB OTG FS | Data Minus (DM) | PA11 | AF10 (OTG_FS_DM) | Jetson Orin USB-C / Type-A Port | USB PHY |
| USB OTG FS | Data Plus (DP) | PA12 | AF10 (OTG_FS_DP) | Jetson Orin USB-C / Type-A Port | USB PHY |

### Communication Protocols & Payloads

#### USB CDC Serial Configuration
* **Baud Rate:** 115200 bps
* **Data Bits:** 8
* **Parity:** None
* **Stop Bits:** 1
* **Flow Control:** None (Virtual COM Port)

#### Jetson $\to$ STM32 Command Frames (ASCII JSON)
1. **Four-Wheel Direct PWM:**
   ```json
   {"m": [300, 300, -300, -300]}
   ```
2. **Individual Motor Assignment:**
   ```json
   {"m1": 250, "m2": 250, "m3": -250, "m4": -250}
   ```
3. **Uniform Speed Broadcast:**
   ```json
   {"speed": 400}
   ```
4. **Emergency Stop:**
   ```json
   {"cmd": "stop"}
   ```

#### STM32 $\to$ Jetson Telemetry Frame (50 Hz ASCII JSON)
```json
{"yaw": 142.35, "x": 0.04, "y": -0.01, "z": 9.81, "e1": 15420, "e2": 15398, "e3": 12, "bno_ok": 1, "pwm": [300, 300, -300, -300]}
```

### ROS 2 Interface Mapping

| ROS 2 Topic | Message Type | Direction | Update Rate | Description |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscription | Up to 50 Hz | Linear $v_x, v_y$ and angular $\omega_z$ velocity converted to motor PWM commands by host bridge node. |
| `/odom` | `nav_msgs/msg/Odometry` | Publication | 50 Hz | Filtered dead-wheel odometry position ($x, y$), linear velocity ($v$), and orientation. |
| `/imu/data` | `sensor_msgs/msg/Imu` | Publication | 50 Hz | Fused orientation quaternion ($q_w, q_x, q_y, q_z$) and linear acceleration from BNO085. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Publication | 50 Hz | Raw wheel positions derived from encoder ticks $e_1, e_2, e_3$. |

## Future Work / Known Limitations

### Critical Limitations Identified in BaseCode V1.0
* **USB Transmit Overwrites (`s_tx_buf` corruption):** `CDC_Transmit_FS()` passes a memory pointer directly to the USB peripheral without double buffering or completion synchronization. If the polling loop initiates the next transmission before the hardware packet completes, `s_tx_buf` is partially overwritten, generating malformed JSON strings (`JSONDecodeError` on Jetson).
* **Silent Packet Drops (`USBD_BUSY`):** If a command ACK and periodic telemetry frame coincide, the second `CDC_Transmit_FS()` returns `USBD_BUSY` and the frame is dropped silently.
* **I2C Bus Lockup Susceptibility:** Motor back-EMF and switching transients on the 24 V rail can cause the BNO085 to stall mid-transaction, holding SDA low. BaseCode V1.0 has no automatic 9-clock recovery routine, causing the superloop to hang permanently.
* **Unbounded Loop Jitter:** SHTP packet parsing latency varies depending on internal BNO085 report queues, causing loop execution timing to fluctuate between 1 ms and 40 ms.
* **Unreliable Deadman Watchdog:** The watchdog timeout check resides inside the polling loop; if an I2C transaction blocks, the motor cutoff check is delayed, creating a runaway hazard.

### Target Improvements for Next Versions
* Transition architecture from bare-metal superloop to **strict zero-heap FreeRTOS** multi-tasking (Rate Monotonic Scheduling).
* Decouple BNO085 driver lifecycle using a formal **2D State Matrix transition table** with automatic 9-clock I2C bus recovery.
* Implement binary semaphore hardware flow control (`xUsbTxSemaphore`) for reliable USB CDC telemetry.
* Isolate motor safety control into a dedicated high-priority task (100 Hz) with an autonomous 150 ms deadman watchdog.
