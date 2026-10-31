# Electronics Subsystem: RoboManipal RoboCup SML/@work 2026 Base Documentation

## Overview & Objectives
This document details the Electronics and Low-Level Embedded Firmware subsystem for the KARMA autonomous 4-wheel mobile base, engineered by RoboManipal for the RoboCup SML/@work 2026 competition. The subsystem bridges physical sensor acquisition, actuator drive electronics, and host autonomous navigation running on an NVIDIA Jetson Orin companion computer.

The documentation encompasses the complete firmware architectural lineage:
* **BaseCode V0.0–V1.1 (Bare-Metal Polling):** Sequential superloop (`robot_loop()`) managing dead-wheel encoder sampling, Hillcrest SHTP IMU packet polling over I2C3, and USB CDC JSON streaming at 50 Hz.
* **BaseCode V2.0 (Zero-Heap FreeRTOS + 2D State Matrix):** Hard real-time multi-tasking architecture utilizing Rate Monotonic Scheduling (RMS), zero dynamic memory allocation, formal 2D state-machine IMU lifecycle management with automatic 9-clock I2C bus recovery, and semaphore-gated USB flow control.

Key engineering objectives:
* Real-time dead-reckoning odometry computation from 3-channel spring-loaded dead-wheel quadrature encoders ($R = 35.0\text{ mm}, W = 150.0\text{ mm}$).
* Drift-free inertial orientation tracking using a BNO085 9-DOF sensor hub outputting Game Rotation Vector quaternions at 50 Hz.
* 4-channel independent 20 kHz PWM generation driving Cytron MD30C motor drivers.
* Elimination of communication lockups, task starvation, and USB buffer collisions via preemptive real-time scheduling.

## Technical Specifications & Bill of Materials (BOM)

### System Technical Specifications
* **Main Embedded Controller:** STM32F411CEU6 (WeAct Black Pill), ARM Cortex-M4F @ 96 MHz, 512 KB Flash, 128 KB SRAM.
* **High-Level Compute Host:** NVIDIA Jetson Orin Nano / NX (ROS 2 Humble / Iron on Ubuntu 22.04 LTS).
* **Actuator Drive Bus:** 24.0 V DC nominal, powered by high-discharge lithium battery packs.
* **Motor Drivers:** 4x Cytron MD30C Rev 2.0 (Single Channel, 5 V–30 V, 30 A continuous / 80 A peak).
* **PWM Switching Frequency:** 20.0 kHz (TIM5 channels 1 to 4, 16-bit ARR = 4799 @ 96 MHz APB1 timer clock).
* **Safety Watchdog:** 150 ms autonomous deadman software cutoff.
* **Dead-Wheel Tracking Geometry:** Wheel radius $R = 35.0\text{ mm}$ ($0.035\text{ m}$), Wheelbase tracking track width $W = 150.0\text{ mm}$ ($0.150\text{ m}$).
* **Primary Sensors:**
  * 3x Broadcom/Avago Optical Quadrature Encoders (1024 CPR optical disk, 4096 counts/revolution in X4 hardware decode mode).
  * 1x CEVA / Hillcrest Labs BNO085 9-DOF SiP Sensor Hub (I2C3 @ 400 kHz Fast-Mode).
* **Telemetry & Transport Layer:** USB 2.0 Full Speed (12 Mbps PHY) Virtual COM Port (115200 bps nominal framing).

### Bill of Materials (BOM)

| Category | Component Description | Manufacturer | Part Number | Operating Parameters | Qty | Designator |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| MCU | 32-bit Cortex-M4 Development Board | WeAct Studio | STM32F411CEU6 | 96 MHz, 3.3 V Logic, USB Type-C | 1 | U1 |
| Driver | 30 A DC Motor Driver | Cytron Technologies | MD30C Rev 2.0 | 5 V–30 V DC, 30 A cont., 20 kHz PWM | 4 | MD1–MD4 |
| IMU | 9-DOF Inertial Measurement Unit | CEVA / Hillcrest Labs | BNO085 | SH-2 Sensor Hub, 400 kHz I2C, 3.3 V | 1 | IMU1 |
| Odometry | Spring-Loaded Tracking Omni-Wheel | Custom RoboManipal | KARMA-DW-35 | Dual-bearing aluminum hub, $R=35\text{ mm}$ | 3 | DW1–DW3 |
| Encoders | Incremental Optical Quadrature Kit | Broadcom / Avago | HEDS-5540-A06 | 1024 CPR (4096 counts/rev X4), 5 V TTL | 3 | ENC1–ENC3 |
| Host | Companion Autonomous Computer | NVIDIA | Jetson Orin | 20–40 TOPS, 6-core ARM, USB 3.2 | 1 | COMP1 |
| Power | Buck Converter Step-Down | Mean Well / Pololu | D24V50F5 | 24 V In $\to$ 5 V Out @ 5 A (Sensors/Logic) | 1 | VR1 |

## Design & Architecture

### Mechanical & Odometry Kinematics
Odometry calculation relies on three unpowered spring-loaded omni-wheels contacting the driving surface independently of drive-wheel traction slip.

```{math}
k_s = \frac{2 \pi R}{\text{CPR}} = \frac{2 \pi \times 0.035\text{ m}}{4096\text{ counts}} \approx 5.371 \times 10^{-5}\text{ m/tick}
```

For discrete sampling interval $[k, k+1]$ with left tracking tick count $\Delta e_{\text{left}}$ and right tracking tick count $\Delta e_{\text{right}}$:

```{math}
\Delta s = \frac{k_s (\Delta e_{\text{left}} + \Delta e_{\text{right}})}{2}
```

```{math}
\Delta \theta = \frac{k_s (\Delta e_{\text{right}} - \Delta e_{\text{left}})}{W}
```

Global position integrates using 2nd-order Runge-Kutta (midpoint arc approximation):

```{math}
x_{k+1} = x_k + \Delta s \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)
```

```{math}
y_{k+1} = y_k + \Delta s \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)
```

```{math}
\theta_{k+1} = \theta_k + \Delta \theta
```

### Firmware Evolution: Bare-Metal Polling vs. Zero-Heap FreeRTOS
The KARMA firmware transitioned through two distinct architectures:

```{mermaid}
graph TD
    subgraph V1_Polling [BaseCode V1.0: Bare-Metal Polling Superloop]
        A1[while 1 Loop] --> B1[BNO085_Update x8 Polling]
        B1 --> C1{I2C Bus Stalled?}
        C1 -- Yes --> D1[Fatal CPU Lockup]
        C1 -- No --> E1[encoder_update: Sample Timers]
        E1 --> F1[snprintf: Serialize JSON Frame]
        F1 --> G1[CDC_Transmit_FS Unsynchronized]
        G1 --> H1{Transmit Busy?}
        H1 -- Yes --> I1[Silent Telemetry Drop]
        H1 -- No --> A1
    end

    subgraph V2_RTOS [BaseCode V2.0: Zero-Heap FreeRTOS + RMS]
        A2[Rate Monotonic Scheduling] --> B2[MotorTask: 100 Hz / 10 ms<br/>150 ms Deadman Watchdog]
        A2 --> C2[AcquisitionTask: 50 Hz / 20 ms<br/>2D State Matrix BNO085]
        A2 --> D2[EstimationTask: 50 Hz / 20 ms<br/>Dead Reckoning Kinematics]
        A2 --> E2[CommunicationTask: Event-Driven<br/>jsmn In-Place Zero-Alloc Parser]
        C2 --> F2{I2C Fault?}
        F2 -- Yes --> G2[9-Clock Bit-Bang Recovery<br/>Actuation Remains 100% Active]
        E2 --> H2[xUsbTxSemaphore Flow Control<br/>Zero Packet Drops]
    end
```

### Architectural Comparison

| Dimension | BaseCode V1.0 (Polling) | BaseCode V2.0 (Zero-Heap FreeRTOS) |
| :--- | :--- | :--- |
| **Execution Paradigm** | Single-threaded sequential superloop (`robot_loop`) | Preemptive Rate Monotonic Scheduling (RMS) across 4 tasks |
| **Memory Management** | Global static buffers; vulnerable to stack overflows | Strict zero-heap static allocation (`configSUPPORT_STATIC_ALLOCATION = 1`) |
| **I2C Fault Handling** | None; slave line lockup stalls entire superloop | 2D State Matrix with automatic 9-clock GPIO bit-bang bus recovery |
| **USB Concurrency** | Unsynchronized `CDC_Transmit_FS()`; frame tearing / drops | Binary semaphore (`xUsbTxSemaphore`) hardware flow control |
| **Safety Watchdog** | Polling-dependent software timestamp check | High-priority 100 Hz `MotorTask` enforcing strict 150 ms deadman cutoff |
| **Timer Counter Rollover** | Manual integer handling | Branchless two's-complement unsigned subtraction: `(int16_t)(curr - prev)` |
| **Parser Overhead** | Repeated `strstr()` and `atoi()` scans | In-place zero-allocation tokenization via `jsmn.h` |

### 2D State Matrix Implementation (`imu_sm.c`)
In BaseCode V2.0, BNO085 communication is isolated into a 2D transition matrix indexed by current state ($S = 7$) and triggering event ($E = 4$):

```{code-block} c
typedef struct {
    imu_state_t     next_state;
    imu_action_fn_t action;
} imu_transition_t;

/* O(1) Transition Lookup Table */
static const imu_transition_t s_transition_table[IMU_STATE_COUNT][IMU_EVT_COUNT] = {
    [IMU_STATE_UNINIT]         = { [IMU_EVT_STEP] = {IMU_STATE_DETECT, action_do_detect}, ... },
    [IMU_STATE_DETECT]         = { [IMU_EVT_SUCCESS] = {IMU_STATE_INIT_DRIVER, action_do_init_driver}, ... },
    [IMU_STATE_INIT_DRIVER]    = { [IMU_EVT_SUCCESS] = {IMU_STATE_CONFIG_REPORTS, action_do_config_reports}, ... },
    [IMU_STATE_CONFIG_REPORTS] = { [IMU_EVT_SUCCESS] = {IMU_STATE_RUNNING, action_do_run}, ... },
    [IMU_STATE_RUNNING]        = { [IMU_EVT_BUS_ERROR] = {IMU_STATE_BUS_RECOVERY, action_do_bus_recovery}, ... },
    [IMU_STATE_BUS_RECOVERY]   = { [IMU_EVT_SUCCESS] = {IMU_STATE_DETECT, action_do_detect},
                                   [IMU_EVT_FAIL]    = {IMU_STATE_ERROR, action_do_error} },
    [IMU_STATE_ERROR]          = { [IMU_EVT_STEP] = {IMU_STATE_BUS_RECOVERY, action_do_bus_recovery}, ... }
};
```

When a bus freeze occurs (e.g. BNO085 clamping SDA low after inductive motor spikes), `action_do_bus_recovery` executes:
1. Reconfigures PA8 (SCL) and PB8 (SDA) as Open-Drain GPIO outputs.
2. Bit-bangs 9 clock pulses on SCL with $5\,\mu\text{s}$ half-period delays to flush the slave shift register.
3. Generates an explicit I2C STOP condition (SDA rising while SCL is high).
4. Reinitializes hardware peripheral I2C3 without blocking task execution.

## Software/Hardware Interface

### Master Hardware Pin & Peripheral Configuration

| Peripheral | STM32 Pin | Mode / AF | Destination / Connected Signal | Voltage Level |
| :--- | :--- | :--- | :--- | :--- |
| TIM5_CH1 | PA0 | AF2 (TIM5) | MD30C Motor 1 PWM Input (20 kHz) | 3.3 V CMOS |
| TIM5_CH2 | PA1 | AF2 (TIM5) | MD30C Motor 2 PWM Input (20 kHz) | 3.3 V CMOS |
| TIM5_CH3 | PA2 | AF2 (TIM5) | MD30C Motor 3 PWM Input (20 kHz) | 3.3 V CMOS |
| TIM5_CH4 | PA3 | AF2 (TIM5) | MD30C Motor 4 PWM Input (20 kHz) | 3.3 V CMOS |
| GPIO_Output | PB0 | Output Push-Pull | MD30C Motor 1 DIR Input | 3.3 V CMOS |
| GPIO_Output | PB1 | Output Push-Pull | MD30C Motor 2 DIR Input | 3.3 V CMOS |
| GPIO_Output | PB2 | Output Push-Pull | MD30C Motor 3 DIR Input (Primary) | 3.3 V CMOS |
| GPIO_Output | PB10 | Output Push-Pull | MD30C Motor 3 DIR Input (V1.1 Redundant A) | 3.3 V CMOS |
| GPIO_Output | PB15 | Output Push-Pull | MD30C Motor 3 DIR Input (V1.1 Redundant B) | 3.3 V CMOS |
| GPIO_Output | PB12 | Output Push-Pull | MD30C Motor 4 DIR Input | 3.3 V CMOS |
| TIM4_CH1 | PB6 | AF2 (TIM4) | Dead-Wheel Encoder 1 Channel A | 3.3 V / 5 V FT |
| TIM4_CH2 | PB7 | AF2 (TIM4) | Dead-Wheel Encoder 1 Channel B | 3.3 V / 5 V FT |
| TIM3_CH1 | PB4 | AF2 (TIM3) | Dead-Wheel Encoder 2 Channel A | 3.3 V / 5 V FT |
| TIM3_CH2 | PB5 | AF2 (TIM3) | Dead-Wheel Encoder 2 Channel B | 3.3 V / 5 V FT |
| TIM2_CH1 | PA15 | AF1 (TIM2) | Dead-Wheel Encoder 3 Channel B | 3.3 V / 5 V FT |
| TIM2_CH2 | PB3 | AF1 (TIM2) | Dead-Wheel Encoder 3 Channel A | 3.3 V / 5 V FT |
| I2C3_SCL | PA8 | AF4 (I2C3) / GPIO | BNO085 SCL (400 kHz Fast-Mode) | 3.3 V OD (PU) |
| I2C3_SDA | PB8 | AF9 (I2C3) / GPIO | BNO085 SDA (400 kHz Fast-Mode) | 3.3 V OD (PU) |
| USB_OTG_FS_DM | PA11 | AF10 (USB_FS) | Jetson Orin Full Speed USB D- | USB PHY |
| USB_OTG_FS_DP | PA12 | AF10 (USB_FS) | Jetson Orin Full Speed USB D+ | USB PHY |

### Serial Framing & Protocol Specifications

#### Inbound JSON Command Payloads (Jetson $\to$ STM32)
* **Four-Wheel Array Format:** `{"m": [250, 250, -250, -250]}` (Values range $-1000$ to $+1000$ representing $-100.0\%$ to $+100.0\%$ duty cycle).
* **Discrete Motor Format:** `{"m1": 200, "m2": 200, "m3": -200, "m4": -200}`.
* **Emergency Halt:** `{"cmd": "stop"}`.

#### Outbound 50 Hz JSON Telemetry Payload (STM32 $\to$ Jetson)
```json
{"yaw": -45.12, "x": 0.02, "y": -0.03, "z": 9.81, "e1": 12450, "e2": 12438, "e3": 15, "bno_ok": 1, "pwm": [250, 250, -250, -250]}
```

### ROS 2 Interface Specifications

| ROS 2 Topic | Message Type | Direction | QoS Profile | Detailed Purpose |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscription | Reliable, Depth=10 | Host motion planner velocity vector ($v_x, v_y, \omega_z$); translated by bridge node into four wheel speeds. |
| `/odom` | `nav_msgs/msg/Odometry` | Publication | Best Effort, Depth=10 | Filtered planar dead-wheel odometry containing pose covariance and twist data. |
| `/imu/data` | `sensor_msgs/msg/Imu` | Publication | Best Effort, Depth=10 | Fused Game Rotation Vector orientation quaternion and triaxial linear acceleration. |
| `/joint_states` | `sensor_msgs/msg/JointState` | Publication | Best Effort, Depth=10 | Unwrapped angular positions and velocities for tracking wheels. |

## Future Work / Known Limitations

### Dormant Sensor Interface (Optical Flow & ToF)
BaseCode V2.0 provides pre-architected dormant hooks for supplementary planar and elevation sensors:
* **Feature Toggle:** Controlled via `#define ENABLE_OPTICAL_FLOW_TOF 0` in `tasks/task_manager.h`.
* **Zero Overhead:** When set to `0`, all corresponding queue slots, data structures, and conditional acquisition code blocks are compiled out.
* **Expansion Procedure:** Set macro to `1`, connect SPI/I2C pins to an optical flow sensor (e.g. PMW3901) and distance sensor (e.g. VL53L1X), and implement low-level register polling in `drivers/optical_flow_tof.c`.

### Known Firmware Limitations
* **Open-Loop Wheel Velocity:** Firmware commands operate on raw PWM duty cycles without closed-loop wheel velocity feedback on the motor shafts. Drive motor encoders must be integrated to form local PI/PID current and velocity loops.
* **UART/USB Physical Connection:** The USB CDC Virtual COM Port cable is vulnerable to high-vibration decoupling during arena transit. Future hardware will implement a galvanically isolated CAN 2.0B / CAN-FD transceiver bus between the STM32 base and the Jetson Orin.
* **Dynamic Covariance Tuning:** Fixed odometry covariance matrices in the ROS 2 bridge must be upgraded to dynamically expand based on detected dead-wheel slippage and ground contact load.
