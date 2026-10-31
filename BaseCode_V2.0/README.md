# Electronics Subsystem: BaseCode V2.0 Real-Time Firmware Documentation

## Overview & Objectives
BaseCode V2.0 transitions the KARMA 4-wheel mobile base firmware for RoboManipal's RoboCup SML/@work 2026 robot from a bare-metal polling superloop to a **strict zero-heap FreeRTOS** multi-tasking architecture. Operating on the STM32F411CEU6 MCU, V2.0 addresses timing jitter, I2C slave lockups, USB transmit buffer collisions, and watchdog starvation by implementing Rate Monotonic Scheduling (RMS) across isolated tasks, backed by a deterministic 2D State Matrix for sensor fault isolation.

Core engineering objectives:
* Eliminate dynamic memory allocation (`malloc`, `free`, `heap_x.c`) to ensure zero heap fragmentation during continuous multi-hour competition runs.
* Isolate motor actuation and implement a hard real-time 150 ms deadman safety watchdog running at 100 Hz preemptive priority.
* Formulate a 2D State Matrix transition table managing BNO085 IMU lifecycle, packet draining, report generation, and automatic 9-clock I2C bus recovery without stalling actuation or telemetry.
* Establish hardware-synchronized USB CDC telemetry at 50 Hz using binary semaphore flow control (`xUsbTxSemaphore`) to eradicate `USBD_BUSY` drops and JSON corruption.
* Perform high-precision dead-reckoning odometry ($R = 35.0\text{ mm}, W = 150.0\text{ mm}$) with branchless 16-bit timer rollover handling.

## Technical Specifications & Bill of Materials (BOM)

### System Specifications
* **Microcontroller:** STM32F411CEU6 (WeAct Black Pill), ARM Cortex-M4 @ 96 MHz, 512 KB Flash, 128 KB SRAM.
* **RTOS Kernel:** FreeRTOS v202012.00 (Zero-Heap, `configSUPPORT_STATIC_ALLOCATION = 1`).
* **Scheduler Tick Rate:** 1000 Hz ($1\text{ ms}$ timebase via `TIM1` / `SysTick`).
* **Task Scheduling Policy:** Preemptive Priority-Based Rate Monotonic Scheduling (RMS).
* **Motor PWM Frequency:** 20 kHz (TIM5, ARR = 4799 @ 96 MHz APB1 timer clock).
* **Safety Watchdog Threshold:** 150 ms strict deadman cutoff.
* **IMU Interface:** I2C3 @ 400 kHz Fast-Mode with GPIO bit-banged 9-clock recovery.
* **Odometry Tracking Geometry:** Dead-wheel radius $R = 35.0\text{ mm}$, Tracking baseline width $W = 150.0\text{ mm}$.

### Bill of Materials (BOM)

| Component | Part Number / Model | Manufacturer | Key Specification | Quantity | Reference Designator |
| :--- | :--- | :--- | :--- | :--- | :--- |
| MCU Module | STM32F411CEU6 | WeAct Studio | ARM Cortex-M4F, 96 MHz, 512 KB Flash, 128 KB SRAM | 1 | U1 |
| Motor Drivers | MD30C Rev 2.0 | Cytron Technologies | 5 V–30 V DC, 30 A continuous, 80 A peak, 20 kHz PWM | 4 | MD1–MD4 |
| IMU Sensor | BNO085 | CEVA / Hillcrest Labs | 9-DOF Triaxial Gyro/Accel/Mag with SH-2 Firmware | 1 | IMU1 |
| Dead Wheels | Omni Wheel Assembly | Custom RoboManipal | Spring-loaded unpowered tracking assembly ($R=35\text{ mm}$) | 3 | DW1–DW3 |
| Encoders | Quadrature Optical | Broadcom / Avago | 1024 CPR (4096 counts/rev X4 decoding) | 3 | ENC1–ENC3 |
| Companion PC | Jetson Orin Nano / NX | NVIDIA | 6-core/8-core ARM, 20–40 TOPS, ROS 2 Humble | 1 | COMP1 |

## Design & Architecture

### Real-Time Task Architecture
The firmware separates responsibilities into four statically allocated FreeRTOS tasks configured under Rate Monotonic Scheduling:

```{mermaid}
graph TD
    subgraph Host [Jetson Orin / PC]
        A[ROS 2 Base Node]
    end

    subgraph USB_Layer [USB CDC Interface]
        B[USB RX ISR]
        C[q_usb_rx Queue: 256B]
        D[xUsbTxSemaphore: Binary Semaphore]
    end

    subgraph Tasks [FreeRTOS Preemptive Tasks]
        E["MotorTask (Pri 4, 100 Hz / 10 ms)<br/>• 150 ms Deadman Watchdog<br/>• TIM5 PWM & GPIOB DIR"]
        F["AcquisitionTask (Pri 3, 50 Hz / 20 ms)<br/>• 2D State Matrix BNO085 IMU<br/>• 3-Ch Hardware Encoder Rollover"]
        G["EstimationTask (Pri 2, 50 Hz / 20 ms)<br/>• Midpoint Arc Dead Reckoning<br/>• Global Robot State Mutex Snapshot"]
        H["CommunicationTask (Pri 1, Event Driven)<br/>• jsmn Zero-Alloc In-Place Parser<br/>• 50 Hz Formatted JSON Telemetry"]
    end

    A -- "JSON Commands" --> B
    B --> C
    C --> H
    H -->|q_motor_cmd| E
    F -->|q_sensor_data| G
    G -->|xRobotStateMutex| H
    H -- "JSON Telemetry" --> D
    D --> A
```

### Static Task Characteristics

| Task Identifier | Priority | Period / Rate | Stack Size | Core Responsibilities |
| :--- | :--- | :--- | :--- | :--- |
| `MotorTask` | 4 (`tskIDLE_PRIORITY + 4`) | 10 ms (100 Hz) | 256 words (1024 B) | 150 ms deadman watchdog evaluation, consuming `q_motor_cmd`, updating TIM5 CCR1–CCR4 registers and GPIOB direction pins. |
| `AcquisitionTask` | 3 (`tskIDLE_PRIORITY + 3`) | 20 ms (50 Hz) | 384 words (1536 B) | Dispatching IMU 2D State Matrix events, sampling 16-bit encoder registers (TIM4, TIM3, TIM2), packaging `sensor_data_t`. |
| `EstimationTask` | 2 (`tskIDLE_PRIORITY + 2`) | 20 ms (50 Hz) | 384 words (1536 B) | Consuming `q_sensor_data`, evaluating Runge-Kutta 2nd-order dead-reckoning displacement, writing mutex-protected `robot_state_t`. |
| `CommunicationTask` | 1 (`tskIDLE_PRIORITY + 1`) | Event-driven / 20 ms | 512 words (2048 B) | Consuming `q_usb_rx`, in-place JSON parsing via `jsmn`, populating `s_tx_buf`, blocking on `xUsbTxSemaphore` for hardware-synchronized transmission. |
| `LoggerTimer` | Timer Daemon | 500 ms (2 Hz) | 128 words (512 B) | Mutex-protected flushing of static ring buffer debug diagnostics. |

### 2D State Matrix for IMU Lifecycle & Fault Recovery
To prevent sensor communication anomalies from stalling system tasks, the BNO085 driver is governed by a formal 2D state transition table (`imu_sm.c`).

#### State & Event Definitions
* **States ($S = 7$):** `IMU_STATE_UNINIT`, `IMU_STATE_DETECT`, `IMU_STATE_INIT_DRIVER`, `IMU_STATE_CONFIG_REPORTS`, `IMU_STATE_RUNNING`, `IMU_STATE_BUS_RECOVERY`, `IMU_STATE_ERROR`.
* **Events ($E = 4$):** `IMU_EVT_STEP`, `IMU_EVT_SUCCESS`, `IMU_EVT_FAIL`, `IMU_EVT_BUS_ERROR`.

#### Transition Matrix Implementation

```{code-block} c
static const imu_transition_t s_transition_table[IMU_STATE_COUNT][IMU_EVT_COUNT] = {
    [IMU_STATE_UNINIT] = {
        [IMU_EVT_STEP]      = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_DETECT] = {
        [IMU_EVT_STEP]      = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_INIT_DRIVER,   action_do_init_driver },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_INIT_DRIVER] = {
        [IMU_EVT_STEP]      = { IMU_STATE_INIT_DRIVER,   action_do_init_driver },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_CONFIG_REPORTS,action_do_config_reports },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_CONFIG_REPORTS] = {
        [IMU_EVT_STEP]      = { IMU_STATE_CONFIG_REPORTS,action_do_config_reports },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_RUNNING,       action_do_run },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_RUNNING] = {
        [IMU_EVT_STEP]      = { IMU_STATE_RUNNING,       action_do_run },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_RUNNING,       action_none },
        [IMU_EVT_FAIL]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery }
    },
    [IMU_STATE_BUS_RECOVERY] = {
        [IMU_EVT_STEP]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_DETECT,        action_do_detect },
        [IMU_EVT_FAIL]      = { IMU_STATE_ERROR,         action_do_error },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_ERROR,         action_do_error }
    },
    [IMU_STATE_ERROR] = {
        [IMU_EVT_STEP]      = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_SUCCESS]   = { IMU_STATE_BUS_RECOVERY,  action_do_bus_recovery },
        [IMU_EVT_FAIL]      = { IMU_STATE_ERROR,         action_none },
        [IMU_EVT_BUS_ERROR] = { IMU_STATE_ERROR,         action_none }
    }
};
```

#### How the State Machine Solves Failure Modes
* **Deterministic $O(1)$ Dispatch:** Replacing monolithic nested switch-case blocks with table lookups guarantees zero branching latency.
* **Decoupled Bus Unlocking:** If motor inductive kickback forces SDA low mid-transaction, `action_do_run` detects repeated communication errors, generates `IMU_EVT_BUS_ERROR`, and executes `action_do_bus_recovery`.
* **Automatic 9-Clock Recovery:** `i2c_recovery.c` configures PA8 (SCL) and PB8 (SDA) as GPIO outputs, pulses SCL 9 times to flush the slave internal shift register, generates a manual STOP condition, and re-enables hardware I2C3. Actuation and dead-wheel odometry remain 100% active during the recovery cycle.

### Odometry & Unsigned Rollover Mathematics
Hardware timers TIM4, TIM3, and TIM2 operate in 16-bit quadrature counter mode (0 to 65535). Delta calculation utilizes two's complement unsigned integer underflow/overflow properties:

```{code-block} c
/* Unsigned subtraction cast directly to signed 16-bit integer */
int16_t delta_ticks = (int16_t)(current_raw - previous_raw);
```

When counter rolls over from `65535` to `0` (forward rotation):
```{math}
\text{delta} = (int16\_t)(0 - 65535) = (int16\_t)(1) = +1\text{ tick}
```
When counter rolls under from `0` to `65535` (reverse rotation):
```{math}
\text{delta} = (int16\_t)(65535 - 0) = (int16\_t)(-1) = -1\text{ tick}
```
This guarantees exact velocity tracking across timer boundaries without conditional checking or interrupt branching.

### Kinematics Formulations
Kinematic tracking utilizes the three dead wheels:

```{math}
\Delta s_{\text{left}} = \text{delta\_left} \times \frac{2 \pi R}{\text{CPR}}
```

```{math}
\Delta s_{\text{right}} = \text{delta\_right} \times \frac{2 \pi R}{\text{CPR}}
```

Midpoint arc integration:
```{math}
\Delta s = \frac{\Delta s_{\text{left}} + \Delta s_{\text{right}}}{2}, \quad \Delta \theta = \frac{\Delta s_{\text{right}} - \Delta s_{\text{left}}}{W}
```

Pose update equations:
```{math}
x_{k+1} = x_k + \Delta s \cos\left(\theta_k + \frac{\Delta \theta}{2}\right)
```

```{math}
y_{k+1} = y_k + \Delta s \sin\left(\theta_k + \frac{\Delta \theta}{2}\right)
```

```{math}
\theta_{k+1} = \theta_k + \Delta \theta
```

## Software/Hardware Interface

### STM32 Hardware Pin & Channel Mapping

| Subsystem | Peripheral | Pin | Mode / AF | Connected Device / Function |
| :--- | :--- | :--- | :--- | :--- |
| Actuation | TIM5_CH1 | PA0 | AF2 (TIM5) | MD30C Motor 1 PWM (20 kHz) |
| Actuation | TIM5_CH2 | PA1 | AF2 (TIM5) | MD30C Motor 2 PWM (20 kHz) |
| Actuation | TIM5_CH3 | PA2 | AF2 (TIM5) | MD30C Motor 3 PWM (20 kHz) |
| Actuation | TIM5_CH4 | PA3 | AF2 (TIM5) | MD30C Motor 4 PWM (20 kHz) |
| Actuation | GPIO Out | PB0 | Output Push-Pull | MD30C Motor 1 DIR |
| Actuation | GPIO Out | PB1 | Output Push-Pull | MD30C Motor 2 DIR |
| Actuation | GPIO Out | PB2 | Output Push-Pull | MD30C Motor 3 DIR |
| Actuation | GPIO Out | PB12 | Output Push-Pull | MD30C Motor 4 DIR |
| Odometry | TIM4_CH1 | PB6 | AF2 (TIM4) | Dead-Wheel Encoder 1 Channel A |
| Odometry | TIM4_CH2 | PB7 | AF2 (TIM4) | Dead-Wheel Encoder 1 Channel B |
| Odometry | TIM3_CH1 | PB4 | AF2 (TIM3) | Dead-Wheel Encoder 2 Channel A |
| Odometry | TIM3_CH2 | PB5 | AF2 (TIM3) | Dead-Wheel Encoder 2 Channel B |
| Odometry | TIM2_CH1 | PA15 | AF1 (TIM2) | Dead-Wheel Encoder 3 Channel B |
| Odometry | TIM2_CH2 | PB3 | AF1 (TIM2) | Dead-Wheel Encoder 3 Channel A |
| IMU / Sensor | I2C3_SCL | PA8 | AF4 / GPIO Bit-Bang | BNO085 SCL (400 kHz / 9-Clock Recovery) |
| IMU / Sensor | I2C3_SDA | PB8 | AF9 / GPIO Bit-Bang | BNO085 SDA (400 kHz / Manual STOP) |
| USB Port | OTG_FS_DM | PA11 | AF10 (USB_FS) | Jetson Orin Full Speed USB D- |
| USB Port | OTG_FS_DP | PA12 | AF10 (USB_FS) | Jetson Orin Full Speed USB D+ |

### Inter-Task Synchronization Primitives

| Object Name | FreeRTOS Primitive Type | Buffer Capacity / Item Size | Synchronization Purpose |
| :--- | :--- | :--- | :--- |
| `q_usb_rx` | Static Queue | 256 bytes | Decouples USB RX ISR from ASCII JSON string extraction. |
| `q_motor_cmd` | Static Queue | 4 items (`motor_cmd_t`) | Passes validated motor setpoints from CommunicationTask to MotorTask. |
| `q_sensor_data` | Static Queue | 4 items (`sensor_data_t`) | Carries raw synchronized encoder counts and IMU data to EstimationTask. |
| `xUsbTxSemaphore` | Static Binary Semaphore | 1 token | Locks CommunicationTask until `CDC_TransmitCplt_FS()` fires, preventing buffer overwrites. |
| `xRobotStateMutex` | Static Mutex | 1 token (Priority Inheritance) | Protects shared global `robot_state_t` snapshot read by CommunicationTask and written by EstimationTask. |

### Serial Frame Protocols

#### Jetson Command Format (Host $\to$ Robot)
```json
{"m": [250, 250, -250, -250]}
```

#### Telemetry Frame Format (Robot $\to$ Host @ 50 Hz)
```json
{"yaw": -12.45, "x": 0.01, "y": 0.00, "z": 9.80, "e1": 1024, "e2": 1024, "e3": 0, "bno_ok": 1, "pwm": [250, 250, -250, -250]}
```

### ROS 2 Interface Specification

| ROS 2 Topic | Message Type | QoS Profile | Rate | Bridge Responsibility |
| :--- | :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Reliable / Volatile | Up to 100 Hz | Converted by Python bridge node into four wheel velocity commands and transmitted over USB CDC. |
| `/odom` | `nav_msgs/msg/Odometry` | SensorData / Best Effort | 50 Hz | Populated from estimated $(x, y, \theta)$ and linear/angular velocities for Nav2 path tracking. |
| `/imu/data` | `sensor_msgs/msg/Imu` | SensorData / Best Effort | 50 Hz | Converted Game Rotation Vector quaternion and linear acceleration from BNO085. |
| `/joint_states` | `sensor_msgs/msg/JointState` | SensorData / Best Effort | 50 Hz | Wheel rotational position and velocity tracking. |

## Future Work / Known Limitations

### Dormant Sensor Expansion (Optical Flow & ToF)
The architecture includes dormant hooks for secondary localization sensors:
* **Flag Configuration:** Controlled via `#define ENABLE_OPTICAL_FLOW_TOF 0` in `task_manager.h`. When set to `0`, all structures, queues, and task logic compile out with zero RAM and zero CPU overhead.
* **Activation Path:** Set flag to `1`, connect SPI/I2C pins to an optical flow sensor (e.g. PMW3901) and Time-of-Flight ranging sensor (e.g. VL53L1X), and populate read logic inside `drivers/optical_flow_tof.c`.

### Known Limitations & Planned Enhancements
* **Open-Loop PWM Control:** Motor commands currently apply open-loop PWM duty cycles. Closed-loop wheel velocity PID controllers must be implemented within `MotorTask` using motor feedback encoders.
* **Covariance Matrix Tuning:** In the ROS 2 bridge node, odometry covariance matrices in `/odom` and `/imu/data` are currently fixed constants; dynamic calculation based on dead-wheel ground contact metrics is required.
* **CAN-FD Migration:** For future iterations with higher motor count and robotic manipulator integration, the USB CDC serial link will be transitioned to an onboard CAN 2.0B / CAN-FD bus to provide hardware-level deterministic bus arbitration.
