# Autonomous Balancing Bicycle Control System

## Project Overview

This project documents the development of a control system to achieve dynamic self-balancing on a two-wheeled bicycle. The system is built upon a high-performance STM32H7 series microcontroller and integrates an Inertial Measurement Unit (IMU), a high-torque steering servo motor, a brushless drive motor, and a magnetic encoder. By leveraging precise attitude estimation and an advanced control algorithm, the system maintains the bicycle's balance by actively manipulating the steering assembly while maintaining a constant forward velocity.

The core control strategy is a practical implementation and validation of the **PID + Feed-forward (PID+FF)** method proposed in the research paper **"Steering Control for Autonomously Balancing Bicycle at Low Speed" (Yu & Zhao, 2018).

## Hardware Components

| Component | Model/Type | Interface/Driver | Role |
| :--- | :--- | :--- | :--- |
| **Microcontroller Unit (MCU)** | STM32H723VGTX | - | Core computation and control |
| **Attitude Sensor (IMU)** | Bosch BMI088 | SPI | Provides real-time Roll, Pitch, Yaw angles and angular rates |
| **Steering Motor** | Damiao DM8009 | FDCAN | Executes steering commands from the balance control algorithm |
| **Drive Motor** | XAG A25 | PWM | Provides forward propulsion for the bicycle |
| **Drive ESC** | Hobbywing XRotor HV80A | PWM (50Hz) | Drives the A25 motor |
| **Speed Sensor** | Magnetic Encoder | FDCAN | Measures drive motor speed for closed-loop control |
| **Remote Controller** | AT9S Pro | UART (SBUS) | Provides manual control inputs and system enable switch |
| **Debug Probe** | ATK HSWLDBG | SWD | Firmware flashing and wireless debugging |
| **Communication Bus** | FDCAN x2 | - | Connects the DM8009 motor and the magnetic encoder |

## Software Architecture & Modules

This system is developed as a **bare-metal** application, without dependency on a Real-Time Operating System (RTOS). All task scheduling is managed with precision by hardware timer interrupts to ensure deterministic, real-time performance. Peripheral initialization is configured using STM32CubeMX.

- **`main.c`**: The main application entry point, responsible for calling initialization routines and handling low-frequency tasks (e.g., status printing, remote control parsing) in the main loop.
- **`bsp_*` Modules**: A Board Support Package (BSP) layer that encapsulates low-level HAL library calls for peripherals like `FDCAN`, `TIM`, `SPI`, `UART`, and `GPIO`.
- **`INS` Module**: The Inertial Navigation System module.
    - Drives the `BMI088` and reads raw sensor data.
    - Implements a **Quaternion-based Extended Kalman Filter (EKF)** to fuse accelerometer and gyroscope data for accurate attitude estimation.
    - Features a non-blocking, state-machine-driven **gyroscope bias calibration** routine, which includes a **preheating stage** to mitigate thermal drift.
- **`encoder` Module**: Parses CAN messages from the external magnetic encoder to calculate the motor's multi-turn cumulative angle and real-time angular velocity.
- **`remote_control` Module**: Parses the SBUS protocol from the remote controller. It maps stick inputs to system control commands (e.g., target speed, target lean angle, system enable) and applies a **Low-Pass Filter (LPF)** to smooth manual steering inputs.
- [cite_start]**`balance_controller_yu` Module**: The core balancing algorithm module, which faithfully replicates the **PID+FF controller** from the referenced research paper[cite: 252].
- **`motor_control` / `pid` Modules**: Contains the logic for driving the motors, including a PID controller for the closed-loop speed control of the drive motor.

## Core Features & Algorithms

### 1. Attitude Estimation and Calibration

- **High-Frequency Updates**: The `INS_Update()` function is called from a `TIM4` hardware timer interrupt at a frequency of **1kHz**, guaranteeing real-time attitude data.
- **Sensor Fusion**: The EKF outputs stable Roll, Pitch, and Yaw angles in both radians and degrees.
- **Thermal Stabilization**: An integrated PID temperature controller heats the BMI088 and maintains it at a stable 40°C to minimize thermal drift effects on the gyroscope's bias.
- **Non-Blocking Calibration**: A state machine manages a scientific calibration process: first, the sensor is preheated to a stable temperature, after which gyroscope bias sampling begins. This entire process runs in the background within an ISR without blocking the main program.

### 2. Drive System Control (XAG A25 Motor)

- **PWM Driver**: Utilizes the `TIM1` peripheral to generate a **50Hz** PWM signal with a pulse width modulated between **1200µs and 2300µs** to control the Hobbywing ESC.
- **Closed-Loop Speed Control**:
    - **Setpoint**: The target speed is received from the remote controller and is processed by a **trapezoidal acceleration profile** to ensure smooth changes in velocity.
    - **Feedback**: The **magnetic encoder** provides high-accuracy, real-time feedback of the motor's actual speed.
    - **Controller**: A PID closed-loop controller, running in a `TIM2` timer interrupt, dynamically adjusts the PWM pulse width sent to the ESC to match the actual speed with the target speed, effectively resisting load disturbances.

### 3. Steering & Self-Balancing Control

- [cite_start]**Control Algorithm**: Implements the PID + Feed-forward controller as described by Yu & Zhao (2018)[cite: 242, 252].
    - [cite_start]**Inputs**: The desired roll angle `φd` (from the remote controller), the actual roll angle `φ` (from the IMU), and the actual roll rate `φ_dot` (from the IMU). [cite: 220, 252]
    - [cite_start]**Output**: The calculated desired steering angle `δd` in radians. [cite: 220]
- **Execution**: The balancing controller is executed within the 1kHz `TIM4` interrupt, immediately following the `INS_Update()` call to ensure it operates on the latest available attitude data.
- **Output Smoothing**: The final steering command is passed through a **Slew Rate Limiter**, which constrains the maximum angular velocity of the handlebar. This effectively smooths the control output, eliminating jitter and resulting in more stable and fluid balancing motions.
- **Actuator**: The calculated steering command `δd` is sent via the FDCAN2 bus to the DM8009 servo motor for precise and powerful steering execution.

## How to Build and Run

1.  Open the `.ioc` file in STM32CubeMX to view the detailed peripheral configurations.
2.  Open the project in a compatible IDE such as CLion or STM32CubeIDE.
3.  Compile the project and flash the firmware to the STM32H723VGTX development board.
4.  Connect all hardware components (motors, sensors, RC receiver, etc.).
5.  **First-Time Run / Calibration**:
    -   Enable the `IMU_CALIBRATION_ENABLE` macro in the `INS.c` file.
    -   Power on the board and place the bicycle in a stationary, level position.
    -   Monitor the status via a serial terminal to observe the IMU preheating and gyroscope calibration process.
    -   Once calibration is complete, copy the stable bias offset values printed to the terminal.
    -   Hard-code these values into the `#else` block in `INS.c` and disable the `IMU_CALIBRATION_ENABLE` macro for subsequent runs.
6.  **Normal Operation**:
    -   Turn on the remote controller.
    -   Use the designated two-position switch to enable the control system.
    -   Slowly increase the throttle command; the bicycle will move forward at the set speed.
    -   The balancing controller will now be active, automatically steering to maintain the bicycle's vertical balance.

## Future Work

- [ ] Implement a feature to save the calibrated IMU bias and encoder zero-point values to the MCU's non-volatile memory (Flash or EEPROM) to eliminate the need for re-calibration on every startup.
- [ ] Optimize the PID+FF and drive motor PID gains to achieve robust performance across a wider range of speeds and load conditions.
- [ ] Integrate a GPS or other localization system to enable path planning and full autonomous navigation.

## References
1. Y. Yu and M. Zhao, "Steering Control for Autonomously Balancing Bicycle at Low Speed," 2018 IEEE International Conference on Robotics and Biomimetics (ROBIO), Kuala Lumpur, Malaysia, 2018, pp. 33-38, doi: 10.1109/ROBIO.2018.8665347. 
