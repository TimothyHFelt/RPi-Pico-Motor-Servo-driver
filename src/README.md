# RP2040 FreeRTOS Actuator & Power Management Firmware

A robust, multi-threaded embedded firmware built for the **Raspberry Pi Pico / Pico W (RP2040)** using **FreeRTOS**. This system handles real-time DC motor PWM actuation, dual-servo gimbal control via an **I2C PCA9685 driver**, high-speed UART packet parsing, battery voltage compensation, and comprehensive hardware/software safety watchdogs.

---

## 🚀 Key Features

* **Multi-Tasking Architecture (FreeRTOS):** Separates core responsibilities into dedicated prioritized tasks:
  * **UART Task (`UartMessage`):** High-speed packet synchronization and queue offloading.
  * **Motor Control Task (`MotorControl`):** Manages RP2040 hardware PWM slices for DC motors, updates PCA9685 servos, and applies voltage compensation.
  * **Voltage Monitor Task (`voltCheck`):** Continuously samples battery health via ADC, handles brownout protection, and triggers emergency safety states.
* **Dynamic Voltage Compensation:** Automatically scales DC motor PWM thresholds (`motorValue`) based on live battery voltage readings to ensure consistent physical performance as the battery discharges.
* **Fail-Safe Watchdog & Heartbeat Systems:**
  * **Hardware Watchdog:** 2-second timeout ensuring automatic system reset if execution locks up.
  * **UART Communication Watchdog:** Implements a 2-second timeout (`UART_TIMEOUT_MS`) that immediately triggers an emergency stop (`StopAllMotors()`) if control packets stop arriving.
* **Thread-Safe State Management:** Utilizes FreeRTOS mutexes (`xSafeMutex`) to safely share safety status flags across tasks.

---

## 📌 Hardware Pinout & Configuration

| Peripherals / Interface | RP2040 Pins / Channels | Description |
| :--- | :--- | :--- |
| **I2C0 (PCA9685 Servo Driver)** | SDA: **GPIO 16**, SCL: **GPIO 17** | Communicates with the 16-channel PCA9685 PWM driver at 400kHz. |
| **UART1 (Command Stream)** | TX: **GPIO 4**, RX: **GPIO 5** | Receives 8-byte motor/servo control packets at 115200 baud. |
| **Hardware PWM Slice 2** | **GPIO 20, 21** | Right DC Motor actuation channel. |
| **Hardware PWM Slice 7** | **GPIO 14, 15** | Left DC Motor actuation channel. |
| **ADC (Battery Monitor)** | **GPIO 26** (ADC Input 0) | Analog voltage sampling line with a 6.05x hardware divider multiplier. |

---

## 🛠️ Software Architecture & Task Priorities

| Task Name | Function | Priority | Stack Size | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Voltage Handler** | `voltCheck` | `tskIDLE_PRIORITY + 3` (Highest) | 2048 bytes | Samples ADC every 100ms, manages safety cuts (`< 12.2V`) and recovery (`> 13.0V`). |
| **Uart Handler** | `UartMessage` | `tskIDLE_PRIORITY + 2` (Medium) | Minimal | Parses incoming UART1 stream for start-of-frame (`0xFF`) and pushes payloads to queue. |
| **Motor Handler** | `MotorControl` | `tskIDLE_PRIORITY + 1` (Lowest) | 2048 bytes | Drains motor queue, drives hardware PWM slices, and invokes servo handlers every 10ms. |

---

## 📦 Communication Protocol

The firmware expects a fixed-size binary packet stream over UART1:
1. **Start-of-Frame (SOF):** `0xFF` byte marker.
2. **Payload:** 8-byte array representing direction and speed indices for motors and servos (stored in the global `myMotors` array via FreeRTOS queue communication).

---

## 🛡️ Safety & Battery Thresholds

* **Healthy State:** Battery voltage $> 13.0\text{V}$ enables full motor operations (`setSafety(true)`).
* **Low Battery Warning:** Voltage $< 13.0\text{V}$ flags low power to connected telemetry/OSD.
* **Emergency Cutoff:** Voltage $< 12.2\text{V}$ immediately trips safety (`setSafety(false)`), cuts CC registers to `0`, and halts all motion to protect hardware.

---

## 📜 Credits & License

* **PCA9685.h:** Adapted from the *Adafruit-PWM-Servo-Driver-Library*.
* **CommsI2C.h:** Pulled from Adafruit and modified for the Raspberry Pi Pico W (RP2040).
* **License:** Released under the **BSD License** (Copyright (c) 2012, Adafruit Industries). See source file headers for full redistribution terms.
