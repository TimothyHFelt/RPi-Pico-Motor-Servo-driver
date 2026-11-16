# 🛡️ RPI Surveillance Tank Firmware

A robust, multi-threaded FreeRTOS firmware built for the **Raspberry Pi Pico W (RP2040)** in **C**. This system powers a custom surveillance tank, managing real-time DC motor actuation via hardware PWM, dual-servo gimbal control through an **I2C PCA9685 driver**, high-speed UART command streaming, live battery voltage compensation, and a comprehensive multi-layered safety framework.

---

## 📋 Project Metadata

* **Author:** Timothy Felt
* **Project Name:** RPI Surveillance Tank
* **Target Hardware:** Raspberry Pi Pico W (RP2040)
* **Language:** C
* **Core Libraries & Frameworks:** FreeRTOS, PCA9685 Driver, CommsI2C, RP2040 Hardware APIs (`pwm`, `adc`, `i2c`, `uart`, `watchdog`)

---

## 🚀 Key Features

* **Multi-Tasking RTOS Architecture:** Decouples system responsibilities into dedicated, prioritized FreeRTOS tasks to ensure zero command lag and reliable real-time execution.
* **Dynamic Voltage Compensation:** Automatically scales DC motor PWM thresholds (`motorValue`) against live battery voltage readings, ensuring consistent driving performance and speed as the battery discharges.
* **Fail-Safe Watchdog & Heartbeat Systems:**
  * **Hardware Watchdog:** 2-second timeout triggering an automatic system reset if execution locks up.
  * **UART Communication Watchdog:** Enforces a strict 2-second timeout (`UART_TIMEOUT_MS`) that immediately executes an emergency stop (`StopAllMotors()`) if control packets stop arriving.
* **Thread-Safe State Management:** Utilizes FreeRTOS mutexes (`xSafeMutex`) to safely share safety status flags between high-priority interrupt/monitoring tasks and actuation loops.

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
2. **Payload:** 8-byte array representing direction and speed indices for motors and servos (queued via `xMotorQueue`).

---

## 🛡️ Safety & Battery Thresholds

* **Healthy State:** Battery voltage $> 13.0\text{V}$ enables full motor operations (`setSafety(true)`).
* **Low Battery Warning:** Voltage $< 13.0\text{V}$ flags low power warnings to telemetry/OSD.
* **Emergency Cutoff:** Voltage $< 12.2\text{V}$ immediately trips safety (`setSafety(false)`), clears CC registers to `0`, and halts all motion to protect system hardware.

---

## 📜 Credits & Software Copyright

* **PCA9685.h:** Pulled from the *Adafruit-PWM-Servo-Driver-Library*.
* **CommsI2C.h:** Pulled from Adafruit; modified for the RPI Pico W (RP2040).

### Software License Agreement (BSD License)
```text
Copyright (c) 2012, Adafruit Industries All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
- Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
