
# DriveSight

A mobile robot project with a camera for visual tracking, based on Raspberry Pi Pico 2W and ESP32-S3.

## Description

DriveSight is a manually controlled robotic platform equipped with real-time video streaming. It integrates a camera module for live feed monitoring and a wireless controller interface for direct motor control, providing precise, remote navigation and situational awareness.

## Motivation

I started this project to challenge myself and explore how far I could push the capabilities of embedded hardware, combining real-time control, wireless communication, and camera streaming on constrained systems.

## Architecture

**Main Components:**
- **Raspberry Pi Pico 2W (Motor Controller):** Controls motors, ultrasonic sensors, and receives movement commands via UART.
- **ESP32-S3:** Captures a live camera feed and sends control commands from a connected Bluetooth controller.
- **Raspberry Pi Pico 2W (Display Host):** Hosts a Wi-Fi access point and receives data from the motor controller to display real-time sensor values or system status on a small screen.

**Feedback Mechanisms:**
- **Current sensors (INA219):** Monitor motor current to help maintain consistent speed across varying surface conditions.
- **Ultrasonic sensors:** Provide obstacle detection and distance feedback for navigation and collision avoidance.

**User Interface:**
- **Web-based interface:** Hosted on the ESP32-S3 for viewing the live camera feed.
- **Bluetooth controller:** Used for manual driving input.
- **On-board display:** Connected to Pico no.2, shows live telemetry from the motor controller over Wi-Fi.

**Connections Between Components:**
- **UART:** Used for communication between the ESP32-S3 and motor controller Pico.
- **Wi-Fi:** Motor controller Pico connects to the display host Pico (AP mode) for data transmission.
- **I2C:** Used by the motor controller Pico for reading data from INA219 current and ultrasonic sensors.
- **GPIO:** Used to control the L298N motor driver from the motor controller Pico.


## Hardware

- Raspberry Pi Pico 2W: Main controller handling sensors and motor control.
- ESP32-S3: Provides camera feed for object tracking and sends control commands.
- Raspberry Pi Pico 2W (Display Unit): Hosts an access point and displays telemetry data.
- ST7735s: Display the telemetry data.
- L298N Motor Driver: Drives the two DC motors.
- 2× DC Motors: Provide movement for the robot.
- 2× INA219 Current Sensors: Monitor motor currents.
- 3× HC-SR04 Ultrasonic Sensors: Detect obstacles.
- Power Bank and Li-ion Batteries: Powers the system.
- Wires, Breadboard, and Connectors: For interconnecting all the components.


## Software

### ESP32-S3 (C)

| Library | Description | Usage |
| ------- | ----------- | ----- |
| [ESP-IDF](https://github.com/espressif/esp-idf) | Official Espressif IoT Development Framework | Base SDK for Wi-Fi, networking, etc. |
| [esp_camera](https://github.com/espressif/esp32-camera) | Camera driver for OV3660 | Captures and streams MJPEG frames |
| [Bluepad32](https://github.com/ricardoquesada/bluepad32) | Bluetooth game controller library | Receives input from Bluetooth controller |

### Raspberry Pi Pico 2W (Rust)

| Library | Description | Usage |
| ------- | ----------- | ----- |
| [embassy](https://crates.io/crates/embassy) | Async embedded runtime for embedded systems | Task scheduling and async infrastructure |
| [embassy-executor](https://crates.io/crates/embassy-executor) | Asynchronous task executor | Schedules and runs tasks in a `no_std` async environment |
| [embassy-rp](https://crates.io/crates/embassy-rp) | RP2040-specific HAL for Embassy | GPIO, SPI, PWM, ADC, and other peripherals |
| [embassy-time](https://crates.io/crates/embassy-time) | Timer and delay handling | Non-blocking frame-rate control, delays, timeouts |
| [embassy-sync](https://crates.io/crates/embassy-sync) | Synchronization primitives | Used for async channels and mutexes |
| [embassy-gpio](https://crates.io/crates/embassy-gpio) | GPIO abstraction for Embassy | Manages input/output pins for buttons, sensors, and motor control |
| [embassy-futures](https://crates.io/crates/embassy-futures) | Futures and async utilities for Embassy | Enables combining and managing multiple async tasks and events |
| [cyw43](https://crates.io/crates/cyw43) | Wi-Fi driver for the CYW43 chip | Connecting the Pico 2W to Wi-Fi |
| [cyw43-pio](https://crates.io/crates/cyw43-pio) | PIO-based driver for the CYW43 chip | Enables Wi-Fi using the RP2040's PIO peripheral |
| [embassy-net](https://crates.io/crates/embassy-net) | TCP/IP networking stack | Hosts server or client for sending/receiving data |
| [embedded-hal](https://crates.io/crates/embedded-hal) | Hardware abstraction layer for embedded systems | Standardized traits for peripherals (GPIO, I2C, SPI, etc.) |
| [hcsr04-async](https://crates.io/crates/hcsr04-async) | Async driver for ultrasonic distance sensor | Obstacle detection |
| [defmt](https://crates.io/crates/defmt) | Lightweight logging framework for embedded systems | Enables efficient, structured logging |
| [defmt-rtt](https://crates.io/crates/defmt-rtt) | RTT (Real-Time Transfer) backend for `defmt` | Outputs logs via RTT for real-time debugging |
| [embedded-graphics](https://crates.io/crates/embedded-graphics) | 2D graphics library for embedded systems | Drawing text, shapes, and UI elements on the display |
| [mipidsi](https://crates.io/crates/mipidsi) | Display driver for MIPI-compatible SPI LCDs | Driving the ST7735s display via SPI |
| [rand](https://crates.io/crates/rand) | Random number generation | Used internally by the TCP stack for port/sequence numbers |
| [smoltcp](https://crates.io/crates/smoltcp) | Embedded TCP/IP networking stack | Provides low-level TCP/IP networking for the Pico 2W |
| [static-cell](https://crates.io/crates/static-cell) | Safe static memory cell abstraction | Manages static data for async tasks and peripherals |
| [alloc-cortex-m](https://crates.io/crates/alloc-cortex-m) | Heap allocator for Cortex-M microcontrollers | Enables dynamic memory allocation in embedded Rust |
| [libm](https://crates.io/crates/libm) | Math library for embedded systems | Provides floating-point math functions in `no_std` environments |
| [heapless](https://crates.io/crates/heapless) | Fixed-capacity data structures without heap | Used for queues, buffers, and collections in constrained systems |
| [panic-probe](https://crates.io/crates/panic-probe) | Minimal panic handler for embedded Rust | Reports panics via RTT or serial for debugging |

## Getting Started

Clone the repository:

```sh
git clone https://github.com/mohgTheOmen/drivesight.git
cd drivesight
```