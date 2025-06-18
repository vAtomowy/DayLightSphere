# Day Light Sphere – Edge Device

**Day Light Sphere** is an IoT project focused on precise color temperature control using LED technology. The system aims to simulate natural daylight cycles by dynamically adjusting the light output based on configurable parameters.

This repository contains the **edge device code**, which runs on an **ESP32-C3** microcontroller. The device is responsible for controlling the hardware and communicating with the backend system. For development purposes, the backend is currently hosted on a **Raspberry Pi** within a local network.

## Key Features

- High-precision LED color temperature control
- Real-time communication with backend services
- Modular hardware design supporting:
  - I2C sensors
  - Current measurement
  - GPIO peripherals
  - Real-Time Clock (RTC)

## Project Architecture

The full Day Light Sphere project consists of the following components:

- **Edge** (this repository): Firmware for the physical device (ESP32-C3)
- **Backend**: Manages data processing, device configuration, and API communication
- **Frontend**: User interface for real-time control and monitoring

## Current Status

The photo below shows the **first working prototype**, in which the following features have been successfully tested:

- I2C communication
- Current sensing
- Communication with the backend
- GPIO peripheral integration
- Real-Time Clock (RTC) functionality

> *More updates and documentation will follow as development progresses.*

---

Feel free to contribute or raise an issue if you'd like to collaborate!
