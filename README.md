# Plant Monitoring IoT System

## Overview

This project implements an IoT-based plant monitoring system using the B-L072Z-LRWAN1 ARM mbed-based platform. The system monitors various environmental parameters critical for plant health and provides real-time feedback through visual indicators and data logging.

## Features

- 🌡️ **Temperature Monitoring**: -10°C to 50°C range with Si7021 sensor
- 💧 **Humidity Monitoring**: 25-75% RH range with Si7021 sensor
- ☀️ **Ambient Light Monitoring**: 0% (dark) to 100% (bright) with HW5P-1 sensor
- 🌱 **Soil Moisture Monitoring**: 0% (dry) to 100% (wet) with SEN-13322 sensor
- 🎨 **Leaf Color Detection**: RGB and clear light levels with TCS34725 sensor
- 📱 **Acceleration Monitoring**: X, Y, and Z axes with MMA8451Q accelerometer
- 🌎 **Global Position Tracking**: Location and time via GPS module
- 💡 **Visual Feedback**: RGB LED indicators for plant health status

## Hardware Components

- B-L072Z-LRWAN1 ARM mbed-based platform
- Si7021 Temperature & Humidity Sensor
- HW5P-1 Light Sensor
- SEN-13322 Soil Moisture Sensor
- TCS34725 RGB Color Sensor
- MMA8451Q Accelerometer
- GPS Module (Ultimate GPS)
- RGB LEDs for status indication

## System Architecture

### Hardware Block Diagram

The system uses multiple communication protocols to interface with sensors:

- **I2C Bus**: Connected to PB_8 (SDA) and PB_9 (SCL) for digital sensors (accelerometer, temperature/humidity, color)
- **UART Communication**: Uses PA_9 (TX) and PA_10 (RX) for GPS module communication
- **ADC Inputs**: For analog sensors (light and soil moisture)
- **GPIO Pins**: For LED control and user button input

### Software Organization

The software is organized into multiple threads for efficient operation:

- **I2C Thread**: Manages sensors using the I2C interface
- **IO Thread**: Handles analog sensors and GPS module
- **PWM Thread**: Controls RGB LED for visual feedback
- **Main Thread**: Coordinates thread operation and manages system modes

## Operation Modes

### TEST Mode
- Real-time sensor monitoring every 2 seconds
- RGB LED displays dominant leaf color
- All sensor data displayed on terminal

### NORMAL Mode
- Data collection every 30 seconds
- Hourly summaries calculated
- Violation detection and alerts
- LED indicators for different violation types

### ADVANCED Mode
- PWM-like RGB LED emulation matching detected leaf color
- All NORMAL mode functionality maintained
- Smooth color transitions

## Violation Thresholds

The system monitors for violations of the following thresholds:

- **Temperature**: Out of range (-10.0°C to 30.0°C) → Red LED
- **Humidity**: Out of range (25.0% to 90.0%) → Green LED
- **Light Intensity**: Out of range (0.0% to 80.0%) → Blue LED
- **Soil Moisture**: Out of range (0.0% to 80.0%) → Green and Blue LEDs
- **X-axis Acceleration**: Outside ±2.0 m/s² → Red LED
- **Y-axis Acceleration**: Outside ±2.0 m/s² → Green LED
- **Z-axis Acceleration**: Outside ±11.0 m/s² → Red and Green LEDs

## Installation and Setup

1. Clone the repository:
```bash
git clone https://github.com/Zelawon/Project-Embedded.git
```

2. Connect the hardware components according to the block diagram in the documentation.

3. Import the project into the Mbed development environment.

4. Compile and flash the program to the B-L072Z-LRWAN1 board.

5. Power the system and use the user button to cycle through operating modes.

## Project Structure

```
Project-Embedded/
├── accelerometer/      # Accelerometer sensor implementation
├── colorsensor/        # RGB color sensor implementation
├── gps/                # GPS module implementation
├── humidity/           # Temperature & humidity sensor implementation
├── lightsensor/        # Light sensor implementation
├── moisturesensor/     # Soil moisture sensor implementation
├── pwm/                # PWM thread implementation
├── i2c_thread/         # I2C thread implementation
├── io_thread/          # IO thread implementation
├── main.cpp            # Main application entry point
└── README.md           # Project documentation
```

## Code Metrics

| Module/File | Lines of Code |
|-------------|---------------|
| Accelerometer (.cpp, .h) | ~122 |
| Color Sensor (.cpp, .h) | ~100 |
| Humidity & Temperature Sensor (.cpp, h) | ~82 |
| GPS Module (.cpp, .h) | ~171 |
| Light Sensor (.cpp, .h) | ~30 |
| Soil Moisture Sensor (.cpp, .h) | ~30 |
| PWM Thread (.cpp, .h) | ~176 |
| I2C Thread (.cpp, .h) | ~128 |
| IO Thread (.cpp, .h) | ~122 |
| Main Application (main.cpp) | ~574 |

### Software PWM for RGB LED Control

- Custom timer-based PWM emulation at 200Hz
- Dynamic color scaling based on sensor readings
- Real-time violation handling with visual feedback
- Round-robin cycling through active violations

### Datasheets

- [Si7021 Temperature & Humidity Sensor](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf)
- [TCS34725 RGB Color Sensor](https://cdn-shop.adafruit.com/datasheets/TCS34725.pdf)
- [MMA8451Q Accelerometer](https://www.nxp.com/docs/en/data-sheet/MMA8451Q.pdf)
- [Ultimate GPS Module](https://cdn-shop.adafruit.com/datasheets/GlobalTop-FGPMMOPA6H-Datasheet-V0A.pdf)
- [SparkFun Soil Moisture Sensor](https://www.sparkfun.com/products/13322)
- [Photoresistor Light Sensor](https://cdn.sparkfun.com/datasheets/Sensors/LightImaging/SEN-09088.pdf)
