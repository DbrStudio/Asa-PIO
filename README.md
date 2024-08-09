# ASA Robotics Motor Control and Sensor Integration

## Overview

This project is designed to control motors and read sensor data using an Arduino-compatible microcontroller. 
It integrates various hardware components, including motors, an MPU6050 accelerometer/gyroscope sensor, 
and RPM sensors in the form of optical switches, to achieve precise motor control and real-time sensor data acquisition.

## Features

- **Motor Control:** Uses Adafruit MotorShield to control four DC motors, allowing for forward and backward movement.
- **Sensor Integration:** Reads data from an MPU6050 sensor to monitor acceleration, rotation, and temperature.
- **Odometry:** Tracks motor RPMs and calculates odometry data for navigation purposes.
- **Communication:** Implements a communication protocol for sending motor commands and receiving sensor data.

## Hardware Requirements

- Arduino-compatible microcontroller
- Adafruit MotorShield (I2C address: 0x60)
- 4x DC Motors (preferrably with a through axle to mount an encoder disc to it)
- MPU6050 Accelerometer and Gyroscope sensor
- Optical switches (these will be used as RPM sensors)
- slotted discs with 20 slots each

## Software Dependencies

The project relies on several Arduino libraries, which need to be installed in your Arduino IDE:

- `Adafruit_MotorShield`
- `Adafruit_MPU6050`
- `Adafruit_Sensor`
- `AutoPID`
- `LowPass` (can be found in the Lib folder)
- `SPI`
- `Wire`
- `TimeLib`

You can install these libraries using the Arduino Library Manager.

## Installation

1. **Clone the repository:**

2. Open the main.cpp file in PlatformIO:

    Ensure that all required libraries are installed.
    Connect your microcontroller to your computer.

3. Upload the code:

    Select the correct board and port in the Arduino IDE.
    Click the "Upload" button to flash the code to your microcontroller.

## Setup

1. Connect Arduino to Raspberry Pi

2. Connect optical switches to pin A0-A3

3. Send Unix Timestamp to Arduino once prompted via serial (BAUD 115200)

4. Once Startup is complete send motor control commands in this format:
    ```bash
    <m1, m2, m3, m4>
    ```
    The values should be between -200 and 200

5. The Arduino will send odometry data every 50ms in the following format:
    ```bash
    Timestamp,m1=value,m2=value,m3=value,m4=value,acc_x=value,acc_y=value,acc_z=value,rot_x=value,rot_y=value,rot_z=value,temp_c=value,hash
    ```
    
## Contributing
Contributions are welcome! If you have any ideas, improvements, or bug fixes, please fork the repository and submit a pull request.

## License
This project is licensed under the MIT License - see the LICENSE file for details.