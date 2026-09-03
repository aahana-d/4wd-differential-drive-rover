# 4WD Differential-Drive Robotic Rover

A 4WD differential-drive robotic rover designed and built using an Arduino microcontroller, L298N motor drivers, and DC motors. The project combines embedded C++ firmware with Bluetooth and SQL-based control interfaces to enable remote directional control, PWM-based speed control, and command logging.

The project combines embedded C++ firmware with Bluetooth and SQL-based control interfaces to enable remote directional control, PWM-based speed control, and command logging.

## Project Status

The project is being developed incrementally, with motor control and Bluetooth communication, with plans to enhance remote control and include SQL-backed command management.

## Planned Features

- 4WD differential-drive movement
- Independent control of left and right motor groups
- Forward, reverse, left, right, and stop commands
- PWM-based motor speed control
- Arduino-based embedded C++ firmware
- Bluetooth remote control
- SQL-based command and status logging
- Safety timeout for lost communication
- REST API for remote rover control
- Hardware and software validation testing

## Hardware

- Arduino microcontroller
- 4 × DC motors
- L298N motor drivers
- Bluetooth module
- Battery/power system
- 4WD rover chassis

## Software

- Embedded C++
- Arduino framework
- Bluetooth serial communication
- Python
- SQL
- REST API

## Project Structure

```text
4wd-differential-drive-rover/
│
├── firmware/       # Arduino embedded C++ firmware
├── server/         # Remote control/API backend
├── database/       # SQL schema and database files
├── tests/          # Unit and integration tests
├── docs/           # Project documentation
│
├── README.md
└── .gitignore
