# Line Following Buggy

[![Language](https://img.shields.io/badge/language-C%2B%2B-blue)](https://www.cplusplus.com/)
[![Framework](https://img.shields.io/badge/framework-Mbed-blue)](https://www.mbed.com/)

This codebase contains the software which controls an autonomous buggy. Within this codebase you can find functionalities such as controlling motors, reading sensors, implementing PID control, and Bluetooth communication.

## Features

- **Motor Control**: Control the speed and direction of the buggy's motors.
- **Sensor Reading**: Read data from infrared sensors for line following.
- **PID Control**: Implement PID control for precise motor speed adjustment.
- **Bluetooth Communication**: Interact with the buggy via Bluetooth commands.

## Requirements

- [mbed OS](https://os.mbed.com/)
- Compatible microcontroller (e.g., STM32, NXP)

## Getting Started

1. **Integration**: Clone or download this repository.
2. **Examples**: Explore example projects included in the repository.
3. **Building from Source**: Compile the code using an appropriate IDE or command-line tools.

## Modules

The main modules of this codebase include:

- **Motor Control**: Provides functionalities for controlling motors, calculating motor speed, and adjusting motor speeds.
- **Sensor Reading**: Contains functions for reading data from infrared sensors.
- **PID Control**: Implements PID control algorithm for motor speed adjustment.
- **Bluetooth Communication**: Handles Bluetooth communication for sending commands to the buggy.

## Contributing

Contributions to this project are welcome! If you have suggestions for improvements or new features, feel free to open an issue or submit a pull request.

## License

This code is licensed under the [MIT License](LICENSE).
