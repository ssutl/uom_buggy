# Line Following Buggy

[![Language](https://img.shields.io/badge/language-C%2B%2B-blue)](https://www.cplusplus.com/)
[![Framework](https://img.shields.io/badge/framework-Mbed-blue)](https://www.mbed.com/)

This codebase contains the software which controls an autonomous buggy. Within this codebase, you can find functionalities such as controlling motors, reading sensors, implementing PID control, and Bluetooth communication.

## Table of Contents
- [Line Following Buggy](#line-following-buggy)
  * [Table of Contents](#table-of-contents)
  * [Features](#features)
  * [Requirements](#requirements)
  * [Getting Started](#getting-started)
    + [Integration](#integration)
  * [Class Declarations](#class-declarations)
  * [Contributing](#contributing)
  * [License](#license)

## Features

- **Motor Control**: Control the speed and direction of the buggy's motors.
- **Sensor Reading**: Read data from infrared sensors for line following.
- **PID Control**: Implement PID control for precise motor speed adjustment.
- **Bluetooth Communication**: Interact with the buggy via Bluetooth commands.

## Requirements

- [mbed OS](https://os.mbed.com/)
- Compatible microcontroller (e.g., STM32, NXP)

## Getting Started

### Integration

1. **Integration**: Clone or download this repository.
2. **Library Dependencies**:
   - Download the [QEI.h library](https://os.mbed.com/users/aberk/code/QEI/file/5c2ad81551aa/QEI.h/) and include it in your project.
   - Download the [C12832 LCD library](https://os.mbed.com/teams/components/code/C12832/docs/tip/C12832_8h_source.html) and include it in your project.

## Class Declarations

**Class Diagrams:**
*Here you would insert an image of the class diagrams.*

### Motor Class

Controls the operation of a motor using PWM output and encoder feedback.

| Function Name   | Description                                               | How to Call                       |
|-----------------|-----------------------------------------------------------|-----------------------------------|
| Motor           | Constructor to initialize a motor with given parameters.  | `Motor motorName(pwmPin, encoderPin, identifier);` |
| setDutyCycle    | Sets the PWM duty cycle for the motor.                    | `motorName.setDutyCycle(newDutyCycle);` |
| stop            | Stops the motor by setting the duty cycle to neutral.     | `motorName.stop();` |
| getPwmValue     | Returns the current PWM duty cycle value.                 | `float duty = motorName.getPwmValue();` |
| getRPM          | Calculates and returns the motor RPM.                     | `float rpm = motorName.getRPM();` |
| getPulse        | Returns the pulse count from the encoder.                 | `int pulseCount = motorName.getPulse();` |
| getSpeed        | Calculates and returns the motor's speed in m/s.          | `float speed = motorName.getSpeed();` |

*Note: More class declarations would follow here based on your other classes.*

## Contributing

Contributions to this project are welcome! If you have suggestions for improvements or new features, feel free to open an issue or submit a pull request.

## License

This code is licensed under the [MIT License](LICENSE).
