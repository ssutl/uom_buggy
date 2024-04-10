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
  * [Classes and Functions](#classes-and-functions)
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

## Classes and Functions

### Motor Class

Controls the operation of a motor using PWM output and encoder feedback.

| Function Name   | Description                                               | How to Call                         |
|-----------------|-----------------------------------------------------------|-------------------------------------|
| Motor           | Constructor to initialize a motor with given parameters.  | `Motor motorName(pwmPin, encoderPin, identifier);` |
| setDutyCycle    | Sets the PWM duty cycle for the motor.                    | `motorName.setDutyCycle(newDutyCycle);` |
| stop            | Stops the motor by setting the duty cycle to neutral.     | `motorName.stop();` |
| getPwmValue     | Returns the current PWM duty cycle value.                 | `float duty = motorName.getPwmValue();` |
| getRPM          | Calculates and returns the motor RPM.                     | `float rpm = motorName.getRPM();` |
| getPulse        | Returns the pulse count from the encoder.                 | `int pulseCount = motorName.getPulse();` |
| getSpeed        | Calculates and returns the motor's speed in m/s.          | `float speed = motorName.getSpeed();` |

### PID Control Functions

Provides functionality for calculating PID values and adjusting motor control based on sensor input.

| Function Name        | Description                                                  | How to Call                                        |
|----------------------|--------------------------------------------------------------|----------------------------------------------------|
| calculatePID         | Calculates the PID value based on error.                     | `float pidValue = calculatePID(error);`            |
| calculatePositionalError | Calculates the error based on sensor readings.            | `float error = calculatePositionalError();`        |
| adjustMotors         | Adjusts the motor speeds based on the provided parameters.   | `adjustMotors(pidOutput, error);`                  |

### Motor Control Functions

Contains functions to control the movements and adjustments of the buggy's motors.

| Function Name        | Description                                                  | How to Call                                        |
|----------------------|--------------------------------------------------------------|----------------------------------------------------|
| turnBuggy            | Commands to turn the buggy.                                  | `turnBuggy();`                                     |
| motorPIDcontrol      | Adjusts the motor speeds based on the PID output and error.  | `motorPIDcontrol(pidOutput, error);`               |

### Bluetooth Communication Functions

Handles Bluetooth communication, allowing remote interaction with the buggy.

| Function Name        | Description                                                  | How to Call                                        |
|----------------------|--------------------------------------------------------------|----------------------------------------------------|
| bluetoothCallback    | Callback function to handle Bluetooth commands.              | `hm10.attach(&bluetoothCallback);`                 |

## Basic Setup

Here's an example of the main function setup in the code:

```cpp
#include "mbed.h"
// Include other necessary headers here

// Instantiate global instances of your classes here
// Example: Motor leftMotor(pwmPin, encoderPin, 'L');

int main()
{
    // Enable/Disable pins

    // Create Motor instances for left and right motors
    Motor leftMotor(pwm1, leftEncoder, 'L');
    Motor rightMotor(pwm2, rightEncoder, 'R');

    while (true)
    {
        calculatePositionalError();

        switch (mode)
        {
        case STOPPED:
            leftMotor.stop();
            rightMotor.stop();
            break;
        case FOLLOW_LINE:
            calculatePID();
            motorPIDcontrol(leftMotor, rightMotor);
            break;
        case TURN:
            turnBuggy(leftMotor, rightMotor);
            break;
        }
        wait(0.1);
    }
}
```





