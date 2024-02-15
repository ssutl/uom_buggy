#include "mbed.h"

// The microcontroller provides a visual response to indicate
// the receipt of data sent via Bluetooth
// Read sensors input
// Read encoder ouput and display visual feedback
// The buggy should draw out a square, with sides of 1 m
// length and then stop when it reaches the starting position. It
// should then turn around, and re-trace the square in the
// opposite direction. 

// Pin configuration on the Nucleo-64 board
// Bipolar1 plugged into D7
DigitalOut bipolarLeft(D7); 
//Bipolar2 plugged into D6
DigitalOut bipolarRight(D6); 
// Enable pin plugged into PB_5
DigitalOut enablePin(PB_5);
// PWM 1 plugged into PA_10
PwmOut motorLeft(PA_10);
// PWM 2 plugged into PB_3
PwmOut motorRight(PB_3);
// Plug BLE RX into PA_11 and TX into PA_12
BufferedSerial hm10(PA_11,PA_12); // TX, RX

char command;

class Motor {
private:
    PwmOut &motor; // Reference to the PWM output for this motor
    float dutyCycle = 0.5f; // Default PWM value
    float period = 0.001f; // Default PWM period

public:
    // Constructor for the Motor class, specifying motor side and optional PWM value
    //The part after the colon is called an initializer list. It is used to initialize the member variables of the class prior to the constructor being run.
    //The part before is basically doing type safety for the inputted motor pin, ensuring that it is a PwmOut type and also a refrence.
    Motor(PwmOut &randomInputtedMotorPin) : motor(randomInputtedMotorPin) {
        motor.period(period);
        motor.write(dutyCycle);
    }

    // Member function to set the PWM value
    void setDutyCycle(float newDutyCycle) {
        // Clamp the PWM value to the range [0.0f, 1.0f]
        if(newDutyCycle < 0.0f) {
            dutyCycle = 0.0f;
        } else if(newDutyCycle > 1.0f) {
            dutyCycle = 1.0f;
        } else {
            dutyCycle = newDutyCycle;
        }

        motor.write(dutyCycle); // Set the PWM duty cycle
    }

    void setPeriod(float newPeriod) {
        motor.period(newPeriod); // Set the PWM period
    }

    void stop() {
        setDutyCycle(0.0f); // Stop the motor
    }

    // Member function to get the PWM value
    float getPwmValue() const {
        return dutyCycle;
    }
};

int main() {
    // Pin configuration for bipolar mode
    enablePin = 1;
    bipolarLeft = 1;
    bipolarRight = 1;

    //BLE configuration
    //Since asynchronous communication doesn’t use a clock, the two devices communicating  agree a common clock speed to determine the bit period. This is called the baud rate.
    //A baud rate is essential in serial communication because it specifies the speed at which data is transmitted and received over a communication channel. 
    hm10.set_baud(9600); // Set the baud rate to 9600




    // Create Motor instances for left and right motors
    Motor leftMotor(motorLeft);
    Motor rightMotor(motorRight);


    char command;
    while (true) {
        if (hm10.readable()) {
            if (hm10.read(&command, 1)) {
                // Implement movement logic based on command
                // For example, if command is 'A', move forward; if 'B', move backward;
                // This is a placeholder. You need to define the actual commands and implement the logic.
                switch (command) {
                    case 'A':
                        // Move forward
                        break;
                    case 'B':
                        // Move backward
                        break;
                    // Add cases for turning and stopping
                }
            }
        }
    }
}
