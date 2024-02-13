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
DigitalOut bipolarLeft(D7); 
DigitalOut bipolarRight(D6); 
DigitalOut enablePin(PB_5);
PwmOut motorLeft(PA_10);
PwmOut motorRight(PB_3);

class Motor {
private:
    PwmOut &motor; // Reference to the PWM output for this motor
    float dutyCycle = 0.5f; // Default PWM value
    float period = 0.001f; // Default PWM period

public:
    // Constructor for the Motor class, specifying motor side and optional PWM value
    Motor(char motorSide) : motor(motorSide == 'L' ? motorLeft : motorRight) {
        motor.period(period); // Set inital PWM period
        motor.write(dutyCycle); // Set initial PWM value
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
        motor.period(period); // Set the PWM period
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


    // Create Motor instances for left and right motors
    Motor leftMotor('L'); 
    Motor rightMotor('R');

    


    while(true) {
        // Main loop
    }
}
