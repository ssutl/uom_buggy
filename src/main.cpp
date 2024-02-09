#include "mbed.h"

// Pin configuration on the Nucleo-64 board
// Bipolar 1 PA_3
// Bipolar 2 PA_2
// PWM1 PA_10
// PWM2 PB_3
// Enable PB_5

DigitalOut bipolarLeft(D7);
DigitalOut bipolarRight(D6);
DigitalOut enablePin(PB_5);
PwmOut pwmLeft(PA_10);
PwmOut pwmRight(PB_3);



int main() {
    //Pin configuration for bipolar mode
    enablePin.write(1);
    bipolarLeft.write(1);
    bipolarRight.write(1);


    //Pwm configuration
    pwmLeft.period(0.001f); 
    pwmLeft.write(1.0f);
    pwmRight.period(0.001f); 
    pwmRight.write(1.0f); 
       

    while(1) {
            // The loop is empty, consider adding control logic or debugging messages here
        // Set PWM frequency to 1kHz and duty cycle to 60%
    }
}
