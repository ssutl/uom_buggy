#include "mbed.h"

// Initialize serial communication for debugging
// Bipolar 1 PA_3
// Bipolar 2 PA_2
// PWM1 PA_10
// PWM2 PB_3
// Enable PB_5

DigitalOut bipolar1(D7);
DigitalOut bipolar2(D6);
DigitalOut enablePin(PB_5);
DigitalOut direction(PA_8);
PwmOut pwm1(PA_10);
PwmOut pwm2(PB_3);



int main() {

    // Enable the motor driver
    enablePin.write(1);

    // // Set bipolar outputs to high
    bipolar1.write(1);
    bipolar2.write(1);


    pwm1.period(0.001f); // 1kHz frequency
    pwm1.write(1.0f);    // 60% duty cycle 


    pwm2.period(0.001f); // 1kHz frequency
    pwm2.write(1.0f);    // 60% duty cycle
       

    while(1) {
            // The loop is empty, consider adding control logic or debugging messages here
        // Set PWM frequency to 1kHz and duty cycle to 60%
    }
}
