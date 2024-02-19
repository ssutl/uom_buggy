#include "mbed.h"
#include "QEI.h"

// The microcontroller provides a visual response to indicate
// the receipt of data sent via Bluetooth
// Read sensors input
// Read encoder ouput and display visual feedback
// The buggy should draw out a square, with sides of 1 m
// length and then stop when it reaches the starting position. It
// should then turn around, and re-trace the square in the
// opposite direction. 

// Pin configuration on the Nucleo-64 board
//Bipolar1 goes to PB_3
DigitalOut bipolarLeft(PB_3); 
//Bipolar2 goes to PA_2
DigitalOut bipolarRight(PA_2); 
//Enable goes to PC_4
DigitalOut enablePin(PC_4);
//PWM1 goes to PB_13
PwmOut motorLeft(PB_13);
//PWM2 goes to PB_14
PwmOut motorRight(PB_14);

// Plug BLE RX into PA_11 and TX into PA_12
Serial hm10(PA_11,PA_12); // TX, RX

// Configure your pins and pulses per revolution
//Channel A goes to PC_8 and Channel B goes to PC_6
QEI leftEncoder(PC_8, PC_6, NC, 512, QEI::X2_ENCODING);
//Channel A goes to PB_15 and Channel B goes to PB_1
QEI rightEncoder(PB_15, PB_1, NC, 512, QEI::X2_ENCODING);

Timer t;

char command;

class Motor {
private:
    PwmOut &motor; // Reference to the PWM output for this motor
    QEI &encoder; // Reference to the encoder for this motor
    float dutyCycle; // PWM duty cycle
    float period; // PWM period
    int previousPulseCount; // Previous pulse count for speed calculation
    Timer timer; // Timer to measure elapsed time for speed calculation
    float wheelVelocity; // Wheel velocity in m/s
    char motorIdentifier; // Identifier for the motor
public:
    // Constructor for the Motor class, specifying motor side and optional PWM value
    //The part after the colon is called an initializer list. It is used to initialize the member variables of the class prior to the constructor being run.
    //The part before is basically doing type safety for the inputted motor pin, ensuring that it is a PwmOut type and also a refrence.
     Motor(PwmOut &motorPin, QEI &encoderPin, char identifier) : motor(motorPin), encoder(encoderPin), motorIdentifier(identifier) {
        dutyCycle = 0.50f; // Set a default duty cycle
        period = 0.0001f; // Set a default PWM period
        motor.period(period); // Configure the PWM period
        motor.write(dutyCycle); // Set the initial PWM duty cycle
        previousPulseCount = encoder.getPulses(); // Initialize previous pulse count
        timer.start(); // Start the timer
    }

    // Member function to set the PWM value
    void setDutyCycle(float newDutyCycle) {
        if (newDutyCycle < 0.0f) newDutyCycle = 0.0f;
        if (newDutyCycle > 1.0f) newDutyCycle = 1.0f;
        dutyCycle = newDutyCycle;
        motor.write(dutyCycle);
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

    // New method to calculate and return the motor speed
    float getSpeed() {
    float wheelDiameter = 0.15f; // in meters
    int gearRatio = 2; // 2:1 gear ratio
    int cpr = 512; // Encoder counts per revolution

    // Reset and start timer for sampling period
    t.reset();
    t.start();

    int previousPulseCount = encoder.getPulses();

    // Sample time (dt) - Wait 100ms
    wait_ms(100);

    t.stop();

    float elapsed_time = t.read(); // No change here


    int currentPulseCount = encoder.getPulses();
    int pulseDiff = currentPulseCount - previousPulseCount;
    

    float encoderTickRate = static_cast<float>(pulseDiff) / elapsed_time; // Convert to seconds
    wheelVelocity = (encoderTickRate * 3.14 * wheelDiameter) / (gearRatio * cpr);

    printf("%c Motor Speed: %.2f m/s\n", motorIdentifier, wheelVelocity);

    return wheelVelocity;
}

};

int main() {
    // Pin configuration for bipolar mode
    enablePin.write(1);
    bipolarLeft.write(1);
    bipolarRight.write(1);

    //BLE configuration
    //Since asynchronous communication doesn’t use a clock, the two devices communicating  agree a common clock speed to determine the bit period. This is called the baud rate.
    //A baud rate is essential in serial communication because it specifies the speed at which data is transmitted and received over a communication channel. 
    hm10.baud(9600); // Set the baud rate to 9600

    // Create Motor instances for left and right motors
    Motor leftMotor(motorLeft, leftEncoder, 'L');
    Motor rightMotor(motorRight, rightEncoder, 'R'); 

    //Demonstrating control of duty cycle and period
    leftMotor.setDutyCycle(0.40f); // Set the PWM duty cycle to 50%
    rightMotor.setDutyCycle(0.40f); // Set the PWM duty cycle to 50%





    char command;
    while (true) {
        // leftMotor.getSpeed();
        // rightMotor.getSpeed();


        if (hm10.readable()) {
            command = hm10.getc(); // Read command from Bluetooth
            // Implement movement logic based on command
            switch (command) {
                case 'A':
                    // Example: Move forward
                    leftMotor.setDutyCycle(0.5f);
                    rightMotor.setDutyCycle(0.5f);
                    break;
                case 'B':
                    // Example: Move backward
                    leftMotor.setDutyCycle(0.6f); // Assuming you have a way to handle reverse
                    rightMotor.setDutyCycle(0.6f);
                    break;
                // Add more cases as needed
            }
        }

       
    }
}
