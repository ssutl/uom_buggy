#include "mbed.h"
#include "QEI.h"
#include <chrono>





// The microcontroller provides a visual response to indicate
// the receipt of data sent via Bluetooth
// Read sensors input
// Read encoder ouput and display visual feedback
// The buggy should draw out a square, with sides of 1 m
// length and then stop when it reaches the starting position. It
// should then turn around, and re-trace the square in the
// opposite direction. 

// Pin configuration on the Nucleo-64 board
//Bipolar1 goes to D10
DigitalOut bipolarLeft(D10); 
//Bipolar2 goes to D8
DigitalOut bipolarRight(D8); 
//Enable goes to D7
DigitalOut enablePin(D7);
//PWM1 goes to D6
PwmOut motorLeft(D6);
//PWM2 goes to D9
PwmOut motorRight(D9);

// Plug BLE RX into PA_11 and TX into PA_12
BufferedSerial hm10(PA_11,PA_12); // TX, RX

// Configure your pins and pulses per revolution
//Channel A goes to D15 and Channel B goes to D14
QEI leftEncoder(D15, D14, NC, 512, QEI::X2_ENCODING);
QEI rightEncoder(D13, D12, NC, 512, QEI::X2_ENCODING);
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
public:
    // Constructor for the Motor class, specifying motor side and optional PWM value
    //The part after the colon is called an initializer list. It is used to initialize the member variables of the class prior to the constructor being run.
    //The part before is basically doing type safety for the inputted motor pin, ensuring that it is a PwmOut type and also a refrence.
     Motor(PwmOut &motorPin, QEI &encoderPin) : motor(motorPin), encoder(encoderPin) {
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
    ThisThread::sleep_for(100ms);

    t.stop();

    auto elapsed_time = t.elapsed_time(); // No change here
    printf("Elapsed Time: %lld us\n", elapsed_time.count()); // Using %lld for long long int


    int currentPulseCount = encoder.getPulses();
    int pulseDiff = currentPulseCount - previousPulseCount;
    

    float encoderTickRate = static_cast<float>(pulseDiff) / elapsed_time.count() * 1e6; // Convert to seconds
    wheelVelocity = (encoderTickRate * M_PI * wheelDiameter) / (gearRatio * cpr);

    printf("Wheel Velocity: %.2f m/s\n", wheelVelocity); // Using %.2f for precision

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
    hm10.set_baud(9600); // Set the baud rate to 9600

    // Create Motor instances for left and right motors
    Motor leftMotor(motorLeft, leftEncoder);
    Motor rightMotor(motorRight, rightEncoder); 

    //Demonstrating control of duty cycle and period
    leftMotor.setDutyCycle(0.50f); // Set the PWM duty cycle to 50%
    rightMotor.setDutyCycle(0.50f); // Set the PWM duty cycle to 50%


    char command;
    while (true) {
        leftMotor.getSpeed();
        //Bluetooth communication
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
