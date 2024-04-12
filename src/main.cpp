#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include <algorithm>
// Random comment
// Doing some changes
// New dev changes

PwmOut pwm1(PB_14);
PwmOut pwm2(PB_13);
DigitalOut bipolar1(PA_13);
DigitalOut bipolar2(PA_14);
DigitalOut enablePin(PC_8);
DigitalOut direction1(PC_10);
DigitalOut direction2(PC_6);
C12832 lcd(D11, D13, D12, D7, D10);
Serial pc(D1, D0);         // Initialize serial connection to PC
Serial hm10(PA_11, PA_12); // UART6 TX,RX

QEI leftEncoder(PB_7, PB_15, NC, 512, QEI::X2_ENCODING);
QEI rightEncoder(PB_12, PB_2, NC, 512, QEI::X2_ENCODING);

DigitalOut LineFollowSensorSwitch1(PC_12);
DigitalOut LineFollowSensorSwitch2(PA_15);
DigitalOut LineFollowSensorSwitch3(PC_11);
DigitalOut LineFollowSensorSwitch4(PD_2);
DigitalOut LineFollowSensorSwitch5(PC_9);

// Line sensor1 being the leftmost sensor
DigitalIn LineFollowSensor1(PC_3);
DigitalIn LineFollowSensor2(PC_2);
DigitalIn LineFollowSensor3(PC_4);
DigitalIn LineFollowSensor4(PB_1);
DigitalIn LineFollowSensor5(PC_5);
int LFSensor[5] = {0, 0, 0, 0, 0};

float Kp = 0.075; // Proportional gain (should be between 0 and 0.075)
float Ki = 0.01;  // Integral gain
float Kd = 0.05;  // Derivative gain
int errorValue = 0;

float previousError = 0;
float integral = 0;
float derivative = 0;
float P = 0;
float I = 0;
float D = 0;
float PIDvalue = 0;

float rightMotorSpeed = 0.5f; // Set the initial motor speed
float leftMotorSpeed = 0.5f;  // Set the initial motor speed
float desiredSpeed = 1.3;

enum Mode
{
    STOPPED,
    FOLLOW_LINE,
    TURN
};

Mode mode = FOLLOW_LINE; // Declare mode as a global variable

Timer t;

// comment

class Motor
{
private:
    PwmOut &motor;          // Reference to the PWM output for this motor
    QEI &encoder;           // Reference to the encoder for this motor
    float dutyCycle;        // PWM duty cycle
    float period;           // PWM period
    int previousPulseCount; // Previous pulse count for speed calculation
    Timer timer;            // Timer to measure elapsed time for speed calculation
    float wheelVelocity;    // Wheel velocity in m/s
    char motorIdentifier;   // Identifier for the motor
public:
    Motor(PwmOut &motorPin, QEI &encoderPin, char identifier) : motor(motorPin), encoder(encoderPin), motorIdentifier(identifier)
    {
        dutyCycle = 0.50f;                        // Set a default duty cycle
        period = 0.0001f;                         // Set a default PWM period
        motor.period(period);                     // Configure the PWM period
        motor.write(dutyCycle);                   // Set the initial PWM duty cycle
        previousPulseCount = encoder.getPulses(); // Initialize previous pulse count
        timer.start();                            // Start the timer
    }

    // Member function to set the PWM value
    void setDutyCycle(float newDutyCycle)
    {
        if (newDutyCycle < 0.0f)
            newDutyCycle = 0.0f;
        if (newDutyCycle > 1.0f)
            newDutyCycle = 1.0f;
        dutyCycle = newDutyCycle;
        motor.write(dutyCycle);
    }

    void stop()
    {
        setDutyCycle(0.5f); // Stop the motor
    }

    float getRPM()
    {
        float elapsedTime = timer.read();
        timer.reset();
        int currentPulseCount = encoder.getPulses();
        int pulseDifference = currentPulseCount - previousPulseCount;
        previousPulseCount = currentPulseCount;
        float revolutions = static_cast<float>(pulseDifference) / static_cast<float>(512);
        float rps = revolutions / elapsedTime;
        float rpm = rps * 60.0f;
        return rpm;
    }

    float getPulseCount()
    {
        return encoder.getPulses();
    }

    // New method to calculate and return the motor speed
    float getSpeed()
    {
        // Wheel velocity in m/s
        wheelVelocity = (getRPM() * 2 * 3.14159 * (0.082 / 2)) / 60;

        return wheelVelocity;
    }
};

// Function to calculate alignment error based on sensor input
// If buggy is to the left the error will be positive else if the buggy is to the right the error will be negative
void calculatePositionalError()
{
    // If the reading is 0 the sensor is detecting the line
    // Need to invert the readings

    LFSensor[0] = !LineFollowSensor1.read();
    LFSensor[1] = !LineFollowSensor2.read();
    LFSensor[2] = !LineFollowSensor3.read();
    LFSensor[3] = !LineFollowSensor4.read();
    LFSensor[4] = !LineFollowSensor5.read();

    if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 1))
        errorValue = 4;

    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 1))
        errorValue = 3;

    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 1) && (LFSensor[4] == 0))
        errorValue = 2;

    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 1) && (LFSensor[4] == 0))
        errorValue = 1;

    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
        errorValue = 0;

    else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 1) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
        errorValue = -1;

    else if ((LFSensor[0] == 0) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
        errorValue = -2;

    else if ((LFSensor[0] == 1) && (LFSensor[1] == 1) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
        errorValue = -3;

    else if ((LFSensor[0] == 1) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
        errorValue = -4;
}

void bluetoothCallback()
{
    if (hm10.readable())
    {
        char command = hm10.getc(); // Read command from Bluetooth
        if (command == 't')
        {
            mode = TURN;
        }
    }
}

// PID calculation
void calculatePID()
{
    P = errorValue;
    I = I + errorValue;
    D = errorValue - previousError;
    PIDvalue = Kp * P + Ki * I + Kd * D;
    previousError = errorValue;
}

void constantMotorVelocity(Motor &leftMotor, Motor &rightMotor)
{
    // Get current speeds
    float leftCurrentSpeed = std::floor(leftMotor.getSpeed() * 10) / 10;
    float rightCurrentSpeed = std::floor(rightMotor.getSpeed() * 10) / 10;

    // Proportional control parameter
    float Kp_speed = 0.09; // Tune this parameter based on performance

    // Calculate speed error
    float leftSpeedError = desiredSpeed - leftCurrentSpeed;
    float rightSpeedError = desiredSpeed - rightCurrentSpeed;

    // Apply proportional control and ensure speed does not exceed 1.3 m/s
    leftMotorSpeed = std::max(0.0f, std::min(1.0f, leftMotorSpeed + Kp_speed * leftSpeedError));
    rightMotorSpeed = std::max(0.0f, std::min(1.0f, rightMotorSpeed + Kp_speed * rightSpeedError));
}

void motorPIDcontrol(Motor &leftMotor, Motor &rightMotor)
{
    // If the buggy is on the line, move forward
    if (errorValue == 0)
    {
        leftMotor.setDutyCycle(leftMotorSpeed);
        rightMotor.setDutyCycle(rightMotorSpeed);
    }
    // If the buggy is on the left side of the line, turn the robot to the right
    else if (errorValue > 0)
    {
        leftMotor.setDutyCycle(leftMotorSpeed + PIDvalue);
    }

    // If the buggy is on the right side of the line, turn the robot to the left
    else if (errorValue < 0)
    {
        // leftMotor.setDutyCycle(0.7f - error * Kp);
        rightMotor.setDutyCycle(rightMotorSpeed - PIDvalue);
    }
}

void turnBuggy(Motor &leftMotor, Motor &rightMotor)
{
    leftMotor.setDutyCycle(0.3f);
    rightMotor.setDutyCycle(0.7f);
    wait(1.2);                    // Adjust time as necessary for the turn
    leftMotor.setDutyCycle(0.5f); // Stop turning by setting motors to neutral
    rightMotor.setDutyCycle(0.5f);
    wait(1.0);          // Adjust waiting time as necessary
    mode = FOLLOW_LINE; // Change mode back to FOLLOW_LINE after the turn is complete
}

int main()
{
    // Pin configuration for bipolar mode
    enablePin.write(1);
    bipolar1.write(1);
    bipolar2.write(1);
    direction1.write(1);
    direction2.write(1);
    LineFollowSensorSwitch1.write(1);
    LineFollowSensorSwitch2.write(1);
    LineFollowSensorSwitch3.write(1);
    LineFollowSensorSwitch4.write(1);
    LineFollowSensorSwitch5.write(1);

    hm10.baud(9600); // Set the baud rate to 9600

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
            // Need a function to increase
            calculatePID();
            constantMotorVelocity(leftMotor, rightMotor);
            motorPIDcontrol(leftMotor, rightMotor);
            break;
        case TURN:
            turnBuggy(leftMotor, rightMotor);
            break;
        }

        lcd.cls(); // Clear the screen before writing new data
        lcd.locate(0, 0);
        lcd.printf("Error: %d", errorValue);
    }
}
