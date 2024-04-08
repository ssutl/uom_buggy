#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
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

DigitalIn LineFollowSensor0(PA_16);
DigitalIn LineFollowSensor1(PA_17);
DigitalIn LineFollowSensor2(PA_18);
DigitalIn LineFollowSensor3(PA_19);
DigitalIn LineFollowSensor4(PA_20);
DigitalIn LineFollowSensor5(PA_21);
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

    LFSensor[0] = LineFollowSensor0.read();
    LFSensor[1] = LineFollowSensor1.read();
    LFSensor[2] = LineFollowSensor2.read();
    LFSensor[3] = LineFollowSensor3.read();
    LFSensor[4] = LineFollowSensor4.read();

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

    // If none of the sensors dont detect anything stop the buggy
    else if ((LFSensor[0] == 0) && (LFSensor[1] == 0) && (LFSensor[2] == 0) && (LFSensor[3] == 0) && (LFSensor[4] == 0))
        mode = STOPPED;
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

void motorPIDcontrol(Motor &leftMotor, Motor &rightMotor)
{
    if (errorValue == 0)
    {
        leftMotor.setDutyCycle(0.7f);
        rightMotor.setDutyCycle(0.7f);
    }
    // If the buggy is on the left side of the line, turn the robot to the right
    else if (errorValue > 0)
    {
        leftMotor.setDutyCycle(0.7f + PIDvalue);
    }

    // If the buggy is on the right side of the line, turn the robot to the left
    else if (errorValue < 0)
    {
        // leftMotor.setDutyCycle(0.7f - error * Kp);
        rightMotor.setDutyCycle(0.7f + PIDvalue);
    }
}

int main()
{
    // Pin configuration for bipolar mode
    enablePin.write(1);
    bipolar1.write(1);
    bipolar2.write(1);
    direction1.write(1);
    direction2.write(1);

    hm10.baud(9600); // Set the baud rate to 9600

    // Create Motor instances for left and right motors
    Motor leftMotor(pwm1, leftEncoder, 'L');
    Motor rightMotor(pwm2, rightEncoder, 'R');

    leftMotor.setDutyCycle(0.7f);
    rightMotor.setDutyCycle(0.7f);

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
            leftMotor.setDutyCycle(0.3f);
            rightMotor.setDutyCycle(0.7f);
            wait(1.2);
            leftMotor.setDutyCycle(0.5f);
            rightMotor.setDutyCycle(0.5f);
            wait(1.0);
            mode = FOLLOW_LINE;
            break;
        }

        // Print sensor values on LCD
        lcd.cls();
        lcd.locate(0, 0);
        lcd.printf("LFSensor0: %d", LFSensor[0]);
        lcd.locate(0, 10);
        lcd.printf("LFSensor1: %d", LFSensor[1]);
        lcd.locate(0, 20);
        lcd.printf("LFSensor2: %d", LFSensor[2]);
        lcd.locate(0, 30);
        lcd.printf("LFSensor3: %d", LFSensor[3]);
        lcd.locate(0, 40);
        lcd.printf("LFSensor4: %d", LFSensor[4]);
        wait_ms(10); // Adjust the delay as needed
    }
}
