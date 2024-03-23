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
DigitalOut direction1(PC_5);
DigitalOut direction2(PC_6);
C12832 lcd(D11, D13, D12, D7, D10);
Serial pc(D1, D0);         // Initialize serial connection to PC
Serial hm10(PA_11, PA_12); // UART6 TX,RX

QEI leftEncoder(PB_1, PB_15, NC, 512, QEI::X2_ENCODING);
QEI rightEncoder(PB_12, PB_2, NC, 512, QEI::X2_ENCODING);

DigitalIn sensor1(1);
DigitalIn sensor2(2);
DigitalIn sensor3(3);
DigitalIn sensor4(4);
DigitalIn sensor5(5);

float Kp = 0.2;  // Proportional gain
float Ki = 0.01; // Integral gain
float Kd = 0.05; // Derivative gain

float previousError = 0;
float integral = 0;

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

    // Member function to get the PWM value
    float getPwmValue() const
    {
        return dutyCycle;
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

    int getPulse()
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

void turnBuggy()
{
    // Turn the buggy
    leftMotor.setDutyCycle(0.3f);
    rightMotor.setDutyCycle(0.7f);
    wait(1.2);
    leftMotor.setDutyCycle(0.5f);
    rightMotor.setDutyCycle(0.5f);
    wait(1.0);
}

// Function to calculate alignment error based on sensor input
float calculatePositionalError()
{
    float error = 0;
    // Example sensor values and weights (adjust based on your setup)
    if (sensor1)
        error += -2;
    if (sensor2)
        error += -1;
    if (sensor4)
        error += 1;
    if (sensor5)
        error += 2;
    // Sensor 3 is the reference, no error contribution

    return error;
}

void bluetoothCallback()
{
    if (hm10.readable())
    {
        char command = hm10.getc(); // Read command from Bluetooth
        if (command = "t")
        {
            turnBuggy();
        }
    }
}

// PID calculation
float calculatePID(float error)
{
    float derivative = error - previousError;
    integral += error;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    return output;
}

void adjustMotors(float pidOutput, float error)
{
    // Default speed
    float defaultSpeed = 0.7;

    // If there is no error, keep the speed at 0.7 for both motors
    if (error == 0)
    {
        leftMotor.setDutyCycle(defaultSpeed);
        rightMotor.setDutyCycle(defaultSpeed);
    }
    else
    {
        // Adjust speeds based on PID output when there is an error
        float leftMotorSpeed = defaultSpeed - pidOutput;  // Adjust left motor speed
        float rightMotorSpeed = defaultSpeed + pidOutput; // Adjust right motor speed

        // Ensure motor speeds are within bounds [0.0, 1.0]
        leftMotorSpeed = std::min(std::max(leftMotorSpeed, 0.0f), 1.0f);
        rightMotorSpeed = std::min(std::max(rightMotorSpeed, 0.0f), 1.0f);

        leftMotor.setDutyCycle(leftMotorSpeed);
        rightMotor.setDutyCycle(rightMotorSpeed);
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

    char command;
    // Set initial motor speeds to 0.7
    leftMotor.setDutyCycle(0.7f);
    rightMotor.setDutyCycle(0.7f);

    while (true)
    {
        float error = calculateError();        // Calculate the alignment error
        float pidOutput = calculatePID(error); // Calculate PID output based on the error
        adjustMotors(pidOutput, error);        // Adjust motors based on PID output and error

        wait_ms(10); // Adjust the delay as needed
    }
}
