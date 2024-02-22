#include "mbed.h"

PwmOut pwm1(PB_14);
PwmOut pwm2(PB_13);
DigitalOut bipolar1(PA_11);
DigitalOut bipolar2(PA_12);
DigitalOut enablePin(PC_8);

class Motor
{
private:
    PwmOut &motor;   // Reference to the PWM output for this motor
    float dutyCycle; // PWM duty cycle
    float period;    // PWM period
public:
    Motor(PwmOut &motorPin, char identifier) : motor(motorPin)
    {
        dutyCycle = 0.50f;      // Set a default duty cycle
        period = 0.0001f;       // Set a default PWM period
        motor.period(period);   // Configure the PWM period
        motor.write(dutyCycle); // Set the initial PWM duty cycle
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
};

class Potentiometer
{
private:
    AnalogIn inputSignal;

public:
    Potentiometer(PinName pin) : inputSignal(pin) {}

    float read(void)
    {
        return inputSignal.read();
    }
};

int main()
{
    // Pin configuration for bipolar mode
    enablePin.write(1);
    bipolar1.write(1);
    bipolar2.write(1);

    // Create Potentiometer instance
    Potentiometer potentiometerLeft(A0);
    Potentiometer potentiometerRight(A1);

    // Potentiometer values
    float initialLeftPotValue = potentiometerLeft.read();
    float initialRightPotValue = potentiometerRight.read();

    // Create Motor instances for left and right motors
    Motor leftMotor(pwm1, 'L');
    Motor rightMotor(pwm2, 'R');

    // Immediately set the initial duty cycle based on the potentiometer readings
    leftMotor.setDutyCycle(initialLeftPotValue);
    rightMotor.setDutyCycle(initialRightPotValue);

    while (true)
    {
        // Continuously read potentiometer values and update the duty cycles
        float leftPotValue = potentiometerLeft.read();
        float rightPotValue = potentiometerRight.read();
        leftMotor.setDutyCycle(leftPotValue);
        rightMotor.setDutyCycle(rightPotValue);
    }
}