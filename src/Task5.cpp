#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

PwmOut pwm1(PB_14);
PwmOut pwm2(PB_13);
DigitalOut bipolar1(PA_11);
DigitalOut bipolar2(PA_12);
DigitalOut enablePin(PC_8);

C12832 lcd(D11, D13, D12, D7, D10);

QEI leftEncoder(PC_8, PC_6, NC, 512, QEI::X2_ENCODING);
QEI rightEncoder(PB_15, PB_1, NC, 512, QEI::X2_ENCODING);

Timer t;

class Motor
{
private:
    PwmOut &motor;          // Reference to the PWM output for this motor
    QEI &encoder;           // Reference to the encoder for this motor
    float dutyCycle;        // PWM duty cycle
    float period;           // PWM period
    int previousPulseCount; // Previous pulse count for speed calculation
    Timer timer;            // Timer to measure elapsed time for speed calculation
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

    // Method to get RPM
    float getRPM()
    {
        float elapsedTime = timer.read();
        timer.reset();
        int currentPulseCount = encoder.getPulses();
        int pulseDifference = currentPulseCount - previousPulseCount;
        previousPulseCount = currentPulseCount;
        float revolutions = static_cast<float>(pulseDifference) / pulsesPerRevolution;
        float rps = revolutions / elapsedTime;
        float rpm = rps * 60.0f;

        return rpm;
    }

    int getPulses()
    {
        return encoder.getPulses();
    }
};

int main()
{
    // Pin configuration for bipolar mode
    enablePin.write(1);
    bipolarLeft.write(1);
    bipolarRight.write(1);

    // Create Motor instances for left and right motors
    Motor leftMotor(pwm1, leftEncoder, 'L');
    Motor rightMotor(pwm2, rightEncoder, 'R');

    while (true)
    {
        // Display RPM and pulses on LCD
        leftMotor.getRPM();
        rightMotor.getRPM();
        lcd.cls();
        lcd.locate(0, 3);
        lcd.printf("Left RPM: %f", leftMotor.getRPM());
        lcd.locate(0, 15);
        lcd.printf("Right RPM: %f", rightMotor.getRPM());
        lcd.locate(0, 27);
        lcd.printf("Left Pulses: %d", leftMotor.getPulses());
        lcd.locate(0, 39);
        lcd.printf("Right Pulses: %d", rightMotor.getPulses());
    }
}