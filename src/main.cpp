#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

// LCD library
// http://os.mbed.com/users/askksa12543/code/C12832/

// Bipolar1 goes to PB_3
DigitalOut bipolarLeft(PB_3);
// Bipolar2 goes to PA_2
DigitalOut bipolarRight(PA_2);
// Enable goes to PC_4
DigitalOut enablePin(PC_4);
// PWM1 goes to PB_13
PwmOut motorLeft(PB_13);
// PWM2 goes to PB_14
PwmOut motorRight(PB_14);

// Plug BLE RX into PA_11 and TX into PA_12
Serial hm10(PA_11, PA_12); // TX, RX

C12832 lcd(D11, D13, D12, D7, D10);

// Configure your pins and pulses per revolution
// Channel A goes to PC_8 and Channel B goes to PC_6
QEI leftEncoder(PC_8, PC_6, NC, 512, QEI::X2_ENCODING);
// Channel A goes to PB_15 and Channel B goes to PB_1
QEI rightEncoder(PB_15, PB_1, NC, 512, QEI::X2_ENCODING);

Timer t;

char command;

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

    void setPeriod(float newPeriod)
    {
        motor.period(newPeriod); // Set the PWM period
    }

    void stop()
    {
        setDutyCycle(0.0f); // Stop the motor
    }

    // Member function to get the PWM value
    float getPwmValue() const
    {
        return dutyCycle;
    }

    // New method to calculate and return the motor speed
    float getSpeed()
    {
        float wheelDiameter = 0.15f; // in meters
        int gearRatio = 2;           // 2:1 gear ratio
        int cpr = 512;               // Encoder counts per revolution

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

class Potentiometer
{                                                    // Begin potentiometer class definition
private:                                             // Private data member declaration
    AnalogIn inputSignal;                            // Declaration of AnalogIn object
public:                                              // Public declarations
    Potentiometer(PinName pin) : inputSignal(pin) {} // Constructor - user provided pin name assigned to AnalogIn

    float read(void) const
    {                              // Public member function to measure the normalised amplitude
        return inputSignal.read(); // Returns the ADC value normalised to range 0.0 - 1.0
    }
};

int main()
{
    // Pin configuration for bipolar mode
    enablePin.write(1);
    bipolarLeft.write(1);
    bipolarRight.write(1);

    hm10.baud(9600); // Set the baud rate to 9600

    // Create Motor instances for left and right motors
    Motor leftMotor(motorLeft, leftEncoder, 'L');
    Motor rightMotor(motorRight, rightEncoder, 'R');

    // Create Potentiometer instance
    Potentiometer potentiometerLeft(A0);
    Potentiometer potentiometerRight(A1);

    char command;
    while (true)
    {
        // Task 3: Change the duty cycle of the motors based on the potentiometer readings
        // Task 4: Drive both motors independently and set their speed based on the potentiometer readings
        float leftPotValue = potentiometerLeft.read();
        float rightPotValue = potentiometerRight.read();
        leftMotor.setDutyCycle(leftPotValue);
        rightMotor.setDutyCycle(rightPotValue);

        // Task 5: Read the speed from encoder and show RPM and pulse count on the LCD
        float leftSpeed = leftMotor.getSpeed();
        float rightSpeed = rightMotor.getSpeed();
        lcd.cls(); // Clear the screen
        lcd.locate(0, 3);
        lcd.printf("Left RPM: %.2f", leftSpeed * 60);
        lcd.locate(0, 15);
        lcd.printf("Right RPM: %.2f", rightSpeed * 60);
        lcd.locate(0, 27);
        lcd.printf("Left Pulse: %d", leftEncoder.getPulses());
        lcd.locate(0, 39);
        lcd.printf("Right Pulse: %d", rightEncoder.getPulses());

        // Task 6: make robot drive in a 1m by 1m square, once the robot has completed the square,turn the robot around and trace the square again

        // Task 8: Show a visual response in response to the command received from the Bluetooth module
        if (hm10.readable())
        {
            command = hm10.getc(); // Read command from Bluetooth
            // Display command on LCD
            lcd.cls(); // Clear the screen
            lcd.locate(0, 3);
            lcd.printf("Command: %c", command);
        }
    }
}
