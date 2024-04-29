#include "mbed.h"
#include "QEI.h"
#include "C12832.h"
#include <algorithm>

C12832 lcd(D11, D13, D12, D7, D10);

DigitalOut LineFollowSensorSwitch1(PC_12);
DigitalOut LineFollowSensorSwitch2(PA_15);
DigitalOut LineFollowSensorSwitch3(PC_11);
DigitalOut LineFollowSensorSwitch4(PD_2);
DigitalOut LineFollowSensorSwitch5(PC_9);

// Line sensor1 being the leftmost sensor
AnalogIn LineFollowSensor1(PC_0);
AnalogIn LineFollowSensor2(PA_4);
AnalogIn LineFollowSensor5(PC_1);
AnalogIn LineFollowSensor3(PC_5);
AnalogIn LineFollowSensor4(PB_0);

// sensor 3 changes when over sensor 4
// sensor 4 changes when over sensor 5
// when over sensor 3 sensor sensor 5 changes

int main()
{
    LineFollowSensorSwitch1.write(1);
    LineFollowSensorSwitch2.write(1);
    LineFollowSensorSwitch3.write(1);
    LineFollowSensorSwitch4.write(1);
    LineFollowSensorSwitch5.write(1);

    while (true)
    {

        // print two in  one line
        lcd.locate(0, 0);
        lcd.printf("Sensor1: %.2f", LineFollowSensor1.read());
        lcd.locate(70, 0);
        lcd.printf("Sensor2: %.2f", LineFollowSensor2.read());
        lcd.locate(0, 10);
        lcd.printf("Sensor3: %.2f", LineFollowSensor3.read());
        lcd.locate(70, 10);
        lcd.printf("Sensor4: %.2f", LineFollowSensor4.read());
        lcd.locate(70, 20);
        lcd.printf("Sensor5: %.2f", LineFollowSensor5.read());

        wait(0.1);
    }
}
