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
AnalogIn LineFollowSensor1(PB_1);
AnalogIn LineFollowSensor2(PC_3);
AnalogIn LineFollowSensor3(PC_5);
int LFSensor[3] = {0, 0, 0};

int main()
{
    LineFollowSensorSwitch1.write(1);
    LineFollowSensorSwitch2.write(1);
    LineFollowSensorSwitch3.write(1);

    while (true)
    {
        // print an output for each sensor
        LFSensor[0] = LineFollowSensor1.read();
        LFSensor[1] = LineFollowSensor2.read();
        LFSensor[2] = LineFollowSensor3.read();

        // print two in  one line
        lcd.cls();
        lcd.locate(0, 0);
        lcd.printf("Sensor1: %.2f", LineFollowSensor1.read());
        lcd.locate(70, 0);
        lcd.printf("Sensor2: %.2f", LineFollowSensor2.read());
        lcd.locate(0, 10);
        lcd.printf("Sensor3: %.2f", LineFollowSensor3.read());

        wait(0.1);
    }
}
