#include "mbed.h"
#include "QEI.h"
#include "C12832.h"

C12832 lcd(D11, D13, D12, D7, D10);
Serial pc(D1, D0); // Initialize serial connection to PC

int main()
{
    // Pin configuration for bipolar mode
    enablePin.write(1);

    char command;
    while (true)
    {
    }
}
