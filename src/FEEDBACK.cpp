#include "FEEDBACK.h"

buzzerClass::buzzerClass()
 :isOn(false), repeat(false)
{
}

ledClass::ledClass() 
{

}

//set the led in a particular color depending on the flight mode
void ledClass::switchColor(int flightMode) {
    // Color code, Mode:Color
    // 0:Red, 1:Green, 2:Blue, 3:Yellow, 4:Purple, 5:Cyan, 6:White

    if (flightMode == 9) {
        return;
    }
    
    switch (flightMode) {
        case 0:
            currentColor = allColor::red;
            durationOff = 1000;
            durationOn = 50;
        break;

        default:
            currentColor.r = 0;
            currentColor.g = 0;
            currentColor.b = 0;
            durationOff = 1000;
        break;
    }
}

//function that makes the led blink
void ledClass::update() {
    if (millis() - timeOn >= durationOn && isOn) {
        timeOff = millis()-((millis()-timeOn)-durationOn);
        isOn = false;
        // Turn off the led
    }

    if (millis()-timeOff >= durationOff && !isOn) {
        timeOn = millis();
        timeOn = millis()-((millis()-timeOff)-durationOff);
        isOn = true;
        // Turn on the led
    }
}

