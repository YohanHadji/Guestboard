#include "FEEDBACK.h"

buzzerClass::buzzerClass()
 :buzzerPin(BUZZER_PIN), isOn(false), repeat(false)
{
    pinMode(buzzerPin, OUTPUT);
}

void buzzerClass::update() {
    if (millis() - timeOn >= duration && isOn && repeat) {
        timeOn = millis()-((millis()-timeOn)-duration);
        isOn = false;
        noTone(buzzerPin); 
    }
    
    if (millis() - timeOff >= durationOff && !isOn && repeat) {
        timeOff = millis()-((millis()-timeOff)-durationOff);
        isOn = true;
        tone(buzzerPin, frequency);
    }

    if (millis() - timeOn >= duration && !repeat) {
        noTone(buzzerPin);
    }
}

void buzzerClass::beep() {
    tone(buzzerPin, frequency);
    timeOn = millis();
}

//functions to produce the song we need depending on the state of R2HOME
void buzzerClass::buzzerTurnOn() {
    buzzerBeep(longLowFrequency); 
}
void buzzerClass::buzzerInit() { 
    buzzerBeep(lowFrequency); 
}
void buzzerClass::buzzerInitEnd() { 
    buzzerBeep(highFrequency);
 }
void buzzerClass::buzzerChangeFlightMode() { 
    buzzerBeep(longHighFrequency); 
}

void buzzerClass::buzzerBeep(typeBeeping type) {
  switch (type) {
    case lowFrequency:
         frequency = 1000;
         duration = 100;
         beep();
    break;

    case longLowFrequency:
        frequency = 1000;
        duration = 200;
        beep();
    break;

    case highFrequency:
        frequency = 3000;
        duration = 100;
        beep();
    break;
    
    case longHighFrequency:
        frequency = 3000;
        duration = 200;
        beep();
    break;
  }
  repeat = false;
  isOn = true;
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

