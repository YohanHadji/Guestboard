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
  :strip(1, LED_PIN, NEO_GRBW + NEO_KHZ800), durationOn(25), durationOff(100), isOn(false)
{
    strip.begin();           
    strip.show();            
    strip.setBrightness(255);
}

void ledClass::colorWipe(uint32_t color) {
    for(int i=0; i<strip.numPixels(); i++) { 
        strip.setPixelColor(i, color);         
        strip.show();                          
    }
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

        case 1:
            currentColor = allColor::green;
            durationOff = 500;
            durationOn = 50;
        break;

        case 2:
            currentColor = allColor::blue; 
            durationOff = 1000;
            durationOn = 50;
            strip.setBrightness(25);
        break;

        case 3:
            currentColor = allColor::yellow;
            durationOff = 1000;
            durationOn = 50;
            strip.setBrightness(25);
        break;

        case 4:
            currentColor = allColor::purple;
            durationOff = 2000;
            durationOn = 50; 
            strip.setBrightness(25);
        break;

        case 5:
            currentColor = allColor::cyan;
            durationOff = 500; 
            durationOn = 10;
            strip.setBrightness(25);
        break;

        case 6:
        case 7:
        case 8:
            currentColor = allColor::white; 
            durationOff = 500;
            durationOn = 50;
            strip.setBrightness(25);
        break;

        case 9:
        case 10:
            //currentColor = allColor::red; 
            //durationOff = 200;
        break;

        default:
            currentColor.r = 0;
            currentColor.g = 0;
            currentColor.b = 0;
            durationOff = 1000;
        break;
    }
}

void ledClass::noWindFile() {
    currentColor = allColor::red;
    durationOff = 100;
    durationOn = 300;
}

void buzzerClass::noWindFile() {
    frequency = 3000;
    durationOff = 100;
    duration = 300;
    repeat = true;
    beep();
}

void ledClass::noWaypoint() {
    currentColor = allColor::red;
    durationOff = 100;
    durationOn = 100;
}

void buzzerClass::noWaypoint() {
    frequency = 3000;
    durationOff = 100;
    duration = 100;
    repeat = true;
    beep();
}

void ledClass::waypointOutOfRange() {
    currentColor.r = 1;
    currentColor.g = 0.5;
    currentColor.b = 0;
    durationOff = 500;
    durationOn = 100;
}

void buzzerClass::waypointOutOfRange() {
    frequency = 1000;
    durationOff = 500;
    duration = 100;
    repeat = true;
    beep();
}

void ledClass::positionOutOfRange() {
    currentColor.r = 1;
    currentColor.g = 0.5;
    currentColor.b = 0;
    durationOff = 200;
    durationOn = 100;
}

void buzzerClass::positionOutOfRange() {
    frequency = 1000;
    durationOff = 200;
    duration = 100;
    repeat = true;
    beep();
}

void ledClass::timeOutOfRange() {
    currentColor = allColor::blue;
    durationOff = 200;
    durationOn = 100;
}

void ledClass::timeWait() {
    currentColor = allColor::white;
    durationOff = 300;
    durationOn = 100;
}

void buzzerClass::timeOutOfRange() {
    frequency = 1500;
    durationOff = 200;
    duration = 100;
    repeat = true;
    beep();
}

//function that makes the led blink
void ledClass::update() {
    if (millis() - timeOn >= durationOn && isOn) {
        timeOff = millis()-((millis()-timeOn)-durationOn);
        isOn = false;
        // Turn off the led
        colorWipe(strip.Color(0,0,0));
    }

    if (millis()-timeOff >= durationOff && !isOn) {
        timeOn = millis();
        timeOn = millis()-((millis()-timeOff)-durationOff);
        isOn = true;
        // Turn on the led
        if (LED_MODEL == 1) { 
            colorWipe(strip.Color(currentColor.g, currentColor.r, currentColor.b)); 
        }
        else { 
            colorWipe(strip.Color(currentColor.r, currentColor.g, currentColor.b)); 
        }
    }
}

