#ifndef FEEDBACK_H
#define FEEDBACK_H

#include "Arduino.h"
#include "CONFIG.h"
#include <Adafruit_NeoPixel.h>

//all the buzzer state we will need 
enum typeBeeping {lowFrequency = 1, longLowFrequency, highFrequency, longHighFrequency};

struct rgbColor {
    int r;
    int g;
    int b;
};

//repertory of all the color we use on the led
namespace allColor {
    const rgbColor red({255, 0, 0});
    const rgbColor green({0, 255, 0});
    const rgbColor blue({0, 0, 255});
    const rgbColor yellow({255, 234, 0});
    const rgbColor purple({238, 130, 238});
    const rgbColor cyan({0, 255, 255});
    const rgbColor white({255, 255, 255});
    const rgbColor orange({255, 165, 0});
};

class buzzerClass {
    public:
    buzzerClass();
    void update();
    void buzzerTurnOn();
    void buzzerInit();
    void buzzerInitEnd();
    void buzzerChangeFlightMode();
    void buzzerBeep(typeBeeping type);
    void beep();

    private:
    int buzzerPin;
    unsigned int duration, frequency, durationOff;
    unsigned long timeOn, timeOff;
    bool isOn, repeat;
};

class ledClass {
    public:
    ledClass();
    void switchColor(int flightMode);
    void update();
    
    private:
    enum color {red = 1, green, blue, yellow, purple, cyan, white};
    void set(int r, int g, int b);
    rgbColor currentColor;
    // Adafruit_NeoPixel strip;
    unsigned durationOn, durationOff, timeOn, timeOff;
    bool isOn;
    bool boolColor;
};


#endif 
