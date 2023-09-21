#ifndef MIXER_H
#define MIXER_H

#include "Arduino.h"
#include "CONFIG.h"
#include "OUTPUT.h"
#include "COMMAND.h"
#include "SENSOR.h"

enum FLIGHTMODE {
  INITIALIZE = 0, 
  READYSTEADY, 
  ASCENT, 
  DESCENT, 
  GLIDING
};

struct sysStatus {
  timeCode time;
  timeCode timeTransition;
  FLIGHTMODE flightMode; 
  bool initialised;
  bool separated;
  bool deployed;
  bool chuteOpened;

  mixStatus mix;
  outStatus out;
};

class mixClass {
  public:
    mixClass();
    mixStatus get();
    mixStatus compute(sysStatus);
  private: 
    bool solenoid1;
    bool solenoid2;
    bool solenoid3;
    bool solenoid4;
    bool servo1;
    bool servo2;
    bool ignitor;
    bool buzzer;
    mixStatus mixInit(sysStatus);
};

class sysClass {
  public:
    sysClass();
    sysStatus get();
    bool isInitialised();
    void setFlightMode(FLIGHTMODE);
    void setTransitionTime(timeCode);
    void setTime(timeCode);
    void setReady();
    void separate();
    void deploy();
    void openChute();

    mixClass mix;
    outClass out;

  private: 
    FLIGHTMODE flightMode; 
    timeCode time;
    timeCode timeTransition;
    bool initialised;
    bool separated;
    bool deployed;
    bool chuteOpened;
};

#endif 
