#ifndef MIXER_H
#define MIXER_H

#include "Arduino.h"
#include "CONFIG.h"
#include "OUTPUT.h"
#include "TELECOM.h"
#include "SENSOR.h"

enum FLIGHTMODE {
  INITIALIZE_MODE = 0, 
  READYSTEADY_MODE,
  CALIBRATION_MODE,
  MANUAL_MODE,
  ARMED_MODE,
  PRESSURED_MODE,
  IGNITER_MODE,
  IGNITION_MODE,
  THRUST_MODE,
  SHUTDOWN_MODE, 
  ASCENT_MODE, 
  DESCENT_MODE, 
  GLIDING_MODE,
  ABORT_MODE
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
    void setManualMemory(mixStatus);
    mixStatus compute(sysStatus);
    mixStatus mixManualMemory;
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
    mixStatus mixReadySteady(sysStatus);
    mixStatus mixCalibration(sysStatus);
    mixStatus mixManual(sysStatus);
    mixStatus mixArmed(sysStatus);
    mixStatus mixPressured(sysStatus);
    mixStatus mixIgniter(sysStatus);
    mixStatus mixIgnition(sysStatus);
    mixStatus mixThrust(sysStatus);
    mixStatus mixShutdown(sysStatus);
    mixStatus mixFlightAscent(sysStatus);
    mixStatus mixFlightDescent(sysStatus);
    mixStatus mixFlightGliding(sysStatus);
    mixStatus mixAbort(sysStatus);

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
