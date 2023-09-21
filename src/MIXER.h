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

enum ERRORCODE {
  NOERROR = 0,
  NOWINDFILE,
  NOWAYPOINTFILE,
  WAYPOINTRANGE,
  POSRANGE,
  TIMERANGE,
  TIMEWAIT
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
    double acc;
    double dir;
    double dep;
    double brk;
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
