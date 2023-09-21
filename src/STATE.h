#ifndef STATE_H
#define STATE_H

#include "FEEDBACK.h"
#include "Arduino.h"
#include "DATA.h"
#include "CONFIG.h"

class uav {
  public:
    uav(dataListString listIn[30], int len);
    void output();
    void compute();
    void update();
    FLIGHTMODE flightState();
    
    // Flight mode functions
    FLIGHTMODE flightInit();
    FLIGHTMODE readySteady();
    FLIGHTMODE flightAscent();
    FLIGHTMODE flightDescent();
    FLIGHTMODE flightGliding();

    dataClass data;
    sysClass  sys;

  private:
    buzzerClass buzzer;
    ledClass    led;
    FLIGHTMODE executeCmd(FLIGHTMODE, cmdStatus);
};

#endif 
