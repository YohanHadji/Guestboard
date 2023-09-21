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
    FLIGHTMODE flightGlidingAuto();
    FLIGHTMODE flightSpiral();
    FLIGHTMODE flightGlidingRecover();
    FLIGHTMODE flightGlidingNoGPS();
    FLIGHTMODE abort();
    FLIGHTMODE configError();
    FLIGHTMODE flyByWire();

    dataClass data;
    sysClass  sys;

  private:
    buzzerClass buzzer;
    ledClass    led;
    ERRORCODE checkConfig();
    bool isAutonomous();
    FLIGHTMODE executeCmd(FLIGHTMODE, cmdStatus);
};

#endif 
