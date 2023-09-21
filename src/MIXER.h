#ifndef MIXER
#define MIXER

#include "Arduino.h"
#include "CONFIG.h"
#include "SERVO.h"
#include "COMMAND.h"
#include "NAVIGATION.h"

enum FLIGHTMODE {
INITIALIZE = 0, 
READYSTEADY, 
ASCENT, 
DESCENT, 
GLIDING, 
GLIDINGAUTO, 
SPIRAL,
GLIDINGRECOVER, 
GLIDINGNOGPS, 
ABORT,
CONFIGERROR,
FLYBYWIRE
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

enum RECOVERYPHASE {
  STALLRAMP = 0,
  STALL,
  BRAKERAMP,
  BRAKE,
  HANDSUPRAMP,
  HANDSUP
};

struct sysStatus {
  timeCode time;
  timeCode timeTransition;
  FLIGHTMODE flightMode; 
  bool initialised;
  bool separated;
  bool deployed;
  bool wingOpened;
  bool nearGround;
  double realVDOWN;
  bool trimDone;
  RECOVERYPHASE recoveryPhase;
  double gainDivider;

  mixStatus mix;
  navStatus nav;
  serStatus ser;
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
    bool isAutonomous(sysStatus);
    mixStatus mixInit(sysStatus);
    mixStatus mixreadySteady(sysStatus);
    mixStatus mixAscent(sysStatus);
    mixStatus mixDescent(sysStatus);
    mixStatus mixGliding(sysStatus);
    mixStatus mixGlidingAuto(sysStatus);  
    mixStatus mixSpiral(sysStatus);  
    mixStatus mixGlidingRecover(sysStatus);
    mixStatus mixGlidingNoGPS(sysStatus);
    mixStatus mixConfigError(sysStatus);
    mixStatus mixAbort(sysStatus);
    mixStatus mixFlyByWire(sysStatus);
};

class sysClass {
  public:
    sysClass();
    sysStatus get();
    bool isInitialised();
    void setFlightMode(FLIGHTMODE);
    void setTransitionTime(timeCode);
    void setTime(timeCode);
    void setRealVDOWN(double);
    void setReady();
    void separate();
    void deploy();
    void openWing();
    void isNearGround();
    void trimIsDone();
    void setRecoveryPhase(RECOVERYPHASE phaseIn);
    void setGainDivider(double gainIn);

    mixClass mix;
    navClass nav;
    serClass ser;

  private: 
    FLIGHTMODE flightMode; 
    timeCode time;
    timeCode timeTransition;
    bool initialised;
    bool separated;
    bool deployed;
    bool wingOpened;
    bool nearGround;
    double realVDOWN;
    bool trimDone;
    RECOVERYPHASE recoveryPhase;
    double gainDivider;
};

#endif 
