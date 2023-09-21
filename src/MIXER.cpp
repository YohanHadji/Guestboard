#include "MIXER.h"

sysClass::sysClass() 
  :flightMode(INITIALIZE),
  initialised(false), separated(false), deployed(false), wingOpened(false), nearGround(false), realVDOWN(0) 
{
  time.second = 0;
  time.nanosecond = 0;
  timeTransition.second = 0;
  timeTransition.nanosecond = 0;
  trimDone = false;
}

sysStatus sysClass::get() {
  sysStatus sysOut;
  sysOut.time = time;
  sysOut.timeTransition = timeTransition;
  sysOut.flightMode = flightMode;
  sysOut.initialised = initialised;
  sysOut.separated = separated;
  sysOut.deployed = deployed;
  sysOut.wingOpened = wingOpened;
  sysOut.nearGround = nearGround;
  sysOut.realVDOWN = realVDOWN;
  sysOut.trimDone = trimDone;
  sysOut.mix = mix.get();
  sysOut.nav = nav.get();
  sysOut.ser = ser.get();
  sysOut.recoveryPhase = recoveryPhase;
  sysOut.gainDivider = gainDivider;
  return sysOut;
}

void sysClass::setReady() { initialised = true; }
void sysClass::separate() { separated = true; }
void sysClass::deploy()   { deployed = true; }
void sysClass::openWing() { wingOpened = true; }
void sysClass::isNearGround() { nearGround = true; }
void sysClass::setFlightMode(FLIGHTMODE flightModeIn) { flightMode = flightModeIn; }
void sysClass::setTransitionTime(timeCode timeIn) { timeTransition = timeIn; }
void sysClass::setTime(timeCode timeIn) { time = timeIn; }
void sysClass::setRealVDOWN(double realVDOWNIn) { realVDOWN = realVDOWNIn; }
void sysClass::trimIsDone() { trimDone = true; }
void sysClass::setRecoveryPhase(RECOVERYPHASE phaseIn) { recoveryPhase = phaseIn; } 
void sysClass::setGainDivider(double gainIn) { gainDivider = gainIn; }
bool sysClass::isInitialised() { return initialised; }

mixClass::mixClass() 
{
}

mixStatus mixClass::get() {
  mixStatus mixOut;
  mixOut.dir = dir;
  mixOut.brk = brk;
  mixOut.acc = acc;
  mixOut.dep = dep;
  return mixOut;
  //return {acc, dir, dep};
}

bool mixClass::isAutonomous(sysStatus sysIn) {
  if(sysIn.flightMode == GLIDINGAUTO) { return true; }
  return false;
}

// Return the command that matches the current flight mode
mixStatus mixClass::compute(sysStatus sysIn) {
  mixStatus mixOut;
  switch (sysIn.flightMode) {
    case INITIALIZE:
      mixOut = mixInit(sysIn);
    break;

    case READYSTEADY:
      mixOut = mixreadySteady(sysIn);
    break;

    case ASCENT:
      mixOut = mixAscent(sysIn);
    break;

    case DESCENT:
      mixOut = mixDescent(sysIn);
    break;

    case GLIDING:
      mixOut = mixGliding(sysIn);
    break;

    case GLIDINGAUTO:
      mixOut = mixGlidingAuto(sysIn);
    break;

    case SPIRAL:
      mixOut = mixSpiral(sysIn);
    break;

    case GLIDINGRECOVER:
      mixOut = mixGlidingRecover(sysIn);
    break;

    case GLIDINGNOGPS:
      mixOut = mixGlidingNoGPS(sysIn);
    break;

    case ABORT:
      mixOut = mixAbort(sysIn);
    break;

    case CONFIGERROR:
      mixOut = mixConfigError(sysIn);
    break;

    case FLYBYWIRE:
      mixOut = mixFlyByWire(sysIn);
    break;

    default:
      mixOut = mixInit(sysIn);
    break;
  }
  dir = mixOut.dir;
  brk = mixOut.brk;
  acc = mixOut.acc;
  dep = mixOut.dep;
  return mixOut;
}

// The system is on ground
mixStatus mixClass::mixInit(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_ASC});
  if (sysIn.separated) {
    mixOut.dep = DEP_CMD_SEP;
  }
  return mixOut;
}


// Transition mode 
mixStatus mixClass::mixreadySteady(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_ASC});
  if (sysIn.separated) {
    mixOut.dep = DEP_CMD_SEP;
  }
  return mixOut;
}

mixStatus mixClass::mixAscent(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_ASC});
  if (sysIn.separated) {
    mixOut.dep = DEP_CMD_SEP;
  }
  return mixOut;
}

mixStatus mixClass::mixDescent(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_ASC});
  if (sysIn.separated) {
    mixOut.dep = DEP_CMD_SEP;
  }
  return mixOut;
}

mixStatus mixClass::mixGliding(sysStatus sysIn) {
  mixStatus mixOut({0, BRK_CMD_TRIM, 0, DEP_CMD_BAG});
  return mixOut;
}

// The direction is chosen depending on the error between the current direction and the wanted direction 
mixStatus mixClass::mixGlidingAuto(sysStatus sysIn) {
  mixStatus mixOut;

  //mixOut.dir = map(sysIn.nav.con.output, -180, 180, -100, 100);
  double gainDivider = sysIn.gainDivider;
  gainDivider = constrain(gainDivider, 1, 10);
  mixOut.dir = sysIn.nav.con.output/gainDivider;

  static unsigned long timeAccApplied = 0;
  static bool accApplied = false;

  if (((sysIn.nav.navMode==HEADING_RELATIVE_FINAL) or (sysIn.nav.navMode == COG))  and (!sysIn.nearGround)) {
    if (!accApplied) {
      timeAccApplied = millis();
      accApplied = true;
    }
    // We want to have the accelerator applied 
    double limit = constrain(double(CUT_ACC_ON)/gainDivider, 0.1*CUT_ACC_ON, 1*CUT_ACC_ON);
    mixOut.dir = constrain(mixOut.dir, -limit, limit);

    double timeSinceTransition = (millis()-timeAccApplied)/1000.0;
    timeSinceTransition = constrain(timeSinceTransition,0,ACC_RAMP_TIMER);
    mixOut.acc = map(timeSinceTransition,0,ACC_RAMP_TIMER,0,ACC_MAX_CMD);
    mixOut.acc = constrain(mixOut.acc,0,100);

    mixOut.brk = 0;
  }
  else {
    if (accApplied) {
      timeAccApplied = millis();
      accApplied = false;
    }
    // We don't want the accelerator to be applied 
    double limit = constrain(double(CUT_ACC_OFF)/gainDivider, 0.1*CUT_ACC_OFF, 1*CUT_ACC_OFF);
    mixOut.dir = constrain(mixOut.dir, -limit, limit);
    
    double timeSinceTransition = (millis()-timeAccApplied)/1000.0;
    timeSinceTransition = constrain(timeSinceTransition,0,ACC_RAMP_TIMER);
    mixOut.acc = map(timeSinceTransition,0,ACC_RAMP_TIMER,ACC_MAX_CMD,0);
    mixOut.acc = constrain(mixOut.acc,0,100);

    mixOut.brk = 0;

    /* Serial.print("Limit : "); Serial.print(limit); 
    Serial.print("  gainDivider : "); Serial.print(gainDivider);
    Serial.print("  dir : "); Serial.println(mixOut.dir); */
  }

  mixOut.dep = DEP_CMD_BAG;
  return mixOut;
}

mixStatus mixClass::mixSpiral(sysStatus sysIn) {
  mixStatus mixOut({TURN_SPIRAL, 0, 0, DEP_CMD_BAG});
  return mixOut;
}

// The system need to be stabilized
mixStatus mixClass::mixGlidingRecover(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_BAG});
  switch (sysIn.recoveryPhase) {
    case STALLRAMP:
    {
      double timeSinceTransition = timeDiff(sysIn.time,sysIn.timeTransition);
      timeSinceTransition = constrain(timeSinceTransition,0,RECOVER_STALL_RAMP_TIMER);
      mixOut.brk = map(timeSinceTransition,0,RECOVER_STALL_RAMP_TIMER,0,BRK_CMD_STALL);
      mixOut.brk = constrain(mixOut.brk,0,100);
    }
    break;
    case STALL:
    {
      mixOut.brk = BRK_CMD_STALL;
    }
    break;

    case BRAKERAMP:
    {
      double timeSinceTransition = timeDiff(sysIn.time,sysIn.timeTransition);
      timeSinceTransition = constrain(timeSinceTransition,0,RECOVER_BRAKE_RAMP_TIMER);
      mixOut.brk = map(timeSinceTransition,0,RECOVER_BRAKE_RAMP_TIMER,BRK_CMD_STALL,BRK_CMD_BRAKE);
      mixOut.brk = constrain(mixOut.brk,0,100);
    }
    break;
    case BRAKE:
    {
      mixOut.brk = BRK_CMD_BRAKE;
    }
    break; 

    case HANDSUPRAMP:
    {
      double timeSinceTransition = timeDiff(sysIn.time,sysIn.timeTransition);
      timeSinceTransition = constrain(timeSinceTransition,0,RECOVER_HANDSUP_RAMP_TIMER);
      mixOut.brk = map(timeSinceTransition,0,RECOVER_HANDSUP_RAMP_TIMER,BRK_CMD_BRAKE,BRK_CMD_HANDSUP);
      mixOut.brk = constrain(mixOut.brk,0,100);
    }
    break;
    case HANDSUP:
    {
      mixOut.brk = BRK_CMD_HANDSUP;
      if (INDUCED_SPIRAL_IN_HANDSUP) {
        mixOut.dir = INDUCED_SPIRAL_IN_HANDSUP_CMD;
      }
    }
    break;

    default:
      mixOut.brk = BRK_CMD_HANDSUP;
    break;
  }
  return mixOut;
}

// The system is "blind" 
mixStatus mixClass::mixGlidingNoGPS(sysStatus sysIn) {
  mixStatus mixOut({0, 10, 0, DEP_CMD_BAG});
  return mixOut;
}

// On ground no commands are needed 
mixStatus mixClass::mixAbort(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_ASC});
  return mixOut;
}

mixStatus mixClass::mixConfigError(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_ASC});
  return mixOut;
}

mixStatus mixClass::mixFlyByWire(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, DEP_CMD_ASC});
  mixOut = mixGlidingAuto(sysIn);
  return mixOut;
}

