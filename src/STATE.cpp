#include "STATE.h"

uav::uav(dataListString listIn[30], int len)
  :data(listIn,len)  
{ 
}

// This function will compute all the new "action" variables
void uav::compute() { 
  FLIGHTMODE newFlightMode;
  FLIGHTMODE oldFlightMode;
  // Run the state machine and check if there is a transition to another flight mode
  oldFlightMode = sys.get().flightMode;
  newFlightMode = flightState(); 
  sys.setTime(data.sen.get().time.code);
  sys.setFlightMode(newFlightMode);

  // We always output the output signal have them in the idle position even if we are not initialised
  sys.out.compute(sys.mix.compute(sys.get()));

  if (newFlightMode != oldFlightMode) {
    data.send(sys.get());
  }
}

void uav::output() {
  sys.out.write();
  // Data save is run in last because it is the most time consuming function and we have 50ms before the next sensor packet.
  data.save(sys.get());
}

void uav::update() {
  buzzer.update();
  led.update();
}


// Return the next flight mode based on current dataset
FLIGHTMODE uav::flightState() {

  FLIGHTMODE flightModeIn;
  FLIGHTMODE flightModeOut;

  flightModeIn = sys.get().flightMode;
  flightModeOut = INITIALIZE;

  if (data.cmd.isUpdated()) {
    flightModeOut = executeCmd(flightModeIn,data.cmd.get());
  }

  else { // Depending on which flight mode, various checks are carried out
    switch(flightModeIn) {
      case INITIALIZE:
        flightModeOut = flightInit();
      break;
      case READYSTEADY:
        flightModeOut = readySteady();
      break;
      case ASCENT:
        flightModeOut = flightAscent();
      break;
      case DESCENT:
        flightModeOut = flightDescent();
      break;
      case GLIDING:
        flightModeOut = flightGliding();
      break;
      default:
        flightModeOut = INITIALIZE;
      break;
    } 
  }

  if (flightModeOut != flightModeIn) {
    // Transition time is used all over the programm to know exactly when we did the last mode transition
    sys.setTransitionTime(sys.get().time);
  }
  led.switchColor(sys.get().flightMode);

  return flightModeOut;
}

// Wait for GPS to be ready
FLIGHTMODE uav::flightInit() {

  FLIGHTMODE flightModeOut;
  flightModeOut = INITIALIZE;

  if (data.get().sen.valid) {
    if (!sys.isInitialised()) {
      sys.setFlightMode(READYSTEADY);
    }
    else {
      flightModeOut = READYSTEADY;
    }
    sys.setReady();
  }
  return flightModeOut;
}

// Check to go in ascent or descent mode, this is just a transition mode, nothing special.
FLIGHTMODE uav::readySteady() { 

  FLIGHTMODE flightModeOut;
  flightModeOut = READYSTEADY;

  if (!data.get().sen.valid) {
    flightModeOut = INITIALIZE;
    return flightModeOut;
  }

  if (data.get().sen.speed.z > VUP) {
    flightModeOut = ASCENT;
  }
  if (data.get().sen.speed.z < VDOWN) {
    flightModeOut = DESCENT;
  }
  
  return flightModeOut;
}

// Ascent mode, if Z speed is lower than VUP go back to readysteady mode, if we reach the separation altitude, separate.
FLIGHTMODE uav::flightAscent() {

  FLIGHTMODE flightModeOut;
  flightModeOut = ASCENT;

  if (!data.get().sen.valid) {
    flightModeOut = INITIALIZE;
    return flightModeOut;
  }

  if (data.get().sen.speed.z < VUP) {
    flightModeOut = READYSTEADY;
  }

  return flightModeOut;
}

// Descent mode, if Z speed is higher than VDOWN go back to ready mode and deploy wing if needed.
FLIGHTMODE uav::flightDescent() {

  FLIGHTMODE flightModeOut;
  flightModeOut = DESCENT;

  if (!data.get().sen.valid) {
    flightModeOut = INITIALIZE;
    return flightModeOut;
  }
  
  if (data.get().sen.speed.z > VDOWN) {
    flightModeOut = READYSTEADY;
  }

  // We have two differents deployment mode, one using a target altitude, and one using a timer
  // If DEP_MODE is at 0, we will wait to be in decent mode for more than DESCENT_TIMER secondes before 
  // deploying the wing. If DEP_MODE is set to 1, we will wait until we go lower than a DEP_ALT to deploy. 
  // This DEP_ALT can be ASL or relative to ground level depending on the DEP_ALT_MODE setting.

  bool safetyTimer = (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= DESCENT_TIMER);

  switch (DEP_MODE) {

    case 0:
      if (safetyTimer) {
        flightModeOut = GLIDING;
      }
    break;

    case 1:
    // ASL altitude VS relative to ground. 
    switch (DEP_ALT_MODE) {

        case 0:
          if (safetyTimer and data.get().sen.position.alt < DEP_ALT) {
            flightModeOut = GLIDING;
          }
        break;

        default:
          if (safetyTimer and data.get().sen.position.alt < DEP_ALT) {
            flightModeOut = GLIDING;
          }
        break;

      }
    break;
  } 
  return flightModeOut;
}

// Gliding mode. 
FLIGHTMODE uav::flightGliding() {

  FLIGHTMODE flightModeOut;
  flightModeOut = GLIDING;
  return flightModeOut;
}

FLIGHTMODE uav::executeCmd(FLIGHTMODE flightModeIn, cmdStatus cmd) {
  FLIGHTMODE flightModeOut = INITIALIZE;
  switch(cmd.tlmCmd) {

    case TLM_CMD::SEPARATE_CMD:
      sys.separate();
      flightModeOut = flightModeIn;
    break;

    case TLM_CMD::DEPLOY_CMD:
      if (flightModeIn != ASCENT) {
      }
      else {
        flightModeOut = flightModeIn;
      }
    break;

    case TLM_CMD::PING_CMD:
      data.send(sys.get());
      flightModeOut = flightModeIn;
    break;

    case TLM_CMD::NO_CMD:
      flightModeOut = flightModeIn;
    break;

    default:
      flightModeOut = flightModeIn;
    break;
  }
  return flightModeOut;
}