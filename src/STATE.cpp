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
  static timeCode lastSave;
  if (HIGH_RATE) {
    if (timeDiff(sys.get().time, lastSave) >= 1.0/HIGH_RATE_RATE) {
      data.save(sys.get());
      lastSave = sys.get().time;
    }
  }
}

void uav::update() {
  led.update();
}


// Return the next flight mode based on current dataset
FLIGHTMODE uav::flightState() {

  FLIGHTMODE flightModeIn;
  FLIGHTMODE flightModeOut;

  flightModeIn = sys.get().flightMode;
  flightModeOut = FLIGHTMODE::INITIALIZE_MODE;

  if (data.com.isUpdated()) {
    flightModeOut = executeCmd(flightModeIn,data.com.get());
    
    // This is NOT "required", but safer. We remove the command from the memory, 
    // so that there is ABSOLUTELY no way to execute it twice if somehow the 
    // "isUpdated" bit of data.com was flipped by mistake by a cosmic ray. 
    data.com.resetCmd();
  }

  else { // Depending on which flight mode, various checks are carried out
    switch(flightModeIn) {
      case FLIGHTMODE::INITIALIZE_MODE:
        flightModeOut = flightInit();
      break;
      case FLIGHTMODE::READYSTEADY_MODE:
        flightModeOut = readySteady();
      break;
      case FLIGHTMODE::CALIBRATION_MODE:
        flightModeOut = calibration();
      break;
      case FLIGHTMODE::MANUAL_MODE:
        flightModeOut = manual();
      break;
      case FLIGHTMODE::ARMED_MODE:
        flightModeOut = armed();
      break;
      case FLIGHTMODE::PRESSURED_MODE:
        flightModeOut = pressured();
      break;
      case FLIGHTMODE::IGNITER_MODE:
        flightModeOut = igniter();
      break;
      case FLIGHTMODE::IGNITION_MODE:
        flightModeOut = ignition();
      break;
      case FLIGHTMODE::THRUST_MODE:
        flightModeOut = thrust();
      break;
      case FLIGHTMODE::SHUTDOWN_MODE:
        flightModeOut = shutdown();
      break;
      case FLIGHTMODE::ASCENT_MODE:
        flightModeOut = flightAscent();
      break;
      case FLIGHTMODE::DESCENT_MODE:
        flightModeOut = flightDescent();
      break;
      case FLIGHTMODE::GLIDING_MODE:
        flightModeOut = flightGliding();
      break;
      case FLIGHTMODE::ABORT_MODE:
        flightModeOut = abort();
      break;
      default:
        flightModeOut = FLIGHTMODE::INITIALIZE_MODE;
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
  flightModeOut = FLIGHTMODE::INITIALIZE_MODE;

  if (data.get().sen.valid) {
    if (!sys.isInitialised()) {
      sys.setReady();
    }
    flightModeOut = FLIGHTMODE::READYSTEADY_MODE;
  }
  return flightModeOut;
}

// Check to go in ascent or descent mode, this is just a transition mode, nothing special.
FLIGHTMODE uav::readySteady() { 

  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::READYSTEADY_MODE;

  if (!data.get().sen.valid) {
    flightModeOut = FLIGHTMODE::INITIALIZE_MODE;
    return flightModeOut;
  }  
  return flightModeOut;
}

// Calibration of engine's sensors.
FLIGHTMODE uav::calibration() {
    FLIGHTMODE flightModeOut;
    flightModeOut = FLIGHTMODE::CALIBRATION_MODE;
  
    if (!data.get().sen.valid) {
      flightModeOut = FLIGHTMODE::INITIALIZE_MODE;
      return flightModeOut;
    }
  
    return flightModeOut;
}

// Working state for when the engine is manually controlled. 
// Whenever the engine receives a manual operation command, it gets locked to this state. 
// This is meant to avoid messing up the sequence, when manual valve operation is used.
// Can move back to IDLE by issuing the RECOVER command.
FLIGHTMODE uav::manual() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::MANUAL_MODE;
  return flightModeOut;
}

// From this state, the system can be safely pressured.
// Can move to PRESSURED with the PRESSURE command.
FLIGHTMODE uav::armed() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::ARMED_MODE;
  return flightModeOut;
}

// The engine is pressured and ready for ignition.
// Can move to IGNITER with the IGNITE command.
FLIGHTMODE uav::pressured() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::PRESSURED_MODE;
  return flightModeOut;
}

// The igniter is fired.
// After IGNITER_COUNTER is elapsed, we move to IGNITION.
FLIGHTMODE uav::igniter() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::IGNITER_MODE;

  if (timeDiff(sys.get().time, sys.get().timeTransition) >= IGNITER_COUNTER) {
    flightModeOut = FLIGHTMODE::IGNITION_MODE;
  }
  return flightModeOut;
}

// Fuel and Oxydizer valves are partially opened.
// After IGNITION_COUNTER is elapsed, we move to THRUST.
FLIGHTMODE uav::ignition() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::IGNITION_MODE;

  if (timeDiff(sys.get().time,sys.get().timeTransition) >= IGNITION_COUNTER) {
    flightModeOut = FLIGHTMODE::THRUST_MODE;
  }
  return flightModeOut;
}

// Fuel and Oxydizer valves are fully opened.
// After THRUST_COUNTER is elapsed, we move to SHUTDOWN.
FLIGHTMODE uav::thrust() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::THRUST_MODE;

  if (timeDiff(sys.get().time,sys.get().timeTransition) >= THRUST_COUNTER) {
    flightModeOut = FLIGHTMODE::SHUTDOWN_MODE;
  }
  return flightModeOut;
}


// Fuel Valve is closed.
// After SHUTDOWN_COUNTER is elapsed, we move to ASCENT.
FLIGHTMODE uav::shutdown() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::SHUTDOWN_MODE;

  if (timeDiff(sys.get().time,sys.get().timeTransition) >= SHUTDOWN_COUNTER) {
    flightModeOut = FLIGHTMODE::ASCENT_MODE;
  }
  return flightModeOut;
}

// Ascent mode, if Z speed is lower than VDOWN go to descent mode.
FLIGHTMODE uav::flightAscent() {

  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::ASCENT_MODE;

  if (!data.get().sen.valid) {
    flightModeOut = FLIGHTMODE::INITIALIZE_MODE;
    return flightModeOut;
  }

  if (data.get().sen.speed.z < VDOWN) {
    flightModeOut = FLIGHTMODE::DESCENT_MODE;
  }
  return flightModeOut;
}

// Descent mode, if Z speed is higher than VDOWN go back to ascent mode. Otherwise we deploy chute if needed.
FLIGHTMODE uav::flightDescent() {

  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::DESCENT_MODE;

  if (!data.get().sen.valid) {
    flightModeOut = FLIGHTMODE::INITIALIZE_MODE;
    return flightModeOut;
  }
  
  if (data.get().sen.speed.z > VDOWN) {
    flightModeOut = FLIGHTMODE::ASCENT_MODE;
  }

  // We have two differents deployment mode, one using a target altitude, and one using a timer
  // If DEP_MODE is at 0, we will wait to be in decent mode for more than DESCENT_TIMER secondes before 
  // deploying the wing. If DEP_MODE is set to 1, we will wait until we go lower than a DEP_ALT to deploy. 
  // This DEP_ALT can be ASL or relative to ground level depending on the DEP_ALT_MODE setting.

  bool safetyTimer = (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= DESCENT_TIMER);

  switch (DEP_MODE) {

    case 0:
      if (safetyTimer) {
        flightModeOut = FLIGHTMODE::GLIDING_MODE;
      }
    break;

    case 1:
    // ASL altitude VS relative to ground. 
    switch (DEP_ALT_MODE) {

        case 0:
          if (safetyTimer and data.get().sen.position.alt < DEP_ALT) {
            flightModeOut = FLIGHTMODE::GLIDING_MODE;
          }
        break;

        default:
          if (safetyTimer and data.get().sen.position.alt < DEP_ALT) {
            flightModeOut = FLIGHTMODE::GLIDING_MODE;
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
  flightModeOut = FLIGHTMODE::GLIDING_MODE;
  return flightModeOut;
}

// Abort mode.
FLIGHTMODE uav::abort() {

  FLIGHTMODE flightModeOut;
  flightModeOut = FLIGHTMODE::ABORT_MODE;
  return flightModeOut;
}

FLIGHTMODE uav::executeCmd(FLIGHTMODE flightModeIn, comStatus com) {

  FLIGHTMODE flightModeOut = FLIGHTMODE::INITIALIZE_MODE;

  switch(com.cmdId) {

    case CMD_ID::AV_CMD_VALVE_FUEL:
    {
      flightModeOut = FLIGHTMODE::MANUAL_MODE;
      mixStatus currentMix = sys.mix.get();
      if (com.cmdValue == OPEN) {
        currentMix.servoFuel = SERVO_FUEL_OPEN;
      }
      else if (com.cmdValue == CLOSED) {
        currentMix.servoFuel = SERVO_FUEL_CLOSED;
      }
      sys.mix.setManualMemory(currentMix);
    }
    break;

    case CMD_ID::AV_CMD_VALVE_N2O:
    {
      flightModeOut = FLIGHTMODE::MANUAL_MODE;
      mixStatus currentMix = sys.mix.get();
      if (com.cmdValue == OPEN) {
        currentMix.servoN2O = SERVO_N2O_OPEN;
      }
      else if (com.cmdValue == CLOSED) {
        currentMix.servoN2O = SERVO_N2O_CLOSED;
      }
      sys.mix.setManualMemory(currentMix);
    }
    break;

    case CMD_ID::AV_CMD_VENT_FUEL:
    {
      flightModeOut = FLIGHTMODE::MANUAL_MODE;
      mixStatus currentMix = sys.mix.get();
      if (com.cmdValue == OPEN) {
        currentMix.ventFuel = VENT_FUEL_OPEN;
      }
      else if (com.cmdValue == CLOSED) {
        currentMix.ventFuel = VENT_FUEL_CLOSED;
      }
      sys.mix.setManualMemory(currentMix);
    }
    break;

    case CMD_ID::AV_CMD_VENT_N2O:
    {
      flightModeOut = FLIGHTMODE::MANUAL_MODE;
      mixStatus currentMix = sys.mix.get();

      // "OPEN" is the value transmitted by GS
      if (com.cmdValue == OPEN) {
        // VENT_N2O_OPEN is the logic level in CONFIG.h to open the valve
        currentMix.ventN2O = VENT_N2O_OPEN;
      }
      else if (com.cmdValue == CLOSED) {
        currentMix.ventN2O = VENT_N2O_CLOSED;
      }
      sys.mix.setManualMemory(currentMix);
    }
    break;

    case CMD_ID::MANUAL:
    {
      flightModeOut = FLIGHTMODE::MANUAL_MODE;
      mixStatus currentMix = sys.mix.get();
      sys.mix.setManualMemory(currentMix);
    }
    break;

    case CMD_ID::ARMED:
    {
      if (flightModeIn == FLIGHTMODE::READYSTEADY_MODE) {
        flightModeOut = FLIGHTMODE::ARMED_MODE;
      }
      else {
        flightModeOut = flightModeIn;
      }
    }
    break;

    case CMD_ID::PRESSURISED:
    {
      if (flightModeIn == FLIGHTMODE::ARMED_MODE) {
        flightModeOut = FLIGHTMODE::PRESSURED_MODE;
      }
      else {
        flightModeOut = flightModeIn;
      }
    }
    break;

    case CMD_ID::IGNITION:
    {
      if (flightModeIn == FLIGHTMODE::PRESSURED_MODE) {
        flightModeOut = FLIGHTMODE::IGNITER_MODE;
      }
      else {
        flightModeOut = flightModeIn;
      }
    }
    break;

    case CMD_ID::ABORT:
      flightModeOut = FLIGHTMODE::ABORT_MODE;
    break;


    default:
      flightModeOut = flightModeIn;
    break;
  }
  return flightModeOut;
}