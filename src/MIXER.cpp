#include "MIXER.h"

sysClass::sysClass() 
  :flightMode(INITIALIZE_MODE),
  initialised(false), separated(false), deployed(false), chuteOpened(false)
{
  time.second = 0;
  time.nanosecond = 0;
  timeTransition.second = 0;
  timeTransition.nanosecond = 0;
}

sysStatus sysClass::get() {
  sysStatus sysOut;
  sysOut.time = time;
  sysOut.timeTransition = timeTransition;
  sysOut.flightMode = flightMode;
  sysOut.initialised = initialised;
  sysOut.separated = separated;
  sysOut.deployed = deployed;
  sysOut.chuteOpened = chuteOpened;
  sysOut.mix = mix.get();
  sysOut.out = out.get();
  return sysOut;
}
void sysClass::setReady() { initialised = true; }
void sysClass::separate() { separated = true; }
void sysClass::deploy()   { deployed = true; }
void sysClass::openChute() { chuteOpened = true; }
void sysClass::setFlightMode(FLIGHTMODE flightModeIn) { flightMode = flightModeIn; }
void sysClass::setTransitionTime(timeCode timeIn) { timeTransition = timeIn; }
void sysClass::setTime(timeCode timeIn) { time = timeIn; }
bool sysClass::isInitialised() { return initialised; }

mixClass::mixClass() 
{
}

mixStatus mixClass::get() {
  mixStatus mixOut;
  mixOut.solenoid1 = solenoid1;
  mixOut.solenoid2 = solenoid2;
  mixOut.solenoid3 = solenoid3;
  mixOut.solenoid4 = solenoid4;
  mixOut.servo1 = servo1;
  mixOut.servo2 = servo2;
  mixOut.ignitor = ignitor;
  mixOut.buzzer = buzzer;
  return mixOut;
}

// Return the command that matches the current flight mode
mixStatus mixClass::compute(sysStatus sysIn) {
  mixStatus mixOut;
  switch (sysIn.flightMode) {
    case FLIGHTMODE::INITIALIZE_MODE:
      mixOut = mixInit(sysIn);
    break;
    case FLIGHTMODE::READYSTEADY_MODE:
      mixOut = mixReadySteady(sysIn);
    break;
    case FLIGHTMODE::CALIBRATION_MODE:
      mixOut = mixCalibration(sysIn);
    break;
    case FLIGHTMODE::MANUAL_MODE:
      mixOut = mixManual(sysIn);
    break;
    case FLIGHTMODE::ARMED_MODE:
      mixOut = mixArmed(sysIn);
    break;
    case FLIGHTMODE::PRESSURED_MODE:
      mixOut = mixPressured(sysIn);
    break;
    case FLIGHTMODE::IGNITER_MODE:
      mixOut = mixIgniter(sysIn);
    break;
    case FLIGHTMODE::IGNITION_MODE:
      mixOut = mixIgnition(sysIn);
    break;
    case FLIGHTMODE::THRUST_MODE:
      mixOut = mixThrust(sysIn);
    break;
    case FLIGHTMODE::SHUTDOWN_MODE:
      mixOut = mixShutdown(sysIn);
    break;
    case FLIGHTMODE::ASCENT_MODE:
      mixOut = mixFlightAscent(sysIn);
    break;
    case FLIGHTMODE::DESCENT_MODE:
      mixOut = mixFlightDescent(sysIn);
    break;
    case FLIGHTMODE::GLIDING_MODE:
      mixOut = mixFlightGliding(sysIn);
    break;
    case FLIGHTMODE::ABORT_MODE:
      mixOut = mixAbort(sysIn);
    break;
    default:
      mixOut = mixInit(sysIn);
    break;
  }
  solenoid1 = mixOut.solenoid1;
  solenoid2 = mixOut.solenoid2;
  solenoid3 = mixOut.solenoid3;
  solenoid4 = mixOut.solenoid4;
  servo1 = mixOut.servo1;
  servo2 = mixOut.servo2;
  ignitor = mixOut.ignitor;
  buzzer = mixOut.buzzer;
  return mixOut;
}

// The system is on ground
mixStatus mixClass::mixInit(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  return mixOut;
}

mixStatus mixClass::mixReadySteady(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  return mixOut;
}

mixStatus mixClass::mixCalibration(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  return mixOut;
}

mixStatus mixClass::mixManual(sysStatus sysIn) {
  return mixManualMemory;
}

mixStatus mixClass::mixArmed(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  return mixOut;
}

// The engine is pressured and ready for ignition.
// Can move to IGNITER with the IGNITE command.
mixStatus mixClass::mixPressured(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  return mixOut;
}

// The igniter is fired.
// After IGNITER_COUNTER is elapsed, we move to IGNITION.
mixStatus mixClass::mixIgniter(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  mixOut.ignitor = IGNITOR_ACTIVE;
  return mixOut;
}


// Fuel and Oxydizer valves are partially opened.
// After IGNITION_COUNTER is elapsed, we move to THRUST.
mixStatus mixClass::mixIgnition(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  mixOut.solenoid1 = SOLENOID1_OPEN;
  mixOut.solenoid2 = SOLENOID2_OPEN;
  mixOut.solenoid3 = SOLENOID3_OPEN;
  mixOut.solenoid4 = SOLENOID4_OPEN;
  return mixOut;
}


// Fuel and Oxydizer valves are fully opened.
// After THRUST_COUNTER is elapsed, we move to SHUTDOWN.
mixStatus mixClass::mixThrust(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  mixOut.solenoid1 = SOLENOID1_OPEN;
  mixOut.solenoid2 = SOLENOID2_OPEN;
  mixOut.solenoid3 = SOLENOID3_OPEN;
  mixOut.solenoid4 = SOLENOID4_OPEN;
  return mixOut;
}

// Fuel Valve is closed.
// After SHUTDOWN_COUNTER is elapsed, we move to ASCENT.
mixStatus mixClass::mixShutdown(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0, 0, 0, 0, 0});
  mixOut.solenoid1 = !SOLENOID1_OPEN;
  mixOut.solenoid2 = !SOLENOID2_OPEN;
  mixOut.solenoid3 = !SOLENOID3_OPEN;
  mixOut.solenoid4 = !SOLENOID4_OPEN;
  return mixOut;
}

void mixClass::setManualMemory(mixStatus mixIn) {
  mixManualMemory = mixIn;
}




