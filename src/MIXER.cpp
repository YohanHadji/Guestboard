#include "MIXER.h"

sysClass::sysClass() 
  :flightMode(INITIALIZE),
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
    case INITIALIZE:
      mixOut = mixInit(sysIn);
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

