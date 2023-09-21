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
  mixOut.dir = dir;
  mixOut.brk = brk;
  mixOut.acc = acc;
  mixOut.dep = dep;
  return mixOut;
  //return {acc, dir, dep};
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
  dir = mixOut.dir;
  brk = mixOut.brk;
  acc = mixOut.acc;
  dep = mixOut.dep;
  return mixOut;
}

// The system is on ground
mixStatus mixClass::mixInit(sysStatus sysIn) {
  mixStatus mixOut({0, 0, 0, 0});
  if (sysIn.separated) {
  }
  return mixOut;
}

