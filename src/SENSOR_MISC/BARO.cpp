#include "BARO.h"

barClass::barClass() {
    temperature = 0;
    pressure = 0;
}

void barClass::begin() {
}

void barClass::update() {
    temperature = 0;
    pressure = 0;
}

barStatus barClass::get() {
    barStatus barOut;
    barOut.temperature = temperature;
    barOut.pressure = pressure;
    return barOut;
}