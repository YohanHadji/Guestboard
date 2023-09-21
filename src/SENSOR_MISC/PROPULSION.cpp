#include "PROPULSION.h"

proClass::proClass() {
}

void proClass::begin() {
    PROPSENSOR_I2C_PORT.begin();
    adc.begin(PROPSENSOR_I2C_PORT,PROPSENSOR_I2C_ADDR);
}

void proClass::read() {
    MCP3427::ADCGain gain = MCP3427::GAIN_1X;
    MCP3427::ADCBitDepth bitDepth = MCP3427::ADC_12_BITS;

    pressureN2O = adc.analogRead(0, gain, bitDepth);
    pressureFuel = adc.analogRead(1, gain, bitDepth);
    pressureChamber = adc.analogRead(2, gain, bitDepth);
    temperatureN2O = adc.analogRead(3, gain, bitDepth);
}

proStatus proClass::get() {
    proStatus status;
    status.pressureFuel = pressureFuel;
    status.pressureChamber = pressureChamber;
    status.temperatureN2O = temperatureN2O;
    return status;
}