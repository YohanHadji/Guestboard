#include "CONFIG.H"
#include "PROPULSION.h"

proClass::proClass() {
}

void proClass::begin() {
    PROPSENSOR_UP_I2C_PORT.begin();
    PROPSENSOR_DOWN_I2C_PORT.begin();
    if(adcUp.begin(PROPSENSOR_UP_I2C_PORT,PROPSENSOR_UP_I2C_ADDR)) {
        Serial.println("ADC Up initialized");
    } else {
        Serial.println("ADC Up failed to initialize");
    }
    if (adcDown.begin(PROPSENSOR_DOWN_I2C_PORT,PROPSENSOR_DOWN_I2C_ADDR)) {
        Serial.println("ADC Down initialized");
    } else {
        Serial.println("ADC Down failed to initialize");
    }
}

void proClass::read() {
    MCP3427::ADCGain gain = MCP3427::GAIN_1X;
    MCP3427::ADCBitDepth bitDepth = MCP3427::ADC_12_BITS;

    pressureN2O = adcUp.analogReadVoltage(0, gain, bitDepth);
    pressureFuel = adcUp.analogReadVoltage(1, gain, bitDepth);
    pressureChamber = adcDown.analogReadVoltage(0, gain, bitDepth);
    temperatureN2O = adcDown.analogReadVoltage(1, gain, bitDepth);
}

proStatus proClass::get() {
    proStatus status;
    status.pressureFuel = pressureFuel;
    status.pressureN2O = pressureN2O;
    status.pressureChamber = pressureChamber;
    status.temperatureN2O = temperatureN2O;
    return status;
}