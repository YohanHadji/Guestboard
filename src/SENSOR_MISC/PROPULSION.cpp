#include "PROPULSION.h"

proClass::proClass() {
}

void proClass::begin() {
    PROPSENSOR_I2C_PORT.begin();
    adc.begin(PROPSENSOR_I2C_PORT,PROPSENSOR_I2C_ADDR);
}

void proClass::read() {
    MCP3427::ADCGain gain = MCP3427::GAIN_8X;
    MCP3427::ADCBitDepth bitDepth = MCP3427::ADC_12_BITS;

    pressure1 = adc.analogRead(0, gain, bitDepth);
    pressure2 = adc.analogRead(1, gain, bitDepth);
    temperature1 = adc.analogRead(2, gain, bitDepth);
}

proStatus proClass::get() {
    proStatus status;
    status.pressure1 = pressure1;
    status.pressure2 = pressure2;
    status.temperature1 = temperature1;
    return status;
}