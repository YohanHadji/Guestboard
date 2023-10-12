#include "CONFIG.H"
#include "PROPULSION.h"

proClass::proClass() {
}

void proClass::begin() {
    PROPSENSOR_UP_I2C_PORT.begin();
    PROPSENSOR_DOWN_I2C_PORT.begin();
    if(adcUp.begin(PROPSENSOR_UP_I2C_PORT,PROPSENSOR_UP_I2C_ADDR)) {
        if (DEBUG) {
            Serial.println("ADC Up initialized");
        }
    } else {
        if (DEBUG) {
            Serial.println("ADC Up failed to initialize");
        }
    }
    if (adcDown.begin(PROPSENSOR_DOWN_I2C_PORT,PROPSENSOR_DOWN_I2C_ADDR)) {
        if (DEBUG) {
            Serial.println("ADC Down initialized");
        }
    } else {
        if (DEBUG) {
            Serial.println("ADC Down failed to initialize");
        }
    }
}

void proClass::readAll() {
    MCP3427::ADCGain gain = MCP3427::GAIN_1X;
    MCP3427::ADCBitDepth bitDepth = MCP3427::ADC_12_BITS;

    pressureN2O = convertToPressure(adcUp.analogReadVoltage(0, gain, bitDepth));
    pressureFuel = convertToPressure(adcUp.analogReadVoltage(1, gain, bitDepth));
    temperatureN2O = convertToPressure(adcDown.analogReadVoltage(0, gain, bitDepth));
    pressureChamber = convertToPressure(adcDown.analogReadVoltage(1, gain, bitDepth));
}

void proClass::readChannel(int channel) {
    MCP3427::ADCGain gain = MCP3427::GAIN_1X;
    MCP3427::ADCBitDepth bitDepth = MCP3427::ADC_12_BITS;

    switch (channel) {
        case 0:
            pressureN2O = convertToPressure(adcUp.analogRead(0, gain, bitDepth));
            // pressureN2O = adcUp.analogRead(0, gain, bitDepth);
        break;

        case 1:
            pressureFuel = convertToPressure(adcUp.analogRead(1, gain, bitDepth));
            // pressureFuel = adcUp.analogRead(1, gain, bitDepth);
        break;

        case 2:
        {
            float voltageMeasured = adcDown.analogRead(0, gain, bitDepth);
            // float resistanceMeasured = 5100.0*voltageMeasured/(5000.0-voltageMeasured);
            temperatureN2O = -125+(-0.282*voltageMeasured)+(0.00215*voltageMeasured*voltageMeasured);
        }
        break;

        case 3:
            pressureChamber = convertToPressure(adcDown.analogRead(1, gain, bitDepth));
            // pressureChamber = adcDown.analogRead(1, gain, bitDepth);
        break;

        default:
            if (DEBUG) {
                Serial.println("Invalid channel");
            }
        break;


    }
}

proStatus proClass::get() {
    proStatus status;
    status.pressureFuel = pressureFuel;
    status.pressureN2O = pressureN2O;
    status.pressureChamber = pressureChamber;
    status.temperatureN2O = temperatureN2O;
    return status;
}

float convertToPressure(float voltageSensor) {
    // float deltaP = 60.0 - 1.0; // 60.0 = Max pressure, 1.0 = Min pressure
    // float vMax = 0.8 * 5.0; // 5.0 = Voltage supply of the sensor
    // float vMin = 0.1 * 5.0; // 5.0 = Voltage supply of the sensor
    return (voltageSensor-200.0)*((0.360-0.200)/4.8);
}