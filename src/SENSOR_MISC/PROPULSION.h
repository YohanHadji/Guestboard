#ifndef PROPULSION_H
#define PROPULSION_H

#include "Arduino.h"
#include "MCP3427.h"

struct proStatus {
    float pressureN2O;
    float pressureFuel;
    float pressureChamber;
    float temperatureN2O;
};

class proClass {
    public:
        proClass();
        void readAll();
        void readChannel(int channel);
        void begin();
        proStatus get();
    private:
        MCP3427 adcUp;
        MCP3427 adcDown;
        float pressureN2O;
        float pressureFuel;
        float pressureChamber;
        float temperatureN2O;
};

float convertToPressure(float voltageSensor);

#endif