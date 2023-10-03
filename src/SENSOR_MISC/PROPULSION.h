#ifndef PROPULSION_H
#define PROPULSION_H

#include "Arduino.h"
#include "MCP3427.h"

struct proStatus {
    uint16_t pressureN2O;
    uint16_t pressureFuel;
    uint16_t pressureChamber;
    float_t temperatureN2O;
};

class proClass {
    public:
        proClass();
        void read();
        void begin();
        proStatus get();
    private:
        MCP3427 adcUp;
        MCP3427 adcDown;
        unsigned pressureN2O;
        unsigned pressureFuel;
        unsigned pressureChamber;
        unsigned temperatureN2O;
};


#endif