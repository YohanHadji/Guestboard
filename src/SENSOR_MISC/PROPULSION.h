#ifndef PROPULSION_H
#define PROPULSION_H

#include "Arduino.h"
#include "CONFIG.h"
#include "MCP3427.h"

struct proStatus {
    unsigned pressure1;
    unsigned pressure2;
    unsigned temperature1;
};

class proClass {
    public:
        proClass();
        void read();
        void begin();
        proStatus get();
    private:
        MCP3427 adc;
        unsigned pressure1;
        unsigned pressure2;
        unsigned temperature1;
};


#endif