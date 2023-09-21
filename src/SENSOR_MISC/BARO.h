#ifndef BARO_H
#define BARO_H

#include "Arduino.h"
#include "CONFIG.h"

struct barStatus {
    double temperature;
    unsigned pressure;
};


class barClass {
    public:
        barClass();
        void begin();
        void update();
        barStatus get();
    private:
        double temperature;
        unsigned pressure;
};

#endif