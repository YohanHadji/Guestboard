#ifndef PHY_H
#define PHY_H

#include "Arduino.h"
#include "CONFIG.h"
#include "math.h"

void enu2lla(double E, double N, double U, double &lat, double &lng, double &alt);
double trueAirspeed(double airspeed, double pressure, double temperature);
double falseAirspeed(double airspeed, double pressure, double temperature);
double density(double pressure, double temperature);
double getAngle(double, double);
double pressureSim(double alt);
double temperatureSim(double alt);

#endif 