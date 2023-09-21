#include "PHY.h"

// Compute standard density in kg/m^2 as function of pressure in Pa and Temperature in °C
double density(double pressure, double temperature) {
    double density = pressure / (287.058 * (temperature + 273.15));
    return density;
}

double trueAirspeed(double airspeed, double pressure, double temperature) {
    double trueAirspeedValue = airspeed * sqrt(1.225/density(pressure, temperature));
    return trueAirspeedValue;
}

double falseAirspeed(double airspeed, double pressure, double temperature) {
    double falseAirspeedValue = airspeed / sqrt(1.225/density(pressure, temperature));
    return falseAirspeedValue;
}


double getAngle(double angle1, double angle2) {
    double angle = angle2 - angle1;
    if (angle > 180) {
        angle -= 360;
    }
    if (angle < -180) {
        angle += 360;
    }
    return angle;
}

void enu2lla(double E, double N, double U, double &lat, double &lng, double &alt) {   
    double R =  6378100 + alt;
    double dlat = (180.0/PI) * (N/R);
    double dlng = (180.0/PI) * (E/(R*cos(lat*(PI/180.0))));
    lat = lat + dlat;
    lng = lng + dlng;
    alt = alt + U;
}

double pressureSim(double alt) {
  float temp = 0.0;
  double pressure = 0.0;
  if (alt > 25000) {
    temp = -131.21 + 0.00299 * alt;
    pressure =  2.488 * pow(((temp + 273.1)/(216.6)),(-11.388));
    return pressure * 1000.0;
  }
  else if (alt >11000 and alt <= 25000) {
    temp = -56.46;
    pressure = 22.65 * exp(1.73 - (0.000157 * alt));
    return pressure * 1000.0;
  }
  else {
    temp = 15.04 - 0.00649 * alt;
    pressure = 101.29 * pow(((temp + 273.1)/288.08),(5.256));
    return pressure * 1000.0;
  }
}

double temperatureSim(double alt) {
  double temp = 0.0;
  if (alt > 25000) {
    temp = -131.21 + 0.00299 * alt;
    return temp;
  }
  else if (alt >11000 and alt <= 25000) {
    temp = -56.46;
    return temp;
  }
  else {
    temp = 15.04 - 0.00649 * alt;
    return temp;
  }
}
