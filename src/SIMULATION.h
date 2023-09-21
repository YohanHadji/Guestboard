#ifndef SIMULATION
#define SIMULATION

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "PHY.h"
#include "CONFIG.h"

#define FIRST_SIZE 4
#define SECOND_SIZE 9

enum DATA_TYPE {
    WIND_U = 0,
    WIND_V,
    ALTITUDE
};

struct simStatus {
    double lat;
    double lng;
    int errorCode;
};

struct xyCoord {
    double x;
    double y;
};

struct worldLimit {
    double latMin;
    double latMax;
    double lngMin;
    double lngMax;
    double timeMin;
    double timeMax;
};

class simClass {
    public:
        simClass();
        simStatus run(double latIn, double lngIn, double altIn, double timeIn, double altBurst, double altFinalIn, double ascentSpeed, double descentSpeed);
        simStatus get();
        void parseWind();
        xyCoord computeWind(double lat, double lon, double alt, double time);
        int getHourModel();
        worldLimit getWorldLimit();
        simStatus runCircleSimulation(double latIn, double lngIn, double altIn, double timeIn, double altFinalIn, double headingIn, double busrtAlt);
        
    private:
        xyCoord interpWind(double xi, double yi, double zi, double ti);
        double interpAlt(double xi, double yi, double zi, double ti);
        double interp(double xi, double yi, double zi, double ti, DATA_TYPE type);

        int pCoord(double xi, double yi, double zi, double ti);
        int xCoord(double xi);
        int yCoord(double yi);
        int zCoord(double zi);
        int tCoord(double ti);

        int pCoordBis(double xi, double yi, double zi, double ti);
        int xCoordBis(double xi);
        int yCoordBis(double yi);
        int zCoordBis(double zi);
        int tCoordBis(double ti);

        double lat;
        double lng;
        double alt;
        double pressure;
        double temperature;
        double windU;
        double windV;
        double course;
        double heading;
        double altFinal;
        unsigned time;

        int metadata[2];
};

#endif