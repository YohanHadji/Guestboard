#ifndef SENSOR_H
#define SENSOR_H

#include "Arduino.h"
#include "CONFIG.h"

#include "SENSOR_MISC/GPS.h"
#include "SENSOR_MISC/BARO.h"
#include "SENSOR_MISC/PROPULSION.h"

struct senSettings {
};

struct timeCode {
  int second;
  int nanosecond;
};

struct vect3 {
  double a;
  double b;
  double c;
};

// GPS Coordinate, Latitude (° decimal), Longitude (° decimal), and Altitude (m)
struct gpsCoord {
  double lat;
  double lng;
  double alt;
};

// XYZ Coordinate
struct xyzCoord {
  double x;
  double y;
  double z;
};

// Roll Pitch Yaw Coordinate
struct rpyCoord {
  double roll;
  double pitch;
  double yaw;
};

struct gpsStatus {
  unsigned fixType;
  unsigned int hdop;
  unsigned int satNumber;
  bool isValid;
};

struct timeStruct {
  unsigned int year;
  unsigned int month;
  unsigned int day;
  unsigned int hour;
  unsigned int minute;
  unsigned int second;
  unsigned int nanosecond;
  timeCode code;
  bool isValid;
};

struct senStatus {
  timeStruct time;
  timeStruct externalTime;
  gpsCoord position;
  gpsCoord positionAux;
  xyzCoord speed; 
  double twoDSpeed;
  double threeDSpeed;
  rpyCoord attitude;
  double course;

  bool valid;
  bool updated;
  unsigned age;

  gpsStatus gpsMainStatus;
  gpsStatus gpsAuxStatus;
  barStatus baro;
  proStatus prop;

  unsigned msSinceMidnight;
};

double timeDiff(timeCode time1, timeCode time2);


class senClass {
  public:
    senClass();
    bool update();
    senStatus get();
    bool isValid();
    void begin(senSettings settings);
    friend void send(uint8_t *data, uint16_t length);
    double timeDiff(timeCode time1, timeCode time2);
  private:
    void config(senSettings settings);

    // GLOBAL ESTIMATOR VALUES // 
    timeStruct time;
    gpsCoord position;
    gpsCoord positionAux;
    xyzCoord speed;
    rpyCoord attitude;
    gpsStatus gps;
    gpsStatus gpsAuxStatus;
    double course;
    bool updated;
    bool valid;
    unsigned age;

    // RAW SENSOR VALUES // 
    barClass baro;
    proClass prop;
    SFE_UBLOX_GNSS_SERIAL gpsMain; 
    SFE_UBLOX_GNSS_SERIAL gpsAux;

    elapsedMillis msSinceMidnight;
};


#endif 
