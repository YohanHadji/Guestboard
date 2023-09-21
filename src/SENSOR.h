#ifndef SENSOR_H
#define SENSOR_H

#include "Arduino.h"
#include "CONFIG.h"

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
  unsigned int vAcc;
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
  xyzCoord speed; 
  double twoDSpeed;
  double threeDSpeed;
  rpyCoord attitude;
  gpsStatus gps;
  double course;
  double temperature;
  uint32_t pressure;
  bool valid;
  bool updated;
  elapsedMillis msSinceMidnight;
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
    void resetVerticalSpeed();
  private:
    void config(senSettings settings);
    void setNoRotation(int16_t timeForNoRotation);
    void printReceived();
    timeStruct time;
    gpsCoord position;
    xyzCoord speed;
    rpyCoord attitude;
    gpsStatus gps;
    double course;
    double temperature;
    uint32_t pressure;
    bool updated;
    bool valid;
};


#endif 
