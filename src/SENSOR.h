#ifndef SENSOR
#define SENSOR

#include "mtiSensor/xsens_mti.h"      // main library
#include "mtiSensor/xsens_utility.h"  // needed for quaternion conversion function
#include "Arduino.h"
#include "CONFIG.h"
#include <movingAvg.h>

struct senSettings {
  int heatingTime; // sec
  int noRotationTime; // sec
  int fusionFilter; // 11 or 13
  bool ahs; // true or false
  bool inRunCompassCalibration; // true or false
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
  gpsCoord position;
  xyzCoord speed; 
  double twoDSpeed;
  double threeDSpeed;
  rpyCoord attitude;
  double rotationSpeed;
  gpsStatus gps;
  double temperature, course;
  uint32_t pressure;
  bool valid;
  bool updated;
  double verticalSpeedAvgData;
  double rotationSpeedAvgData;
};

double timeDiff(timeCode time1, timeCode time2);

//void receive(XsensEventFlag_t event, XsensEventData_t *mtdata);
//void send(uint8_t *data, uint16_t length);

class senClass {
  public:
    senClass();
    bool update();
    senStatus get();
    bool isValid();
    void begin(senSettings settings);
    //void receive(XsensEventFlag_t event, XsensEventData_t *mtdata);
    //void send(uint8_t *data, uint16_t length);
    friend void receive(XsensEventFlag_t event, XsensEventData_t *mtdata);
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
    double temperature;
    uint32_t pressure;
    double rotationSpeed;
    double computeRotationSpeed(double, double);
    double course;
    double verticalSpeedAvgData;
    double rotationSpeedAvgData;
    xsens_interface_t sen_interface;
    bool updated;
    bool valid;
};


#endif 
