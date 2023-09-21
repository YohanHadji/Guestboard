#ifndef DATA_H
#define DATA_H

#include <Arduino.h>
#include <String.h>
#include <SD.h>
#include <SPI.h>
#include "CONFIG.h"
#include "MIXER.h"

struct batStatus {
    double voltage;
    double current;
};

struct dataStruct { 
  senStatus sen;
  comStatus com;
  batStatus bat;
};

class batClass {
    public:
    batClass();
    batStatus update();
    batStatus get();
    private:
    double voltage;
    double current;
    int pinVoltage;
};

enum dataListString {
    NULL_DATA = 0,
    FLIGHT_MODE,
    DATE_TIME,
    TIME_CODE,
    TIME_TRANSITION,
    INITIALIZED,
    SEPARATED,
    DEPLOYED,
    CHUTE_OPEN,
    SEN_HEALTH,                 //fixType, hdop, satNumber
    POSITION_VALUES,            //Lat, Lon, Alt
    ATTITUDE_VALUES,         //Roll, Pitch, Yaw
    SPEED_VALUES,               //X, Y, Z
    COURSE, 
    TEMPERATURE,
    PRESSURE, 
    MIX_VALUES,                 //dir, acc, dep
    OUTPUT_VALUES,               //l, r, x1, x2
    BAT_VALUES,
};

// Contains every single data field needed to fly the UAV 
class dataClass {
  public: 
    dataClass(dataListString list[], int);
    dataStruct get();
    bool update();
    void save(sysStatus);
    void send(sysStatus);
    void begin();
    String print(sysStatus);
    String printStarterString();
    bool isUpdated();

    senClass sen;
    comClass com;
    batClass bat;
    
  private:
    bool updated;
    bool firstTimeSave;
    File dataFile;
    File dataFileLowRate; 
    char namebuff[35]; 
    char namebuffLowRate[35];
    dataListString list[35];
    static void dateTime(uint16_t* date, uint16_t* time);
    void floatToByte(float f, byte *b);
    void int32ToByte(int32_t intIn, byte *b);
    void int16ToByte(int16_t intIn, byte *b);
    void int8ToByte(int8_t intIn, byte *b); 
};

#endif
