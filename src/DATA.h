#ifndef DATA
#define DATA

#include <Arduino.h>
#include <String.h>
#include <SD.h>
#include <SPI.h>
#include "CONFIG.h"
#include "NAVIGATION.h"
#include "COMMAND.h"
#include "MIXER.h"
#include "../R2HomeTelemetryInterface/PacketDefinition.h"


struct batStatus {
    double voltage;
    double current;
};

struct dataStruct { 
  senStatus sen;
  cmdStatus cmd;
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
    WING_OPEN,
    SEN_HEALTH,                 //fixType, hdop, satNumber
    POSITION_VALUES,            //Lat, Lon, Alt
    INCLINATION_VALUES,         //Roll, Pitch, Yaw
    SPEED_VALUES,               //X, Y, Z
    COURSE, 
    TEMPERATURE,
    PRESSURE, 
    VDOWN_VALUE,
    WAYPOINT_POSITION_VALUES,   //Lat, Lon, Alt
    WAYPOINT_SCORE,
    TRAJ_VALUES,                //Lat, Lon
    DIST_TRAJ,
    DIST_POS,
    NAV_VALUES,                 //input, setpoint, output
    MIX_VALUES,                 //dir, acc, dep
    SERVO_VALUES,               //l, r, x1, x2
    WIND_VALUES,
    FACTOR_VALUE,
    BAT_VALUES,
};

// Contains every single data field needed to fly the UAV 
class dataClass {
  public: 
    dataClass(dataListString list[], int);
    dataStruct get();
    void setWaypoint(gpsCoord);
    bool update();
    void save(sysStatus);
    void send(sysStatus);
    void begin();
    String print(sysStatus);
    String printWaypoint(sysStatus);
    String printStarterString();
    String printStarterStringWaypoint(sysStatus sysIn);
    bool isUpdated();

    senClass sen;
    cmdClass cmd;
    batClass bat;
    
  private:
    bool updated;
    bool firstTimeSave;
    File dataFile;
    File dataFileLowRate; 
    File missionFile;
    File waypointLogFile;
    char namebuff[35]; 
    char namebuffLowRate[35];
    char namebuffWaypointLog[35];
    dataListString list[35];
    static void dateTime(uint16_t* date, uint16_t* time);
    void floatToByte(float f, byte *b);
    void int32ToByte(int32_t intIn, byte *b);
    void int16ToByte(int16_t intIn, byte *b);
    void int8ToByte(int8_t intIn, byte *b); 
};

waypointData readWaypoint();
gpsCoord chooseWaypoint(gpsCoord waypointData[]);

#endif
