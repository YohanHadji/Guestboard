#include "STATE.h"
#include "Arduino.h"
#include "CONFIG.h"

// This is the list of data that we want to save on the SD card, for more details, check DATA.c, print() function
dataListString list1[] = {DATE_TIME, TIME_CODE, BAT_VALUES, FLIGHT_MODE, SEN_HEALTH, 
                          POSITION_VALUES, INCLINATION_VALUES, SPEED_VALUES, COURSE, TEMPERATURE, PRESSURE, 
                          VDOWN_VALUE, WAYPOINT_POSITION_VALUES, TRAJ_VALUES, DIST_TRAJ, NAV_VALUES, WIND_VALUES, FACTOR_VALUE, 
                          MIX_VALUES, SERVO_VALUES}; 
                          
int len = sizeof(list1)/sizeof(list1[0]);

// uav, r2home is the main object of the code, everything is stored in this object. 
static uav r2home(list1, len);
const int chipSelect = BUILTIN_SDCARD;

// Normal arduino setup/loop functions
void setup() {
  pinMode(BARO_PIN, OUTPUT);
  digitalWrite(BARO_PIN, HIGH); 
  Serial.begin(115200);

  senSettings basicSettings;
  basicSettings.heatingTime = 5;
  basicSettings.noRotationTime = 10;
  basicSettings.fusionFilter = 13;
  basicSettings.ahs = true;
  basicSettings.inRunCompassCalibration = true;

  r2home.data.sen.begin(basicSettings);

  r2home.data.cmd.begin();
  if (SD.begin(chipSelect)) {
    // This function will read the wind file and store the wind matrix in the uav object
    r2home.sys.nav.sim.parseWind();
    // This fonction will read the waypoints file and store the waypoints in the uav object
    r2home.sys.nav.setWaypoint(readWaypoint());
  }
  // Must be called after SD.begin();
  r2home.data.begin();
}

void loop() {
  // The condition is true if the sensor has been updated since the last time we asked for it
  if (r2home.data.update()) {
    // .compute() will updated every single "action" variable in the uav object
    r2home.compute();
    // .output() will output the updated action dataset on the servos, led, buzzer, etc. 
    r2home.output();
  }
  r2home.update();
}
