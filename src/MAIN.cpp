#include "Arduino.h"
#include "STATE.h"
#include "CONFIG.h"

// This is the list of data that we want to save on the SD card, for more details, check DATA.c, print() function
dataListString list1[] = {DATE_TIME, TIME_CODE, BAT_VALUES, FLIGHT_MODE, SEN_HEALTH, 
                          POSITION_VALUES, ATTITUDE_VALUES, SPEED_VALUES, COURSE, TEMPERATURE, PRESSURE, 
                          MIX_VALUES, OUTPUT_VALUES}; 
                          
int len = sizeof(list1)/sizeof(list1[0]);

// uav, nordend is the main object of the code, everything is stored in this object. 
static uav nordend(list1, len);
const int chipSelect = BUILTIN_SDCARD;

// Normal arduino setup/loop functions
void setup() {
  Serial.begin(115200);

  senSettings basicSettings;
  nordend.data.sen.begin(basicSettings);
  nordend.data.com.begin();
  if (SD.begin(chipSelect)) {
  }
  // Must be called after SD.begin();
  nordend.data.begin();
}

void loop() {
  // The condition is true if the sensor has been updated since the last time we asked for it
  if (nordend.data.update()) {
    // .compute() will updated every single "action" variable in the uav object
    nordend.compute();
    // .output() will output the updated action dataset on the servos, led, buzzer, etc. 
    nordend.output();
  }
  nordend.update();
}
