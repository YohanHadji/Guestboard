#include "Arduino.h"
#include "STATE.h"
#include "CONFIG.h"

// This is the list of data that we want to save on the SD card, for more details, check DATA.c, print() function
// dataListString list1[] = {DATE_TIME, TIME_CODE, BAT_VALUES, FLIGHT_MODE, SEN_HEALTH, 
//                           POSITION_VALUES, ATTITUDE_VALUES, SPEED_VALUES, COURSE, TEMPERATURE, PRESSURE, 
//                           OUTPUT_VALUES}; 

dataListString list1[] = {TIME_CODE, FLIGHT_MODE, PROP_VALUES, POSITION_VALUES, SPEED_VALUES,
                          OUTPUT_VALUES}; 
                          
int len = sizeof(list1)/sizeof(list1[0]);

// Adafruit_NeoPixel ledA(1, LED_A_PIN, NEO_GRB + NEO_KHZ800); // 1 led
// Adafruit_NeoPixel ledB(1, LED_B_PIN, NEO_GRB + NEO_KHZ800); // 1 led

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
    nordend.data.setSDIsPresent();
  }
  else {
    nordend.data.setSDIsNotPresent();
  }
  // Must be called after SD.begin();
  nordend.data.begin();


  // { 
  //   ledA.begin();
  //   ledA.rainbow(10);
  //   ledA.show();
  // }

  // { 
  //   ledB.begin();
  //   ledB.rainbow(10);
  //   ledB.show();
  // }

  pinMode(CAM_TOP_PIN, OUTPUT);
  digitalWrite(CAM_TOP_PIN, HIGH);
  delay(10000);
  digitalWrite(CAM_TOP_PIN, LOW);
  delay(200);
  digitalWrite(CAM_TOP_PIN, HIGH);

}


void loop() {
  // The condition is true if the sensor has been updated since the last time we asked for it

  if (nordend.data.update()) {
    // static unsigned counter = 0;
    // static unsigned lastCounter = 0;
    // if (counter-lastCounter>100) {
    //   lastCounter = counter;
    // Serial.println("After Update" + String(counter++));
    // }
    // else {
    //   counter ++;
    // }
    // .compute() will updated every single "action" variable in the uav object
    // Serial.println("Before Compute");
    nordend.compute();
    // Serial.println("After Compute");
    // Serial.println("Before Ouput");
    // .output() will output the updated action dataset on the servos, led, buzzer, etc. 
    nordend.output();
    // Serial.println("After Output");
  }
  nordend.update();
  // Serial.println("After updated");
  delayMicroseconds(500);
}

