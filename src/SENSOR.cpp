#include "SENSOR.h"

static senStatus sen;


bool senClass::update() {

  if (sen.updated) {
    sen.updated = false;
    position = sen.position;
    speed = sen.speed;
    attitude = sen.attitude;
    temperature = sen.temperature;
    pressure = sen.pressure;
    gps = sen.gps;
    time = sen.time;
    course = sen.course;
    valid = sen.valid; 
    return true;
  }
  return false;
}


senClass::senClass() 
{
    time.isValid = false;
    gps.isValid = false;
}

void senClass::begin(senSettings settings) {
    Serial.begin(115200);

    config(settings);
}


void senClass::config(senSettings settings) {

}

bool senClass::isValid() {
    if (sen.time.year !=1970 && sen.position.lat !=0 && sen.position.lng !=0) {
        return (gps.isValid && time.isValid);
    }
    else {
        return false;
    }
}

senStatus senClass::get() {
    senStatus senOut;
    senOut.position = position;
    senOut.attitude = attitude;
    senOut.speed = speed;
    senOut.temperature = temperature;
    senOut.pressure = pressure;
    senOut.time = time;
    senOut.valid = isValid();
    senOut.course = int(((((PI/2.0)-atan2(speed.y, speed.x))*180/PI)+360)*100.0)%36000;
    senOut.course = senOut.course/100.0;
    senOut.twoDSpeed = sqrt(pow(speed.x,2)+pow(speed.y,2));
    senOut.threeDSpeed = sqrt(pow(speed.x,2)+pow(speed.y,2)+pow(speed.z,2));
    senOut.gps = gps;
    return senOut;
}


double senClass::timeDiff(timeCode time1, timeCode time2) {
    double timeDiff = (time1.second - time2.second) + ((time1.nanosecond - time2.nanosecond)/1000000000.0);
    return timeDiff;
}

double timeDiff(timeCode time1, timeCode time2) {
    double timeDiff = (time1.second - time2.second) + ((time1.nanosecond - time2.nanosecond)/1000000000.0);
    return timeDiff;
}
