#include "SENSOR.h"

bool senClass::update() {
// Collect data from all sensors and return true if some data has been updated
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
    if (time.year !=1970 && position.lat !=0 && position.lng !=0) {
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
    senOut.time = time;
    senOut.valid = isValid();
    senOut.course = int(((((PI/2.0)-atan2(speed.y, speed.x))*180/PI)+360)*100.0)%36000;
    senOut.course = senOut.course/100.0;
    senOut.twoDSpeed = sqrt(pow(speed.x,2)+pow(speed.y,2));
    senOut.threeDSpeed = sqrt(pow(speed.x,2)+pow(speed.y,2)+pow(speed.z,2));

    // senOut.gpsMainStatus.fixType = gpsMain.fixType;
    senOut.gpsMainStatus.satNumber = gpsMain.satellites.value();
    senOut.gpsMainStatus.hdop = gpsMain.hdop.value();
    senOut.gpsMainStatus.isValid = gpsMain.location.isValid();

    // senOut.gpsAuxStatus.fixType = gpsAux.fixType;
    senOut.gpsAuxStatus.satNumber = gpsAux.satellites.value();
    senOut.gpsAuxStatus.hdop = gpsAux.hdop.value();
    senOut.gpsAuxStatus.isValid = gpsAux.location.isValid();

    senOut.baro = baro.get();
    senOut.prop = prop.get();

    senOut.msSinceMidnight = msSinceMidnight;

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
