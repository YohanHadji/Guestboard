#include "SENSOR.h"

bool senClass::update() {
    // Collect data from all sensors and return true if some data has been updated

    bool positionUpdated = false;
    if (gpsMain.getPVT() == true) {
        unsigned long startOfFunc = micros();
        positionUpdated = true;
        if (gpsMain.getGnssFixOk() == true) {
            static float lastAltitude = position.alt;
            static unsigned long lastTime = millis()-1;

            speed.z = (position.alt-lastAltitude)/(millis()-lastTime);

            position.lat = gpsMain.getLatitude()/10000000.0;
            position.lng = gpsMain.getLongitude()/10000000.0;
            position.alt = gpsMain.getAltitudeMSL()/1000.0;
            // TODO: Verify that this way to note the time is correct
            // It might be jumping a bit due to the delay to transmit 
            // one full NMEA message. In that case we would only note time once
            // at initialisation for example and then use elapsed ms to know the time
            // msSinceMidnight; // gpsMain.time.value()*10.0+gpsMain.time.age();
            time.hour = gpsMain.getHour();
            time.minute = gpsMain.getMinute();
            time.second = gpsMain.getSecond();
            time.nanosecond = gpsMain.getNanosecond();

            static bool timeInitialised = false;

            if (!timeInitialised) {
                msSinceMidnight = time.hour*3600000+time.minute*60000+time.second*1000+time.nanosecond/1000000;
            }

            time.isValid = gpsMain.getTimeValid();
            gps.fixType = gpsMain.getFixType();
            //gps.hdop = gpsMain.getHorizontalDOP();
            gps.isValid = gpsMain.getGnssFixOk();
        }
        else if (gpsAux.getGnssFixOk() == true) {
            // if (DEBUG) {
            //     Serial.println("Only GPS Aux is valid");
            // }
            // position.lat = gpsAux.location.lat();
            // position.lng = gpsAux.location.lng();
            // position.alt = gpsAux.altitude.meters();
            // // TODO: Verify that this way to note the time is correct
            // // It might be jumping a bit due to the delay to transmit 
            // // one full NMEA message. In that case we would only note time once
            // // at initialisation for example and then use elapsed ms to know the time
            // msSinceMidnight = gpsAux.time.value()*10.0+gpsAux.time.age();
            // time.isValid = gpsAux.time.isValid();
            // // gps.fixType = gpsAux.fixType();
            // gps.hdop = gpsAux.hdop.value();
            // gps.satNumber= gpsAux.satellites.value();
            // gps.isValid = gpsAux.location.isValid();
        }
    }

    static unsigned long lastUpdate = 0;
    if (((millis() - lastUpdate) >= (1000.0/SENSOR_UPDATE_RATE))) {   // or positionUpdated) {
        
        if (DEBUG) {
            // Serial.print("Looptime: ");
            // Serial.println(millis() - lastUpdate);
        }

        lastUpdate = millis();
        // positionUpdated = false;

        time.hour = msSinceMidnight/3600000;
        time.minute = (msSinceMidnight%3600000)/60000;
        time.second = ((msSinceMidnight%3600000)%60000)/1000;
        time.nanosecond = (((msSinceMidnight%3600000)%60000)%1000)*1000000;
        time.code.second = msSinceMidnight/1000;
        time.code.nanosecond = time.nanosecond;

        baro.read();

        static unsigned long lastPropSample = 0;
        if ((millis() - lastPropSample) >= (1000.0/(PROP_SENSOR_SAMPLE_RATE*4.0))) {
            if (DEBUG) {
                // Serial.println("Reading Prop");
            }
            lastPropSample = millis();
            static int channelSelect = 0;
            prop.readChannel((channelSelect++)%4);
        }
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

    GPS_MAIN_PORT.begin(GPS_MAIN_BAUD);
    while (gpsMain.begin(GPS_MAIN_PORT) == false) {
        if (DEBUG) {
            Serial.println(F("u-blox Main GNSS not detected. Retrying..."));
        }
        delay(100);
    }
    gpsMain.setUART1Output(COM_TYPE_UBX);
    gpsMain.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    gpsMain.setMeasurementRate(500);
    gpsMain.setNavigationRate(1); 
    gpsMain.setAutoPVT(true);
    gpsMain.setSerialRate(115200);

    GPS_MAIN_PORT.end();
    GPS_MAIN_PORT.begin(115200);

    GPS_AUX_PORT.begin(GPS_AUX_BAUD);
    while (gpsAux.begin(GPS_AUX_PORT) == false) {
        if (DEBUG) {
            Serial.println(F("u-blox Aux GNSS not detected. Retrying..."));
        }
        delay(100);
    }
    gpsAux.setUART1Output(COM_TYPE_UBX);
    gpsAux.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    gpsAux.setMeasurementRate(500);
    gpsAux.setNavigationRate(1);
    gpsAux.setAutoPVT(true);
    gpsAux.setSerialRate(115200);

    GPS_AUX_PORT.end();
    GPS_AUX_PORT.begin(115200);

    baro.begin();
    prop.begin();
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
    senOut.position = positionAux;
    senOut.attitude = attitude;
    senOut.speed = speed;
    senOut.time = time;
    senOut.valid = isValid();
    senOut.course = int(((((PI/2.0)-atan2(speed.y, speed.x))*180/PI)+360)*100.0)%36000;
    senOut.course = senOut.course/100.0;
    senOut.twoDSpeed = sqrt(pow(speed.x,2)+pow(speed.y,2));
    senOut.threeDSpeed = sqrt(pow(speed.x,2)+pow(speed.y,2)+pow(speed.z,2));

    // senOut.gpsMainStatus.fixType = gpsMain.fixType;
    // senOut.gpsMainStatus.satNumber = gpsMain.satellites.value();
    // senOut.gpsMainStatus.hdop = gpsMain.hdop.value();
    // senOut.gpsMainStatus.isValid = gpsMain.location.isValid();

    // // senOut.gpsAuxStatus.fixType = gpsAux.fixType;
    // senOut.gpsAuxStatus.satNumber = gpsAux.satellites.value();
    // senOut.gpsAuxStatus.hdop = gpsAux.hdop.value();
    // senOut.gpsAuxStatus.isValid = gpsAux.location.isValid();

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
