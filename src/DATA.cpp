#include "DATA.h"

static int year, month, day, hour, minute, second;

waypointData readWaypoint() {

  File missionFile = SD.open("mission.txt", FILE_READ);
  bool endOfFile = false;
  waypointData failReading;
  failReading.length = 0;

  if (!missionFile) {
    return failReading;
  }

  double dataArray[51][4]; // Added one to prevent potential data flooding

  for (int i = 0; i < 50; i++) {
    for (int j = 0; j < 3; j++) {
      dataArray[i][j] = 0;
    }
  }

  while (missionFile.available() && !endOfFile) {

    char dataChar;
    String recupData;
    int i(0);
    int j(0);

    do {
      dataChar = missionFile.read();
      if (dataChar != 10 and dataChar != 13 and dataChar != '*') {
        if (dataChar != ',') {
          recupData += dataChar;
        }
        else {
          dataArray[i][j] = recupData.toFloat();
          recupData = "";
          j++;
        }
      }
      else {
        dataArray[i][j] = recupData.toFloat();
        recupData = "";
        j = 0;
        i++;
      }
    } while (dataChar != '*');

    endOfFile = true;
  }

  int nbOfWaypoint;

  for (int i = 0; i < 50; i++) {
    if (dataArray[i][0] == 0) {
      nbOfWaypoint = i;
      break;
    }
    else if (dataArray[i][0] != 0 && i == 49) {
      nbOfWaypoint = 49;
    }
  }

  gpsCoord waypointMemory[nbOfWaypoint+1];

  for (int i(0); i < nbOfWaypoint; ++i) {
    waypointMemory[i].lat = dataArray[i][0];
    waypointMemory[i].lng = dataArray[i][1];
    waypointMemory[i].alt = dataArray[i][2];
    if (DEBUG) {
      Serial.println("List of waypoints as read..");
      Serial.print(waypointMemory[i].lat,12); Serial.print(" "); 
      Serial.print(waypointMemory[i].lng,12); Serial.print(" "); 
      Serial.println(waypointMemory[i].alt,12);
      Serial.println("Size of waypoint list as calculated..");
      Serial.println(nbOfWaypoint);
    }
  }

  waypointData waypointMemoryData;
  waypointMemoryData.length = nbOfWaypoint;

  for (int i = 0; i < nbOfWaypoint; i++) {
    waypointMemoryData.gpsData[i] = waypointMemory[i];
  }

  return waypointMemoryData;
}


dataClass::dataClass(dataListString listIn[30], int len)
: firstTimeSave(true)
{
  //if (DEBUG) { Serial.println("DataClass Constructor In Second");}
  for(int i(0); i < len; ++i) {
    list[i] = listIn[i];
  }
  for (int i(len); i < 25; i++) {
    list[i] = NULL_DATA;
  }
  //if (DEBUG) { Serial.println("DataClass Constructor Out Second");}
  //RADIOSONDE_PORT.begin(RADIOSONDE_BAUD);
}

void dataClass::begin() {
  RADIOSONDE_PORT.begin(RADIOSONDE_BAUD);
  SdFile::dateTimeCallback(dateTime);
}

dataStruct dataClass::get() {
  dataStruct dataOut;
  dataOut.cmd = cmd.get();
  dataOut.sen = sen.get();
  dataOut.bat = bat.get();
  return dataOut;
}

bool dataClass::update() {
  cmd.update();
  bat.update();
  if (sen.update()) {
    updated = true;
    return true;
  }
  updated = false;
  return false;
}

void dataClass::send(sysStatus sysIn) {
  TelemetryPacket packet;

  senStatus senData = sen.get();

  gpsCoord driftPoint;
  driftPoint.lat = sysIn.nav.sim.lat;
  driftPoint.lng = sysIn.nav.sim.lng;
  double distTraj = distanceTo(driftPoint,sysIn.nav.waypoint);
         
  gpsCoord currentPoint;
  currentPoint.lat = sen.get().position.lat;
  currentPoint.lng = sen.get().position.lng;
  double distPos = distanceTo(currentPoint,sysIn.nav.waypoint);

  double realSetpoint = 0;
  if(sysIn.nav.navMode == COG) {
    realSetpoint = -sysIn.nav.con.input+senData.course;
    realSetpoint = (int((realSetpoint+360)*100.0)%36000)/100.0;
  }
  if (sysIn.nav.navMode == HEADING_ABSOLUTE) {
    realSetpoint = -sysIn.nav.con.input+senData.attitude.yaw;
    realSetpoint = (int((realSetpoint+360)*100.0)%36000)/100.0;
  }
  if (sysIn.nav.navMode == HEADING_RELATIVE or sysIn.nav.navMode == HEADING_RELATIVE_FINAL or sysIn.nav.navMode == MANUAL) {
    realSetpoint = sysIn.nav.headingRelativeSetpoint;
  }

  packet.timeSecond = senData.time.code.second;
  packet.flightMode = sysIn.flightMode; 

  // packet.status is 1 byte where each bit represents a status. 
  // Bit 0 = initialised
  // Bit 1 = separated
  // Bit 2 = deployed 
  // Bit 3 = wing opened 
  // Bit 4 = sensor valid 
  // Bit 5 = gps valid
  // Bit 6 = time valid

  packet.status = 0;
  if (sysIn.initialised) {
    packet.status |= 1;
  }
  if (sysIn.separated) {
    packet.status |= 2;
  }
  if (sysIn.deployed) {
    packet.status |= 4;
  }
  if (sysIn.wingOpened) {
    packet.status |= 8;
  }
  if (senData.valid) {
    packet.status |= 16;
  }
  if (senData.gps.isValid) {
    packet.status |= 32;
  }
  if (senData.time.isValid) {
    packet.status |= 64;
  }

  senData.gps.fixType = senData.gps.fixType;

  packet.latitude = senData.position.lat;
  packet.longitude = senData.position.lng;
  packet.altitude = senData.position.alt;

  packet.yaw = senData.attitude.yaw;
  packet.rotationSpeed = senData.rotationSpeedAvgData;

  packet.zSpeed = senData.speed.z;
  packet.twoDSpeed = senData.twoDSpeed;

  packet.temperature = senData.temperature;
  packet.voltage = bat.get().voltage;

  packet.waypointLatitude = sysIn.nav.waypoint.lat;
  packet.waypointLongitude = sysIn.nav.waypoint.lng;
  packet.waypointAltitude = sysIn.nav.waypoint.alt;

  packet.trajectoryLatitude = driftPoint.lat;
  packet.trajectoryLongitude = driftPoint.lng;

  // packet.distanceToTrajectory = distTraj;
  // packet.distanceToPosition = distPos;

  packet.relativeHeadingSetpoint = realSetpoint;
  packet.projectedGroundSpeed = sysIn.nav.projectedGroundSpeed;

  size_t size = sizeof(packet); // Get the size of the struct in bytes
  byte* buffer = new byte[size]; // Allocate memory for the byte array
  memcpy(buffer, &packet, size); // Copy the struct to the byte array

  size_t codedSize = cmd.telemetryRadio.getCodedLen(size);
  byte packetId = 0x00;
  byte* codedBuffer = cmd.telemetryRadio.encode(packetId, buffer, size);
  //codedBuffer = cmd.telemetryRadio.encode(packetId, buffer, size);

  TLM_PORT.write(codedBuffer, codedSize);

  //Serial.println("Telemetry sent");
  delete[] buffer; // Don't forget to deallocate the memory when you're done
  delete[] codedBuffer;
}

void dataClass::save(sysStatus sysIn) {
  if (sysIn.initialised) {
    if(firstTimeSave) {
      int sdNameFile;

      year = sen.get().time.year;
      month = sen.get().time.month;
      day = sen.get().time.day;
      hour = sen.get().time.hour;
      minute = sen.get().time.minute;
      second = sen.get().time.second;

      do {
        static unsigned trialNumber = 0;
        second = (second+trialNumber)%60;
        minute = (minute+trialNumber/60);
        sdNameFile = day*1000000 + hour*10000 + minute*100 + second;
        sprintf(namebuff, "%d.txt", sdNameFile);
        sprintf(namebuffLowRate, "%dLR.txt", sdNameFile);
        sprintf(namebuffWaypointLog, "%dWPLog.txt", sdNameFile);
        trialNumber++;
      } while (SD.exists(namebuff));

      SdFile::dateTimeCallback(dateTime);
      dataFile = SD.open(namebuff, FILE_WRITE);
      if (dataFile) { 
        dataFile.println(printStarterString());
      }
      dataFile.close();

      SdFile::dateTimeCallback(dateTime);
      dataFileLowRate = SD.open(namebuffLowRate, FILE_WRITE);
      if (dataFileLowRate) {
        dataFileLowRate.println(printStarterString());
      }
      dataFileLowRate.close();

      SdFile::dateTimeCallback(dateTime);
      waypointLogFile = SD.open(namebuffWaypointLog, FILE_WRITE);
      if (waypointLogFile) {
         waypointLogFile.println(printStarterStringWaypoint(sysIn));
      }
      waypointLogFile.close();
      firstTimeSave = false;
    }

    SdFile::dateTimeCallback(dateTime);
    dataFile = SD.open(namebuff, FILE_WRITE);
    if (dataFile) { 
      dataFile.println(print(sysIn)); 
    }
    dataFile.close(); 

    SdFile::dateTimeCallback(dateTime);
    waypointLogFile = SD.open(namebuffWaypointLog, FILE_WRITE);
    if (waypointLogFile) {
      waypointLogFile.println(printWaypoint(sysIn));
    }
    waypointLogFile.close();

    if (LOW_RATE) {
      static timeCode lastTime;
      if (sen.timeDiff(sen.get().time.code,lastTime)>=(1.0/LOW_RATE_RATE)) {
        lastTime = sen.get().time.code;
        SdFile::dateTimeCallback(dateTime);
        dataFileLowRate = SD.open(namebuffLowRate, FILE_WRITE);
        if (dataFileLowRate) { 
          dataFileLowRate.println(print(sysIn)); 
        }
        dataFileLowRate.close(); 
      }
      /* if (millis()-lastTime>=(1000.0/LOW_RATE_RATE)) {
        lastTime = millis()-((millis()-lastTime)-(1000.0/LOW_RATE_RATE));
        dataFileLowRate = SD.open(namebuffLowRate, FILE_WRITE);
        if (dataFileLowRate) { 
          dataFileLowRate.println(print(sysIn)); 
        }
        dataFileLowRate.close(); 
      } */
    }
  }

  if(TLM_MONITOR) { 
    String tlmString;
    tlmString = print(sysIn);
    Serial.println(tlmString); 
  }

  if (TLM_MAIN) {
    static timeCode lastTime;
    //Serial.print(sen.get().time.code.second); Serial.print(" "); Serial.println(sen.get().time.code.nanosecond);
    //Serial.println(sen.timeDiff(sen.get().time.code,lastTime));
    if (sen.timeDiff(sen.get().time.code,lastTime)>=(1.0/(TLM_MAIN_RATE))) {
      lastTime = sen.get().time.code;
      send(sysIn);
    }
    /* if (millis()-lastTime>=(1000.0/(TLM_MAIN_RATE)))  {
      lastTime = millis()-((millis()-lastTime)-(1000.0/(TLM_MAIN_RATE)));
      send(sysIn);
    } */
  }

  if(TLM_RADIOSONDE) {
    static timeCode lastTime;
    if (sen.timeDiff(sen.get().time.code,lastTime)>=(1.0/TLM_RADIOSONDE_RATE)) {
      lastTime = sen.get().time.code;
      String shortTLM = "";
      shortTLM = (String)TLM_RADIOSONDE_START + (String)TLM_RADIOSONDE_PREFIX +
                  String(sysIn.flightMode)+"C"+
                  String(int(bat.get().voltage*10.0))+"C"+
                  String(int(sen.get().temperature))+"C"+
                  String(int(sen.get().attitude.yaw))+"C"+
                  String(int(sen.get().speed.z*10.0));
      RADIOSONDE_PORT.println(shortTLM);
      //Serial.println(shortTLM);
    }
    /* if (millis()-lastTime>=(1000.0/TLM_RADIOSONDE_RATE)) {
      lastTime = millis()-((millis()-lastTime)-(1000.0/TLM_RADIOSONDE_RATE));
      String shortTLM = "";
      shortTLM = (String)TLM_RADIOSONDE_START + (String)TLM_RADIOSONDE_PREFIX +
                  String(sysIn.flightMode)+"C"+
                  String(int(bat.get().voltage*10.0))+"C"+
                  String(int(sen.get().temperature))+"C"+
                  String(int(sen.get().attitude.yaw))+"C"+
                  String(int(sen.get().speed.z*10.0));
      RADIOSONDE_PORT.println(shortTLM);
      //Serial.println(shortTLM);
    } */
  }
}

void dataClass::dateTime(uint16_t* date, uint16_t* time) {
  *date = FAT_DATE(year, month, day);
  *time = FAT_TIME(hour, minute, second);
}

bool dataClass::isUpdated() {
  if (updated) {
    updated = false;
    return true;
  }
  else {
    return false;
  }
}

String dataClass::printStarterString() {
  String output = "";
  int len = sizeof(list)/sizeof(list[0]);
  for(int i(0); i < len; ++i) {
    if (list[i] == NULL_DATA) {
      break;
    }
    switch(list[i]) {
      case NULL_DATA:
      break;
      case DATE_TIME:
        output += "YEAR, MONTH, DAY, HOUR, MIN, SECOND, NANOSECOND";
      break;
      case TIME_CODE:
        output += "TIME_CODE";
      break;
      case FLIGHT_MODE:
        output += "FLIGHT_MODE";
      break;
      case TIME_TRANSITION:
        output += "TIME_TRANSITION";
      break;
      case INITIALIZED:
        output += "INITIALIZED";
      break;
      case SEPARATED:
        output += "SEPARATED";
      break;
      case DEPLOYED:
        output += "DEPLOYED";
      break;
      case WING_OPEN:
        output += "WING_OPEN";
      break;
      case SEN_HEALTH:
        output += "SENSOR_VALID, GPS_VALID, SAT_NUMBER, VERTICAL_ACCURACY, TIME_VALID";
      break;
      case POSITION_VALUES:
        output += "LATITUDE, LONGITUDE, ALTITUDE";
      break;
      case INCLINATION_VALUES:
        output += "ROLL, PITCH, YAW";
      break;
      case SPEED_VALUES:
        output += "X, Y, Z, TWOD, THREED";
      break;
      case COURSE:
        output += "COURSE";
      break;
      case TEMPERATURE:
        output += "TEMPERATURE";
      break;
      case PRESSURE:
        output += "PRESSURE";
      break;
      case VDOWN_VALUE:
        output += "VDOWN";
      break;
      case WAYPOINT_POSITION_VALUES:
        output += "WAYPOINT_LAT, WAYPOINT_LON, WAYPOINT_ALT";
      break;
      case WAYPOINT_SCORE:
        output += "WAYPOINT_SCORE";
      break;
      case TRAJ_VALUES:
        output += "TRAJ_LAT, TRAJ_LON";
      break;
      case DIST_TRAJ:
        output += "DIST_TRAJ";
      break;
      case DIST_POS:
        output += "DIST_POS";
      break;
      case NAV_VALUES:
        output += "INPUT, SETPOINT, SETPOINT_REAL, OUTPUT, NAVMODE, PROJECTED_SPEED";
      break;
      case MIX_VALUES:
        output += "DIR, BRK, ACC, DEP";
      break;
      case SERVO_VALUES:
        output += "L, R, X1, X2";
      break;
      case WIND_VALUES:
        output += "WIND_SPE_GFS, WIND_DIR_GFS, WIND_SPE_MEASURED, WIND_DIR_MEASURED";
      break;
      case FACTOR_VALUE:
        output += "FACTOR";
      break;
      case BAT_VALUES:
        output += "BAT_VOLT, BAT_CURR";
      break;
    }
    if(i<(len-1) && list[i+1] != NULL_DATA) { output += ", "; }
  }
  output += "";
  return output;
}

String dataClass::printStarterStringWaypoint(sysStatus sysIn) {
  String output = "";
  int len = sysIn.nav.waypointChoice.length;
  output += "TIME_CODE, ";
  for(int i(0); i < len; ++i) {
    output += "WAYPOINT_"+String(i)+"_LAT, "+ 
              "WAYPOINT_"+String(i)+"_LON, "+
              "WAYPOINT_"+String(i)+"_ALT, "+
              "WAYPOINT_"+String(i)+"_SCORE";
    if(i!=len-1) { output += ", "; }
  }
  return output;
}

String dataClass::printWaypoint(sysStatus sysIn) {
  String output = "";
  int len = sysIn.nav.waypointChoice.length;
  output += String((sysIn.time.second+sysIn.time.nanosecond/1000000000.0),4) + ", ";
  for(int i(0); i < len; ++i) {
    output += String(sysIn.nav.waypointChoice.gpsData[i].lat,5)+", "+
              String(sysIn.nav.waypointChoice.gpsData[i].lng,5)+", "+
              String(sysIn.nav.waypointChoice.gpsData[i].alt,0)+", "+
              String(sysIn.nav.waypointChoice.score[i]);
    if(i!=len-1) { output += ", "; }
  }
  return output;
}

// Return a string that contains all the informations of the flight 
String dataClass::print(sysStatus sysIn) {
  String output = "";
  String nanosecondData = "";
  int len = sizeof(list)/sizeof(list[0]);
  for(int i(0); i < len; ++i) {
    if (list[i] == NULL_DATA) {
      break;
    }
    switch(list[i]) {
      case NULL_DATA:
      break;
      case DATE_TIME:
        output += String(sen.get().time.year) + ",";
        output += String(sen.get().time.month) + ",";
        output += String(sen.get().time.day) + ",";
        output += String(sen.get().time.hour) + ",";
        output += String(sen.get().time.minute) + ",";
        output += String(sen.get().time.second) + ",";
        output += String(sen.get().time.nanosecond);
      break;

      case TIME_CODE:
        nanosecondData = String(sysIn.time.nanosecond/1000000000.0,6);
        nanosecondData.remove(0,1);
        output+= String(sysIn.time.second)+nanosecondData;
      break;

      case FLIGHT_MODE:
        output += String(sysIn.flightMode);
      break;
      case TIME_TRANSITION:
        output += String(sysIn.timeTransition.second)+String(sysIn.timeTransition.nanosecond/1000000000.0,6);
      break;
      case INITIALIZED:
        output += String(sysIn.initialised);
      break;
      case SEPARATED:
        output += String(sysIn.separated);
      break;
      case DEPLOYED:
        output += String(sysIn.deployed);
      break;
      case WING_OPEN:
        output += String(sysIn.wingOpened);
      break;
      case SEN_HEALTH:
        output += String(sen.isValid()) + ",";
        output += String(sen.get().gps.isValid) + ",";
        output += String(sen.get().gps.satNumber) + ",";
        output += String(sen.get().gps.vAcc) + ","; 
        output += String(sen.get().time.isValid);
      break;
      case POSITION_VALUES:
        output += String(sen.get().position.lat,5) + ",";
        output += String(sen.get().position.lng,5) + ",";
        output += String(sen.get().position.alt);
      break;
      case INCLINATION_VALUES:
        output += String(sen.get().attitude.roll) + ",";
        output += String(sen.get().attitude.pitch) + ",";
        output += String(sen.get().attitude.yaw);
      break;
      case SPEED_VALUES:
        output += String(sen.get().speed.x) + ",";
        output += String(sen.get().speed.y) + ",";
        output += String(sen.get().speed.z) + ",";
        output += String(sen.get().twoDSpeed) + ",";
        output += String(sen.get().threeDSpeed);
      break;
      case COURSE:
        output += String(sen.get().course);
      break;
      case TEMPERATURE:
        output += String(sen.get().temperature);
      break;
      case PRESSURE:
        if (sen.get().pressure != 0) {
          output += String(sen.get().pressure);
        }
        else {
          output += String(pressureSim(sen.get().position.alt));
        }
      break;
      case VDOWN_VALUE:
        output += String(sysIn.realVDOWN);
      break;
      case WAYPOINT_POSITION_VALUES:
        output += String(sysIn.nav.waypoint.lat,5) + ",";
        output += String(sysIn.nav.waypoint.lng,5) + ",";
        output += String(sysIn.nav.waypoint.alt);
      break;
      case WAYPOINT_SCORE:
        if (1) {
          navStatus navCopy = sysIn.nav;
          double max = -1000;
          unsigned index = 0;
          for (int i(0); i < navCopy.waypointChoice.length; ++i) {
            if (navCopy.waypointChoice.score[i] > max) {
              max = navCopy.waypointChoice.score[i];
              index = i;
            }
          }
          output += String(navCopy.waypointChoice.score[index]);
        }
      break;
      case TRAJ_VALUES:
        output += String(sysIn.nav.sim.lat,5) + ",";
        output += String(sysIn.nav.sim.lng,5);
      break;
      case DIST_TRAJ:
        if (1) {
          gpsCoord driftPoint;
          driftPoint.lat = sysIn.nav.sim.lat;
          driftPoint.lng = sysIn.nav.sim.lng;
          double distTraj = distanceTo(driftPoint,sysIn.nav.waypoint);
          output += String(distTraj);
        }
      break;
      case DIST_POS:
        {
          gpsCoord driftPoint;
          driftPoint.lat = sen.get().position.lat;
          driftPoint.lng = sen.get().position.lng;
          double distPos = distanceTo(driftPoint,sysIn.nav.waypoint);
          output += String(distPos);
        }
      break;
      case NAV_VALUES :
        output += String(sysIn.nav.con.input) + ",";
        output += String(sysIn.nav.con.setpoint) + ",";
        if (1) {
          double realSetpoint = 0;
          if(sysIn.nav.navMode == COG) {
            realSetpoint = -sysIn.nav.con.input+sen.get().course;
            realSetpoint = (int((realSetpoint+360)*100.0)%36000)/100.0;
          }
          if (sysIn.nav.navMode == HEADING_ABSOLUTE)  {
            realSetpoint = -sysIn.nav.con.input+sen.get().attitude.yaw;
            realSetpoint = (int((realSetpoint+360)*100.0)%36000)/100.0;
          }
          if (sysIn.nav.navMode == HEADING_RELATIVE or sysIn.nav.navMode == HEADING_RELATIVE_FINAL) {
            realSetpoint = sysIn.nav.headingRelativeSetpoint;
          }
          output += String(realSetpoint) + ",";
        }
        output += String(sysIn.nav.con.output) + ",";
        output += String(sysIn.nav.navMode) + ",";
        output += String(sysIn.nav.projectedGroundSpeed);
      break;
      case MIX_VALUES:
        output += String(sysIn.mix.dir) + ",";
        output += String(sysIn.mix.brk) + ",";
        output += String(sysIn.mix.acc) + ",";
        output += String(sysIn.mix.dep);
      break;
      case SERVO_VALUES:
        output += String(int(sysIn.ser.l)) +  ",";
        output += String(int(sysIn.ser.r)) + ",";
        output += String(int(sysIn.ser.x1)) + ",";
        output += String(int(sysIn.ser.x2));
      break;
      case WIND_VALUES:
        output += String(sysIn.nav.windSpeedGFS) + ",";
        output += String(sysIn.nav.windDirGFS) + ",";
        output += String(sysIn.nav.windSpeedMeasured) + ",";
        output += String(sysIn.nav.windDirMeasured);
      break; 
      case FACTOR_VALUE:
        output += String(sysIn.nav.spiralFactor);
      break;
      case BAT_VALUES:
        output += String(bat.get().voltage) + ",";
        output += String(bat.get().current);
      break;
    }
    if(i!=len-1 && list[i+1] != NULL_DATA) { output += ","; }
  }
  output += "";
  return output;
}

batClass::batClass() 
  :voltage(16.4), current(0), pinVoltage(BAT_PIN)
{
    pinMode(pinVoltage, INPUT);
    analogReadResolution(12); 
}

batStatus batClass::update() {
    batStatus batOut;
    //voltage = analogRead(pinVoltage)/222.4539;
    voltage = analogRead(pinVoltage)/112.1124;
    current = 0;
    batOut.voltage = voltage;
    batOut.current = current;
    return batOut;
}

batStatus batClass::get() {  
    batStatus batOut;
    batOut.voltage = voltage;
    batOut.current = current;
    return batOut;
}

void dataClass::floatToByte(float fIn, byte *b) {
  union {
      float f;
      uint8_t i[4];
  } u;

  u.f = fIn;

  b[0] = u.i[3];
  b[1] = u.i[2];
  b[2] = u.i[1];
  b[3] = u.i[0];
}

void dataClass::int32ToByte(int32_t intIn, byte *b) {
  union {
      int32_t intData;
      uint8_t i[4];
  } u;

  u.intData = intIn;

  b[0] = u.i[3];
  b[1] = u.i[2];
  b[2] = u.i[1];
  b[3] = u.i[0];
}

void dataClass::int16ToByte(int16_t intIn, byte *b) {
  union {
      int16_t intData;
      uint8_t i[2];
  } u;

  u.intData = intIn;

  b[0] = u.i[1];
  b[1] = u.i[0];
}

void dataClass::int8ToByte(int8_t intIn, byte *b) {
  union {
      int8_t intData;
      uint8_t i[1];
  } u;

  u.intData = intIn;

  b[0] = u.i[0];
}
