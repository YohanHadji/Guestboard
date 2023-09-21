#include "DATA.h"

static int year, month, day, hour, minute, second;

dataClass::dataClass(dataListString listIn[30], int len)
: firstTimeSave(true)
{
  for(int i(0); i < len; ++i) {
    list[i] = listIn[i];
  }
  for (int i(len); i < 25; i++) {
    list[i] = NULL_DATA;
  }
}

void dataClass::begin() {
  SdFile::dateTimeCallback(dateTime);
}

dataStruct dataClass::get() {
  dataStruct dataOut;
  dataOut.com = com.get();
  dataOut.sen = sen.get();
  dataOut.bat = bat.get();
  return dataOut;
}

bool dataClass::update() {
  com.update();
  bat.update();
  if (sen.update()) {
    updated = true;
    return true;
  }
  updated = false;
  return false;
}

void dataClass::send(sysStatus sysIn) {

  senStatus senIn;
  senIn = sen.get();

  PacketAVDownlink packetToSend;

  packetToSend.timestamp = senIn.msSinceMidnight;
  packetToSend.flightMode = sysIn.flightMode;

  packetToSend.lat = senIn.position.lat;
  packetToSend.lng = senIn.position.lng;
  packetToSend.alt = senIn.position.alt;

  packetToSend.positionAge = senIn.age;

  packetToSend.engineStatus.pressureN2O = senIn.prop.pressureN2O;
  packetToSend.engineStatus.pressureFuel = senIn.prop.pressureFuel;
  packetToSend.engineStatus.pressureChamber = senIn.prop.pressureChamber;
  packetToSend.engineStatus.temperatureN2O = senIn.prop.temperatureN2O;

  if (sysIn.out.ventN2O == VENT_N2O_OPEN) {
    packetToSend.engineStatus.solenoidVentN2O = OPEN;
  }
  else {
    packetToSend.engineStatus.solenoidVentN2O = CLOSED;
  }

  if (sysIn.out.ventFuel == VENT_FUEL_OPEN) {
    packetToSend.engineStatus.solenoidVentFuel = OPEN;
  }
  else {
    packetToSend.engineStatus.solenoidVentFuel = CLOSED;
  }

  if (sysIn.out.pressurizer == PRESSURIZER_OPEN) {
    packetToSend.engineStatus.solenoidPressure = OPEN;
  }
  else {
    packetToSend.engineStatus.solenoidPressure = CLOSED;
  }

  if (sysIn.out.servoN2O == SERVO_N2O_OPEN) {
    packetToSend.engineStatus.servoN2O = OPEN;
  }
  else {
    packetToSend.engineStatus.servoN2O = CLOSED;
  }

  if (sysIn.out.servoFuel == SERVO_FUEL_OPEN) {
    packetToSend.engineStatus.servoFuel = OPEN;
  }
  else {
    packetToSend.engineStatus.servoFuel = CLOSED;
  }

  com.sendTelemetry(CAPSULE_ID::AV_TELEMETRY, (uint8_t*)&packetToSend, sizeof(packetToSend));
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
      firstTimeSave = false;
    }

    SdFile::dateTimeCallback(dateTime);
    dataFile = SD.open(namebuff, FILE_WRITE);
    if (dataFile) { 
      dataFile.println(print(sysIn)); 
    }
    dataFile.close(); 

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
    }
  }

  if(TLM_MONITOR) { 
    String tlmString;
    tlmString = print(sysIn);
    Serial.println(tlmString); 
  }

  if (TLM_MAIN) {
    static timeCode lastTime;
    if (sen.timeDiff(sen.get().time.code,lastTime)>=(1.0/(TLM_MAIN_RATE))) {
      lastTime = sen.get().time.code;
      send(sysIn);
    }
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
      case CHUTE_OPEN:
        output += "WING_OPEN";
      break;
      case SEN_HEALTH:
        output += "SENSOR_VALID, GPS_MAIN_VALID, SAT_MAIN_NUMBER, GPS_AUX_VALID, SAT_AUX_NUMBER, TIME_VALID";
      break;
      case POSITION_VALUES:
        output += "LATITUDE, LONGITUDE, ALTITUDE";
      break;
      case ATTITUDE_VALUES:
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
      case MIX_VALUES:
        output += "SOL1_MIX, SOL2_MIX, SOL3_MIX, SOL4_MIX, SER1_MIX, SER2_MIX, IGN_MIX, BUZ_MIX";
      break;
      case OUTPUT_VALUES:
        output += "SOL1_OUT, SOL2_OUT, SOL3_OUT, SOL4_OUT, SER1_OUT, SER2_OUT, IGN_OUT, BUZ_OUT";
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
      case CHUTE_OPEN:
        output += String(sysIn.chuteOpened);
      break;
      case SEN_HEALTH:
        output += String(sen.isValid()) + ",";
        output += String(sen.get().gpsMainStatus.isValid) + ",";
        output += String(sen.get().gpsMainStatus.satNumber) + ",";
        output += String(sen.get().gpsAuxStatus.isValid) + ",";
        output += String(sen.get().gpsAuxStatus.satNumber) + ",";
        output += String(sen.get().time.isValid);
      break;
      case POSITION_VALUES:
        output += String(sen.get().position.lat,5) + ",";
        output += String(sen.get().position.lng,5) + ",";
        output += String(sen.get().position.alt);
      break;
      case ATTITUDE_VALUES:
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
        output += String(sen.get().baro.temperature);
      break;
      case PRESSURE:
        if (sen.get().baro.pressure != 0) {
          output += String(sen.get().baro.pressure);
        }
        else {
          output += "99999.0";
        }
      break;
      case MIX_VALUES:
        output += String(sysIn.mix.ventN2O) + ",";
        output += String(sysIn.mix.ventFuel) + ",";
        output += String(sysIn.mix.pressurizer) + ",";
        output += String(sysIn.mix.solenoid4) + ",";
        output += String(sysIn.mix.servoN2O) + ",";
        output += String(sysIn.mix.servoFuel) + ",";
        output += String(sysIn.mix.ignitor) + ",";
        output += String(sysIn.mix.buzzer);
      break;
      case OUTPUT_VALUES:
        output += String(sysIn.out.ventN2O) + ",";
        output += String(sysIn.out.ventFuel) + ",";
        output += String(sysIn.out.pressurizer) + ",";
        output += String(sysIn.out.solenoid4) + ",";
        output += String(sysIn.out.servoN2O) + ",";
        output += String(sysIn.out.servoFuel) + ",";
        output += String(sysIn.out.ignitor) + ",";
        output += String(sysIn.out.buzzer);
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
