#include "NAVIGATION.h"

navClass::navClass() 
{
}

navStatus navClass::compute(senStatus sen, bool wingIsFlying) {

  //unsigned timeBefore = micros();
  double currentFalseAirspeed = falseAirspeed(sen.verticalSpeedAvgData, pressureSim(sen.position.alt), temperatureSim(sen.position.alt));
  if (currentFalseAirspeed > -1.75) {
    currentFalseAirspeed = -SPEED_GLIDE_Z;
  }

  sim.run(sen.position.lat, sen.position.lng, sen.position.alt, sen.time.code.second, sen.position.alt, waypoint.alt, SPEED_ASCENT_BALLOON, currentFalseAirspeed);
  //Serial.print(micros()-timeBefore); Serial.print(" "); Serial.println(sen.position.alt);

  waypoint = chooseWaypoint(sim.get(), sen.position.alt);
  groundAltLanding = waypoint.alt;

  con.setTime(sen.time.code.second,sen.time.code.nanosecond);

  if (navMode != COG and navMode != HEADING_ABSOLUTE and navMode != HEADING_RELATIVE and navMode != HEADING_RELATIVE_FINAL and navMode != MANUAL) {
    navMode = HEADING_RELATIVE;
  }

  switch (navMode) {

    case NAVMODE::COG:
      {
        con.setInput(sen.course);
        //con.setInput(sen.attitude.yaw);
        if (CIRCLE_LOITER) {
          con.setPoint(angleLoiterTo(sen.position.lat, sen.position.lng, waypoint.lat, waypoint.lng, sen.position.alt));
        }
        else { 
          con.setPoint(azimuthTo(sen.position.lat, sen.position.lng, waypoint.lat, waypoint.lng));
        }
        if (distanceTo(sen.position, waypoint) < CIRCLE_RADIUS*2*map(sen.position.alt,0,30000,1,10)) {
          isCloseToHome = true;
        }
        else {
          isCloseToHome = false;
        }
      }
    break;

    case NAVMODE::HEADING_ABSOLUTE:
      {
        con.setInput(sen.attitude.yaw);
        simStatus simResult;
        gpsCoord simPoint;
        simResult = sim.get();
        simPoint.lat = simResult.lat;
        simPoint.lng = simResult.lng;
        if (CIRCLE_LOITER) {
          con.setPoint(angleLoiterTo(simResult.lat, simResult.lng, waypoint.lat, waypoint.lng, sen.position.alt));
        }
        else { 
          con.setPoint(azimuthTo(simResult.lat, simResult.lng, waypoint.lat, waypoint.lng));
        }
        if (distanceTo(simPoint, waypoint) < CIRCLE_RADIUS*2*map(sen.position.alt,0,30000,1,10)) {
          isCloseToHome = true;
        }
        else {
          isCloseToHome = false;
        }
      }
    break;

    case NAVMODE::HEADING_RELATIVE:
      {
        con.setInput(sen.attitude.yaw);

        static double projectedGroundSpeedOld = 0;

        simStatus simResult;
        gpsCoord simPoint;
        simResult = sim.get();
        simPoint.lat = simResult.lat;
        simPoint.lng = simResult.lng;

        double angleToHome = angleLoiterTo(simResult.lat, simResult.lng, waypoint.lat, waypoint.lng, sen.position.alt);
        double angleDiff = getAngle(angleToHome, sen.course);

        projectedGroundSpeed = sen.twoDSpeed*cos((angleDiff/180.0)*PI);

        double speedGradient = (projectedGroundSpeed - projectedGroundSpeedOld)/sen.rotationSpeed;

        projectedGroundSpeedOld = projectedGroundSpeed;

        double angleError = getAngle(sen.attitude.yaw, headingRelativeSetpoint);

        if (abs(angleError) < 90) {
          if (speedGradient>0) {
            headingRelativeSetpoint = ((int)((headingRelativeSetpoint+2.0)*100.0)%36000)/100.0;
          }
          else {
            headingRelativeSetpoint = ((int)((headingRelativeSetpoint+358.0)*100.0)%36000)/100.0;
          }
        }
        else {
          headingRelativeSetpoint = headingRelativeSetpoint;
        }

        con.setPoint(headingRelativeSetpoint);
      }
    break;

    case NAVMODE::HEADING_RELATIVE_FINAL:
      {
        con.setInput(sen.attitude.yaw);

        static double projectedGroundSpeedOld = 0;

        double angleToHome = angleLoiterTo(sen.position.lat, sen.position.lng, waypoint.lat, waypoint.lng, sen.position.alt);
        double angleDiff = getAngle(angleToHome, sen.course);

        projectedGroundSpeed = sen.twoDSpeed*cos((angleDiff/180.0)*PI);

        double speedGradient = (projectedGroundSpeed - projectedGroundSpeedOld)/sen.rotationSpeed;

        projectedGroundSpeedOld = projectedGroundSpeed;

        double angleError = getAngle(sen.attitude.yaw, headingRelativeSetpoint);

        if (abs(angleError) < 90) {
          if (speedGradient>0) {
            headingRelativeSetpoint = ((int)((headingRelativeSetpoint+2.0)*100.0)%36000)/100.0;
          }
          else {
            headingRelativeSetpoint = ((int)((headingRelativeSetpoint+358.0)*100.0)%36000)/100.0;
          }
        }
        else {
          headingRelativeSetpoint = headingRelativeSetpoint;
        }

        con.setPoint(headingRelativeSetpoint);
      }
    break;

    case NAVMODE::MANUAL:
      {
        con.setInput(sen.attitude.yaw);
        con.setPoint(headingRelativeSetpoint);

        double angleToHome = angleLoiterTo(sen.position.lat, sen.position.lng, waypoint.lat, waypoint.lng, sen.position.alt);
        double angleDiff = getAngle(angleToHome, sen.course);
        projectedGroundSpeed = sen.twoDSpeed*cos((angleDiff/180.0)*PI);
      }
    break; 
    
    default:
    break;
  }

  unsigned timeCodeFile = (int(sim.getHourModel())%24)*3600.0;
  double timeDiffModel = (sen.time.code.second - timeCodeFile)/3600.0;

  double x = sim.computeWind(sen.position.lat, sen.position.lng, sen.position.alt, timeDiffModel).x;
  double y = sim.computeWind(sen.position.lat, sen.position.lng, sen.position.alt, timeDiffModel).y;

  windSpeedGFS = sqrt(x*x+y*y);

  windDirGFS = ((3.0/2.0)*PI)-atan2(y,x);
  windDirGFS = (windDirGFS/PI)*180.0;
  windDirGFS = (int(windDirGFS*100.0)%36000)/100.0;

  double forwardSpeed = trueAirspeed(SPEED_GLIDE_X, pressureSim(sen.position.alt), temperatureSim(sen.position.alt));

  double airX = 0;
  double airY = 0;

  if (wingIsFlying) {
    airX = forwardSpeed*sin(sen.attitude.yaw/180.0*PI);
    airY = forwardSpeed*cos(sen.attitude.yaw/180.0*PI); 
  }
  else {
    airX = 0;
    airY = 0;
  }

  double gndX = sen.speed.x;
  double gndY = sen.speed.y;

  double windMeasuredX = gndX - airX;
  double windMeasuredY = gndY - airY;

  windSpeedMeasured = sqrt(windMeasuredX*windMeasuredX+windMeasuredY*windMeasuredY);
  windDirMeasured = ((3.0/2.0)*PI)-atan2(windMeasuredY,windMeasuredX);
  windDirMeasured = (windDirMeasured/PI)*180.0;
  windDirMeasured = (int(windDirMeasured*100.0)%36000)/100.0;

  double r = (SPEED_GLIDE_X/SPEED_GLIDE_Z);

  double bX = x*((1/r)-1);
  double bY = y*((1/r)-1);

  double airXSetpoint = forwardSpeed*sin(con.get().setpoint/180.0*PI);
  double airYSetpoint = forwardSpeed*cos(con.get().setpoint/180.0*PI);

  double angleDiff = atan2(airYSetpoint, airXSetpoint) - atan2(bY, bX);

  double normA = sqrt(airXSetpoint*airXSetpoint+airYSetpoint*airYSetpoint);
  double normB = sqrt(bX*bX+bY*bY);

  spiralFactor =  normA - normB*cos(angleDiff);
  con.compute();

  navStatus navOut;
  navOut = get();
  return navOut;
}

navStatus navClass::get() {
  navStatus navOut;
  navOut.con = con.get();
  navOut.sim = sim.get();
  navOut.waypoint = waypoint;
  navOut.waypointChoice = waypointChoice;
  navOut.groundAltLaunch = groundAltLaunch;
  navOut.groundAltLanding = groundAltLanding;
  navOut.navMode = navMode;
  navOut.windSpeedGFS = windSpeedGFS;
  navOut.windDirGFS = windDirGFS;
  navOut.windSpeedMeasured = windSpeedMeasured;
  navOut.windDirMeasured = windDirMeasured;
  navOut.spiralFactor = spiralFactor;
  navOut.isCloseToHome = isCloseToHome;
  navOut.headingRelativeSetpoint = headingRelativeSetpoint;
  navOut.projectedGroundSpeed = projectedGroundSpeed;
  return navOut;
}

void navClass::setWaypoint(waypointData waypointDataIn) {

  waypointChoice = waypointDataIn; 

  if (DEBUG) {
    Serial.println("Waypoint as being saved onboard:");
    for (int i = 0; i < waypointDataIn.length; i++) {
      Serial.print(waypointDataIn.gpsData[i].lat); Serial.print(" "); 
      Serial.print(waypointDataIn.gpsData[i].lng); Serial.print(" ");
      Serial.println(waypointDataIn.gpsData[i].alt); 
    }
  }

  if (waypointDataIn.length !=0) {
    waypoint = waypointDataIn.gpsData[0];
  }

}
void navClass::setGroundAltLaunch(double groundAltIn) { groundAltLaunch = groundAltIn; }
void navClass::setGroundAltLanding(double groundAltIn) { groundAltLanding = groundAltIn; }
void navClass::setHeadingRelativeSetpoint(double headingRelativeSetpointIn) { headingRelativeSetpoint = headingRelativeSetpointIn; }
void navClass::setNavMode(NAVMODE navModeIn) { navMode = navModeIn; }

double azimuthTo(double lat1, double lng1, double lat2, double lng2) {
  lat1 = lat1 * PI / 180;
  lng1 = lng1 * PI / 180;
  lat2 = lat2 * PI / 180;
  lng2 = lng2 * PI / 180;

  double dlon = lng2-lng1;
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += 2*PI;
  }
  return a2/PI*180.0;
}

double angleLoiterTo(double lat1, double lng1, double lat2, double lng2, double alt) {
  double angleToPoint = azimuthTo(lat1, lng1, lat2, lng2);

  float loiterAngle = (-angleToPoint+90) + (CIRCLE_DIR * CIRCLE_CHASE_OFFSET);
  loiterAngle = loiterAngle/180.0*PI;

  double realRadius = CIRCLE_RADIUS*map(alt,0,30000,1,10);

  float E = realRadius * cos(loiterAngle);
  float N = realRadius * sin(loiterAngle);

  double latPoint = lat2;
  double lngPoint = lng2;
  double altPoint = alt;

  enu2lla(E, N, 0, latPoint, lngPoint, altPoint);
  //Serial.print(latPoint,6); Serial.print(","); Serial.println(lngPoint,6);
  return azimuthTo(lat1, lng1, latPoint, lngPoint);
}

gpsCoord navClass::chooseWaypoint(simStatus simResult, double alt) {

  unsigned size = waypointChoice.length;
  double minScore = 999999; // Best score is the smallest one
  int pos = 0;

  gpsCoord kingPoint = waypointChoice.gpsData[0];
  gpsCoord driftPoint;
  driftPoint.lat = simResult.lat;
  driftPoint.lng = simResult.lng;

  bool noAvailableWaypoint = true;

  for (unsigned i(0); i < size; i++) {
    // double part1;
    // double part2;

    double distanceToDrift = distanceTo(waypointChoice.gpsData[i], driftPoint);
    double R = (alt-waypointChoice.gpsData[i].alt)*(abs(SPEED_GLIDE_X)/abs(SPEED_GLIDE_Z));

    // If the point is considered as available, we keep it like that 
    // Unless it really become unavailable
    if (waypointChoice.isAvailable[i]) {
      if (distanceToDrift < R*1.1) { 
        // part1 = 1; 
        noAvailableWaypoint = false;
        waypointChoice.isAvailable[i] = true;
      }
      else {
        // part1 = 0;
        waypointChoice.isAvailable[i] = false;
      }
    }
    // Otherwise if it was previously considered as unavailable, we ask for some margin
    // before considering it as available again. 
    else {
      if (distanceToDrift < R*0.9) { 
        // part1 = 1; 
        noAvailableWaypoint = false;
        waypointChoice.isAvailable[i] = true;
      }
      else {
        // part1 = 0;
        waypointChoice.isAvailable[i] = false;
      }
    }

    // double secondDistance = distanceTo(kingPoint,driftPoint);
    // if (secondDistance < 100) {
    //   secondDistance = 100;
    // };

    // part2 = distanceTo(waypointChoice.gpsData[i],kingPoint)/secondDistance;
    double distanceWaypointToKing = distanceTo(waypointChoice.gpsData[i],kingPoint);
    waypointChoice.score[i] = distanceWaypointToKing;
  }

  for (unsigned i(0); i < size; i++) {
    if (waypointChoice.isAvailable[i]) {
      if (waypointChoice.score[i] < minScore) {
        minScore = waypointChoice.score[i];
        pos = i;
      }
    }
  }

  if (noAvailableWaypoint) {
    double minDistanceToDrift = 999999;
    // Return the closest waypoint to the drift point
    for (unsigned i(0); i < size; i++) {
      double distanceToDrift = distanceTo(waypointChoice.gpsData[i], driftPoint);
      if (distanceToDrift < minDistanceToDrift) {
        minDistanceToDrift = distanceToDrift;
        pos = i;
      }
    }
  }

  currentWaypointPos = pos;
  return waypointChoice.gpsData[currentWaypointPos];
}

double distanceTo(gpsCoord point1, gpsCoord point2) {
  double R = 6371000;
  double lat1 = point1.lat * PI / 180.0;
  double lng1 = point1.lng * PI / 180.0;
  double lat2 = point2.lat * PI / 180.0;
  double lng2 = point2.lng * PI / 180.0;

  double dlat = lat2-lat1;
  double dlng = lng2-lng1;

  double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlng/2) * sin(dlng/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R * c;
  return d;
}
