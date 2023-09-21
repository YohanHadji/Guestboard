#include "STATE.h"

static int buzzerCooldown(1000);

uav::uav(dataListString listIn[30], int len)
  :data(listIn,len)  
{ 
  //Serial.begin(115200);
  //if (DEBUG) { Serial.println("UAV Constructor In"); }
  //buzzer.buzzerTurnOn();
  //if (DEBUG) { Serial.println("UAV Constructor Out"); }
}

// This function will compute all the new "action" variables
void uav::compute() { 
  FLIGHTMODE newFlightMode;
  FLIGHTMODE oldFlightMode;
  // Run the state machine and check if there is a transition to another flight mode
  oldFlightMode = sys.get().flightMode;
  newFlightMode = flightState(); 
  sys.setTime(data.sen.get().time.code);
  sys.setFlightMode(newFlightMode);

  // We only run the navigation if we are initialised because we need correct GPS coordinates. 
  if (sys.get().initialised and sys.get().flightMode != CONFIGERROR) { // and sys.get().flightMode != READYSTEADY, removed, I think it was a mistake to have it here.
    double localAltitude  = data.sen.get().position.alt;
    double localDensity   = density(pressureSim(localAltitude),temperatureSim(localAltitude));
    double groundDensity  = density(pressureSim(0),temperatureSim(0));
    sys.setGainDivider(groundDensity/localDensity);
    sys.setRealVDOWN(trueAirspeed(VDOWN, pressureSim(localAltitude), temperatureSim(data.sen.get().position.alt)));
    // Then the PID loop is only active if we are in an autonomous mode, to avoid I term windup.
    sys.nav.con.isActive(isAutonomous());
    sys.nav.compute(data.get().sen, isAutonomous()); 
  }

  // However, we always output the servo signal have them in the idle position even if we are not initialised
  sys.ser.compute(sys.mix.compute(sys.get()));

  if (newFlightMode != oldFlightMode) {

    // We don't really care about transitions from mode 1 to 2 or mode 2 to 1 so we don't send data in that case
    bool condition1 = (newFlightMode == ASCENT) && (oldFlightMode == READYSTEADY);
    bool condition2 = (newFlightMode == READYSTEADY) && (oldFlightMode == ASCENT);
    if (!condition1 and !condition2) {
      data.send(sys.get());
    }
  }
}

void uav::output() {
  sys.ser.write();
  // Data save is run in last because it is the most time consuming function and we have 50ms before the next sensor packet.
  data.save(sys.get());
}

void uav::update() {
  buzzer.update();
  led.update();
}


// Return the next flight mode based on current dataset
FLIGHTMODE uav::flightState() {

  FLIGHTMODE flightModeIn;
  FLIGHTMODE flightModeOut;

  flightModeIn = sys.get().flightMode;
  flightModeOut = INITIALIZE;

  if (data.cmd.isUpdated()) {
    flightModeOut = executeCmd(flightModeIn,data.cmd.get());
  }

  else { // Depending on which flight mode, various checks are carried out
    switch(flightModeIn) {
      case INITIALIZE:
        flightModeOut = flightInit();
      break;
      case READYSTEADY:
        flightModeOut = readySteady();
      break;
      case ASCENT:
        flightModeOut = flightAscent();
      break;
      case DESCENT:
        flightModeOut = flightDescent();
      break;
      case GLIDING:
        flightModeOut = flightGliding();
      break;
      case GLIDINGAUTO:
        flightModeOut = flightGlidingAuto();
      break;
      case SPIRAL:
        flightModeOut = flightSpiral();
      break;
      case GLIDINGRECOVER:
        flightModeOut = flightGlidingRecover();
      break;
      case GLIDINGNOGPS:
        flightModeOut = flightGlidingNoGPS();
      break;
      case ABORT:
        flightModeOut = abort();
      break;
      case CONFIGERROR:
        flightModeOut = configError();
      break;
      case FLYBYWIRE:
        flightModeOut = flyByWire();
      break;
      default:
        flightModeOut = INITIALIZE;
      break;
    } 
  }

  if (flightModeOut != flightModeIn) {
    buzzer.buzzerChangeFlightMode();
    // Transition time is used all over the programm to know exactly when we did the last mode transition
    sys.setTransitionTime(sys.get().time);
  }
  led.switchColor(sys.get().flightMode);

  return flightModeOut;
}

// Wait for GPS to be ready
FLIGHTMODE uav::flightInit() {

  FLIGHTMODE flightModeOut;
  flightModeOut = INITIALIZE;

  if (checkConfig() != NOERROR) {
    flightModeOut = CONFIGERROR;
    return flightModeOut;
  }
  
  if(buzzerCooldown >= 100) {
    buzzer.buzzerInit();
    buzzerCooldown = 0;
  } 
  else {
    ++buzzerCooldown;
  }
  
  if (data.get().sen.valid) {

    //Serial.println("GPS is ready");
    if (!sys.isInitialised()) {
      // If RETURN_MODE is 1, then set waypoint as the current position
      if (RETURN_MODE == 0) {
        waypointData waypointChoice;
        waypointChoice.gpsData[0] = data.get().sen.position;
        waypointChoice.length = 1;
        sys.nav.setWaypoint(waypointChoice);
        sys.nav.setGroundAltLanding(data.get().sen.position.alt);
      }
      sys.nav.setGroundAltLaunch(data.get().sen.position.alt);
      sys.setFlightMode(READYSTEADY);

      if (checkConfig() != NOERROR) {
        flightModeOut = CONFIGERROR;
      }
    }
    else {
      flightModeOut = READYSTEADY;
    }

    if (INFLIGHT_REBOOT) {
      if (data.get().sen.position.alt > INFLIGHT_REBOOT_ALTITUDE) {
        double realVdownReboot = trueAirspeed(INFLIGHT_REBOOT_VERTICAL_SPEED, pressureSim(data.sen.get().position.alt), temperatureSim(data.sen.get().position.alt));
        if (data.get().sen.speed.z < realVdownReboot) {
          sys.setRecoveryPhase(BRAKE);
          flightModeOut = GLIDINGRECOVER;
        }
      }
    }

    sys.setReady();
    buzzer.buzzerInitEnd();
  }
  return flightModeOut;
}

// Check to go in ascent or descent mode, this is just a transition mode, nothing special.
FLIGHTMODE uav::readySteady() { 

  FLIGHTMODE flightModeOut;
  flightModeOut = READYSTEADY;

  if (checkConfig() != NOERROR) {
    flightModeOut = CONFIGERROR;
    return flightModeOut;
  }

  if (!data.get().sen.valid) {
    flightModeOut = INITIALIZE;
    return flightModeOut;
  }

  if (data.get().sen.speed.z > VUP) {
    flightModeOut = ASCENT;
  }
  if (data.get().sen.speed.z < sys.get().realVDOWN) {
    flightModeOut = DESCENT;
  }
  
  return flightModeOut;
}

// Ascent mode, if Z speed is lower than VUP go back to readysteady mode, if we reach the separation altitude, separate.
FLIGHTMODE uav::flightAscent() {

  FLIGHTMODE flightModeOut;
  flightModeOut = ASCENT;

  if (!data.get().sen.valid) {
    flightModeOut = INITIALIZE;
    return flightModeOut;
  }

  if (data.get().sen.speed.z < VUP) {
    flightModeOut = READYSTEADY;
  }

  // Switch between ASL altitude and ground relative altitude. 
  switch (SEP_ALT_MODE) {
    case 0:
      if (data.get().sen.position.alt > SEP_ALT) {
        sys.separate();
      }
    break;
    case 1:
      if ((data.get().sen.position.alt-sys.get().nav.groundAltLaunch) > SEP_ALT) {
        sys.separate();
      }
    break;
    default:
      if (data.get().sen.position.alt > SEP_ALT) {
        sys.separate();
      }
    break;
  }  
  return flightModeOut;
}

// Descent mode, if Z speed is higher than VDOWN go back to ready mode and deploy wing if needed.
FLIGHTMODE uav::flightDescent() {

  FLIGHTMODE flightModeOut;
  flightModeOut = DESCENT;

  if (!data.get().sen.valid) {
    flightModeOut = INITIALIZE;
    return flightModeOut;
  }
  
  if (data.get().sen.speed.z > sys.get().realVDOWN) {
    flightModeOut = READYSTEADY;
  }

  // We have two differents deployment mode, one using a target altitude, and one using a timer
  // If DEP_MODE is at 0, we will wait to be in decent mode for more than DESCENT_TIMER secondes before 
  // deploying the wing. If DEP_MODE is set to 1, we will wait until we go lower than a DEP_ALT to deploy. 
  // This DEP_ALT can be ASL or relative to ground level depending on the DEP_ALT_MODE setting.

  bool safetyTimer = (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= DESCENT_TIMER);
  RECOVERYPHASE initialRecoveryPhase = RECOVERYPHASE::STALLRAMP;

  switch (DEP_MODE) {

    case 0:
      if (safetyTimer) {
        flightModeOut = GLIDINGRECOVER;
        sys.setRecoveryPhase(initialRecoveryPhase);
      }
    break;

    case 1:
    // ASL altitude VS relative to ground. 
    switch (DEP_ALT_MODE) {

        case 0:
          if (safetyTimer and data.get().sen.position.alt < DEP_ALT) {
            flightModeOut = GLIDINGRECOVER;
            sys.setRecoveryPhase(initialRecoveryPhase);
          }
        break;

        case 1:
          if (safetyTimer and (data.get().sen.position.alt-sys.get().nav.groundAltLanding) < DEP_ALT) {
            flightModeOut = GLIDINGRECOVER;
            sys.setRecoveryPhase(initialRecoveryPhase);
          }
        break;

        default:
          if (safetyTimer and data.get().sen.position.alt < DEP_ALT) {
            flightModeOut = GLIDINGRECOVER;
            sys.setRecoveryPhase(initialRecoveryPhase);
          }
        break;

      }
    break;
  } 
  return flightModeOut;
}

// Gliding mode. 
FLIGHTMODE uav::flightGliding() {

  static double trim; 

  FLIGHTMODE flightModeOut;
  flightModeOut = GLIDING;

  // If we disabled in fligh trim or we already did it successfully skip this part. 
  if (!INFLIGHT_TRIM or sys.get().trimDone) {
    sys.openWing();
    flightModeOut = GLIDINGAUTO;
  }
  // Otherwise (so if in flight trim is activated AND trim is not done yet) we try to stabilize the wing
  // using a simple P integration controller. 
  else if (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) < INFLIGHT_TRIM_TIMER) {
    sys.openWing();
    double trimDelta = (data.get().sen.rotationSpeed/TRIM_TIME_FACTOR)/SENSOR_RATE;
    trimDelta = constrain(trimDelta, -0.4, 0.4);
    trim -= trimDelta;
    trim = constrain(trim, TRIM_MIN, TRIM_MAX);
    sys.ser.setDirTrim(int(trim));
  }
  // After INFLIGHT_TRIM_TIMER delay, we check if the rotation speed is stabilized. If yes, go to gliding
  // If no, it means the wing is not fully deployed, go back to deployment mode. 
  else { 
    if (abs(data.sen.get().rotationSpeedAvgData)<15) {
      sys.trimIsDone();
      flightModeOut = GLIDINGAUTO;
      sys.nav.setHeadingRelativeSetpoint(data.sen.get().attitude.yaw);
    }
    else {
      trim = 0;
      sys.ser.setDirTrim(int(trim));
      flightModeOut = GLIDINGRECOVER;
      sys.setRecoveryPhase(STALLRAMP);
    }
  }
  return flightModeOut;
}

// Gliding Auto
// In this flight mode the system is in automatic controls
FLIGHTMODE uav::flightGlidingAuto() {

  FLIGHTMODE flightModeOut;
  flightModeOut = GLIDINGAUTO;

  // The realVSPIRAL speed is adjusted with altitude, the higher we go, the higher the airspeed will be
  // so our we need to ask for a higher vertical speed to detect an unwanted spiral at high altitude than
  // at lower altitude. 
  double realVSPIRAL = trueAirspeed(VSPIRAL, pressureSim(data.sen.get().position.alt), temperatureSim(data.sen.get().position.alt));
  if (data.get().sen.speed.z < realVSPIRAL) {
    flightModeOut = GLIDINGRECOVER;
    sys.setRecoveryPhase(BRAKE);
  }

  // If there is a lack of gps connection, switch to a "blind" gliding mode
  if (!data.get().sen.valid) {
    flightModeOut = GLIDINGNOGPS;
  }

  // R2HOME has two navigation modes, the number 1 is for high altitude and will use a fancy navigation method 
  // featuring trajectory prediction using a wing model. The second navigation mode will use a simpler method 
  // of control with GPS course over ground as an input. The transition alt can be ASL or relative to ground
  // depending on the TRANSITION_ALT_MODE setting. 
  switch (TRANSITION_ALT_MODE) {
    case 0:
      if (data.get().sen.position.alt < TRANSITION_ALT) {
        sys.nav.setNavMode(COG);
      }
      else {
        sys.nav.setNavMode(HEADING_RELATIVE);
      }
    break;

    case 1:
      if (data.get().sen.position.alt-sys.get().nav.groundAltLanding < TRANSITION_ALT) {
        sys.nav.setNavMode(COG);
      }
      else {
        sys.nav.setNavMode(HEADING_RELATIVE);
      }
    break;

    default:
      if (data.get().sen.position.alt-sys.get().nav.groundAltLanding < TRANSITION_ALT) {
        sys.nav.setNavMode(COG);
      }
      else {
        sys.nav.setNavMode(HEADING_RELATIVE);
      }
    break;
  }

  // The spiral mode is a cheating method to win some glide ratio when it's more efficient to fall faster than
  // to glide against the wind. In this situation, we use the spiral as a great thing, we don't want to fight it. 
  if (SPIRAL_ENABLE) {
    // Only do spirals if we are in high altitude navigation mode 
    if (sys.nav.get().navMode != HEADING_RELATIVE_FINAL) {
      // Only do spirals if we are higher than the low altitude limit 
      switch (SPIRAL_ALT_MODE) {
        case 0:
          if (data.get().sen.position.alt > SPIRAL_ALT) {
            // Only do spirals if it's efficient (the factor is <0)
            if (sys.nav.get().spiralFactor<-1 && (sys.nav.get().isCloseToHome==0)) {
              flightModeOut = SPIRAL;
            }
          }
        break;

        case 1:
          if (data.get().sen.position.alt-sys.get().nav.groundAltLanding > SPIRAL_ALT) {
            if (sys.nav.get().spiralFactor<-1 && (sys.nav.get().isCloseToHome==0)) {
              flightModeOut = SPIRAL;
            }
          }
        break;

        default:
          if (data.get().sen.position.alt-sys.get().nav.groundAltLanding > SPIRAL_ALT) {
            if (sys.nav.get().spiralFactor<-1 && (sys.nav.get().isCloseToHome==0)) {
              flightModeOut = SPIRAL;
            }
          }
        break;
      }
    }
  }

  // When we get really clode to ground level, we might want to reduce the max command to do slower turns
  // and also brake to reduce the speed before touchdown. The altitude for this transition can be set as ASL
  // or above ground level. 
  switch(BRAKE_ALT_MODE) {
    case 0:
      if (data.get().sen.position.alt < BRAKE_ALT) {
        sys.isNearGround();
      }
    break;

    case 1:
      if (data.get().sen.position.alt-sys.get().nav.groundAltLanding < BRAKE_ALT) {
        sys.isNearGround();
      }
    break;

    default:
      if (data.get().sen.position.alt-sys.get().nav.groundAltLanding < BRAKE_ALT) {
        sys.isNearGround();
      }
    break;
  }
  return flightModeOut;
}

// Spiral mode, we continuously check if it's still more efficient to do the spiral than to glide against the wind
// and adjust based on that. 
FLIGHTMODE uav::flightSpiral() {
  
    FLIGHTMODE flightModeOut;
    flightModeOut = SPIRAL;
  
    // We want to avoid doing spirals close to the ground, this is a safety measure. 
    switch (SPIRAL_ALT_MODE) {
      case 0:
        if (data.get().sen.position.alt < SPIRAL_ALT) {
          flightModeOut = GLIDINGRECOVER;
          sys.setRecoveryPhase(BRAKE);
        }
      break;

      case 1:
        if (data.get().sen.position.alt-sys.get().nav.groundAltLanding < SPIRAL_ALT) {
          flightModeOut = GLIDINGRECOVER;
          sys.setRecoveryPhase(BRAKE);
        }
      break;

      default:
        if (data.get().sen.position.alt-sys.get().nav.groundAltLanding < SPIRAL_ALT) {
          flightModeOut = GLIDINGRECOVER;
          sys.setRecoveryPhase(BRAKE);
        }
      break;
    }

    // If doing spirals is not efficient anymore, glide again. 
    if (sys.nav.get().spiralFactor>0) {
      flightModeOut = GLIDINGRECOVER;
      sys.setRecoveryPhase(BRAKE);
    }

    // If there is a lack of gps connection, switch to a "blind" gliding mode
    if (!data.get().sen.valid) {
      flightModeOut = GLIDINGNOGPS;
    }
    return flightModeOut;
}

// Recover mode, stabilize R2HOME after detecting a spiral
FLIGHTMODE uav::flightGlidingRecover() {

  FLIGHTMODE flightModeOut;
  flightModeOut = GLIDINGRECOVER;

  switch (sys.get().recoveryPhase) {
    case STALLRAMP:
      if (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= RECOVER_STALL_RAMP_TIMER) {
        flightModeOut = GLIDINGRECOVER;
        sys.setTransitionTime(sys.get().time);
        sys.setRecoveryPhase(STALL);
      }
    break;
    case STALL:
      if (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= RECOVER_STALL_TIMER) {
        flightModeOut = GLIDINGRECOVER;
        sys.setTransitionTime(sys.get().time);
        sys.setRecoveryPhase(BRAKERAMP);
      }
    break;

    case BRAKERAMP:
      if (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= RECOVER_BRAKE_RAMP_TIMER) {
        flightModeOut = GLIDINGRECOVER;
        sys.setTransitionTime(sys.get().time);
        sys.setRecoveryPhase(BRAKE);
      }
    break;
    case BRAKE:
      if (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= RECOVER_BRAKE_TIMER) {
        flightModeOut = GLIDINGRECOVER;
        sys.setTransitionTime(sys.get().time);
        sys.setRecoveryPhase(HANDSUPRAMP);
      }
    break;

    case HANDSUPRAMP:
      if (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= RECOVER_HANDSUP_RAMP_TIMER) {
        flightModeOut = GLIDINGRECOVER;
        data.sen.resetVerticalSpeed();
        sys.setTransitionTime(sys.get().time);
        sys.setRecoveryPhase(HANDSUP);
      }
    break;
    case HANDSUP:
      if (data.sen.timeDiff(sys.get().time,sys.get().timeTransition) >= RECOVER_HANDSUP_TIMER) {
        flightModeOut = GLIDING;
        data.sen.resetVerticalSpeed();
        sys.setTransitionTime(sys.get().time);
        sys.setRecoveryPhase(STALLRAMP);
      }
    break;

    default:
      flightModeOut = GLIDING;
      data.sen.resetVerticalSpeed();
      sys.setTransitionTime(sys.get().time);
      sys.setRecoveryPhase(STALLRAMP);
    break;
  }
  return flightModeOut;
}

// No Gps mode, Go in a straight ligne until the gps signal is recovered
FLIGHTMODE uav::flightGlidingNoGPS() {

  FLIGHTMODE flightModeOut;
  flightModeOut = GLIDINGNOGPS;

  // If the gps connection is recovered, switch to a normal gliding mode
  if (data.get().sen.valid) {
    flightModeOut = GLIDINGAUTO;
  }
  return flightModeOut;
}

// Ground mode, does nothing. 
FLIGHTMODE uav::abort() {

  FLIGHTMODE flightModeOut;
  flightModeOut = ABORT;

  return flightModeOut;
}

FLIGHTMODE uav::configError() {
  
    FLIGHTMODE flightModeOut;
    flightModeOut = CONFIGERROR;

    if (checkConfig() == NOERROR) {
      flightModeOut = INITIALIZE;
    }
  
    return flightModeOut;
}

bool uav::isAutonomous() {
  if (sys.get().flightMode == FLIGHTMODE::GLIDINGAUTO or sys.get().flightMode == FLIGHTMODE::FLYBYWIRE) {
    return true;
  }
  return false;
}

ERRORCODE uav::checkConfig() {

  worldLimit limit = sys.nav.sim.getWorldLimit();

  File windFile = SD.open("windFile.txt", FILE_READ);  
  if (!windFile) { 
    Serial.println("ERROR - No wind file detected");
    led.noWindFile();
    buzzer.noWindFile();
    return NOWINDFILE;
  }

  if (RETURN_MODE == 1 and (sys.nav.get().waypoint.lat == 0 or sys.nav.get().waypoint.lng == 0)) {
    Serial.println("ERROR - No waypoints detected");
    led.noWaypoint();
    buzzer.noWaypoint();
    return NOWAYPOINTFILE;
  }

  if (RETURN_MODE == 1) {
    for (int i = 0; i < sys.nav.get().waypointChoice.length; i++) {
      if (sys.nav.get().waypointChoice.gpsData[i].lat > limit.latMax or
          sys.nav.get().waypointChoice.gpsData[i].lat < limit.latMin or
          sys.nav.get().waypointChoice.gpsData[i].lng > limit.lngMax or
          sys.nav.get().waypointChoice.gpsData[i].lng < limit.lngMin){
        Serial.println("ERROR - Waypoints our of range");
        led.waypointOutOfRange();
        buzzer.waypointOutOfRange();
        return WAYPOINTRANGE;
      }
    }
  }

  if (sys.get().flightMode != INITIALIZE) {

    /* simStatus simResult = sys.nav.sim.run(data.get().sen.position.lat, 
                                          data.get().sen.position.lng, 
                                          data.get().sen.position.alt, 
                                          data.get().sen.time.code, 
                                          SEP_ALT, 
                                          sys.nav.get().waypoint.alt); */

    if ((sys.get().time.second-(sys.nav.sim.getHourModel()*3600.0))/3600.0 > (limit.timeMin-1) and 
        (sys.get().time.second-(sys.nav.sim.getHourModel()*3600.0))/3600.0 < limit.timeMin) {
      Serial.println("");
      Serial.println(sys.get().time.second-(sys.nav.sim.getHourModel()*3600.0));
      Serial.println(sys.nav.sim.getHourModel()*3600.0);
      Serial.println("WARNING - Need to wait before launch");
      led.timeWait();
      return TIMEWAIT;
    }
    else if ((sys.get().time.second-(sys.nav.sim.getHourModel()*3600.0))/3600.0 < (limit.timeMin-1) or 
        (sys.get().time.second-(sys.nav.sim.getHourModel()*3600.0))/3600.0 > limit.timeMax) {
      Serial.println("");
      Serial.println(sys.get().time.second-(sys.nav.sim.getHourModel()*3600.0));
      Serial.println(sys.nav.sim.getHourModel()*3600.0);
      Serial.println("ERROR - Time out of range");
      led.timeOutOfRange();
      buzzer.timeOutOfRange();
      return TIMERANGE;
    }

     if (data.get().sen.position.lat > limit.latMax or 
        data.get().sen.position.lat < limit.latMin or 
        data.get().sen.position.lng > limit.lngMax or 
        data.get().sen.position.lng < limit.lngMin) {
      Serial.println("ERROR - Position out of range");
      Serial.print("Range was : "); 
      Serial.print(" latitude : ");
      Serial.print(limit.latMin); Serial.print(" to "); Serial.print(limit.latMax);
      Serial.print(" longitude : ");
      Serial.print(limit.lngMin); Serial.print(" to "); Serial.println(limit.lngMax);
      Serial.print("Waypoint was : ");
      Serial.print(" latitude : "); Serial.print(data.get().sen.position.lat);
      Serial.print(" longitude : "); Serial.println(data.get().sen.position.lng);
      led.positionOutOfRange();
      buzzer.positionOutOfRange();
      return POSRANGE;
    }

    /* if (simResult.errorCode != 0) {
      Serial.println("ERROR - Simulation getting out of time or space range");
      return true;
    } */
  }
  return NOERROR; 
}

FLIGHTMODE uav::flyByWire() {
  FLIGHTMODE flightModeOut;
  flightModeOut = FLYBYWIRE;

  sys.nav.setNavMode(NAVMODE::MANUAL);
  return flightModeOut;
}

FLIGHTMODE uav::executeCmd(FLIGHTMODE flightModeIn, cmdStatus cmd) {
  FLIGHTMODE flightModeOut = INITIALIZE;
  switch(cmd.tlmCmd) {

    case TLM_CMD::SEPARATE_CMD:
      sys.separate();
      flightModeOut = flightModeIn;
    break;

    case TLM_CMD::DEPLOY_CMD:
      if (flightModeIn != ASCENT) {
        sys.setRecoveryPhase(BRAKE);
        flightModeOut = GLIDINGRECOVER;
      }
      else {
        flightModeOut = flightModeIn;
      }
    break;

    case TLM_CMD::RECOVER_CMD:
      sys.setRecoveryPhase(STALLRAMP);
      flightModeOut = GLIDINGRECOVER;
    break;

    case TLM_CMD::SPIRAL_CMD:
      flightModeOut = SPIRAL;
    break;

    case TLM_CMD::ABORT_CMD:
      flightModeOut = ABORT;
    break;

    case TLM_CMD::UNABORT_CMD:
      flightModeOut = READYSTEADY;
    break;

    case TLM_CMD::PING_CMD:
      data.send(sys.get());
      flightModeOut = flightModeIn;
    break;

    case TLM_CMD::MANUAL_CMD:
      if (flightModeIn == GLIDINGAUTO) {
        flightModeOut = FLYBYWIRE;
      }
      else {
        flightModeOut = flightModeIn;
      }
    break;

    case TLM_CMD::AUTO_CMD:
      if (flightModeIn == FLYBYWIRE) {
        flightModeOut = GLIDINGAUTO;
      }
      else {
        flightModeOut = flightModeIn;
      }
    break;

    case TLM_CMD::MANUAL_LEFT_CMD:
     if (flightModeIn == FLYBYWIRE) {
        double newHeading;
        newHeading = int((sys.nav.get().headingRelativeSetpoint-MANUAL_HEADING_INCR)+360)%360;
        sys.nav.setHeadingRelativeSetpoint(newHeading);
        flightModeOut = FLYBYWIRE;
      }
      else {
        flightModeOut = flightModeIn;
      }
    break;

    case TLM_CMD::MANUAL_RIGHT_CMD:
      if (flightModeIn == FLYBYWIRE) {
        double newHeading;
        newHeading = int((sys.nav.get().headingRelativeSetpoint+MANUAL_HEADING_INCR)+360)%360;
        sys.nav.setHeadingRelativeSetpoint(newHeading);
        flightModeOut = FLYBYWIRE;
      }
      else {
        flightModeOut = flightModeIn;
      }
    break;

    case TLM_CMD::NO_CMD:
      flightModeOut = flightModeIn;
    break;

    default:
      flightModeOut = flightModeIn;
    break;
  }
  return flightModeOut;
}