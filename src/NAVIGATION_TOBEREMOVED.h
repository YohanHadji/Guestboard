// #ifndef NAVIGATION
// #define NAVIGATION

// #include "Arduino.h"
// #include "CONFIG.h"
// #include "math.h"
// #include "SENSOR.h"
// #include "CONTROL.h"
// #include "SIMULATION.h"

// enum NAVMODE {
//   HEADING_ABSOLUTE = 0,
//   HEADING_RELATIVE,
//   HEADING_RELATIVE_FINAL,
//   COG,
//   MANUAL
// };

// struct waypointData {
//   gpsCoord gpsData[50];
//   double score[50];
//   bool isAvailable[50];
//   int length;
// };

// struct navStatus {
//   gpsCoord waypoint;
//   waypointData waypointChoice;
//   conStatus con;
//   simStatus sim;
//   double groundAltLaunch;
//   double groundAltLanding;
//   NAVMODE navMode;
//   double headingRelativeSetpoint;
//   double projectedGroundSpeed;
//   double windSpeedGFS;
//   double windDirGFS;
//   double windSpeedMeasured;
//   double windDirMeasured;
//   double spiralFactor;
//   bool isCloseToHome;
// };


// class navClass {
//   public:
//     navClass();
//     navStatus get();
//     navStatus compute(senStatus sen, bool wingIsFlying);
//     void setWaypoint(waypointData);
//     void setGroundAltLaunch(double);
//     void setGroundAltLanding(double);
//     void setHeadingRelativeSetpoint(double);
//     void setNavMode(NAVMODE);
//     conClass con;
//     simClass sim;
//     gpsCoord chooseWaypoint(simStatus, double);
//   private: 
//     gpsCoord waypoint; 
//     waypointData waypointChoice;
//     unsigned currentWaypointPos;
//     double groundAltLaunch;
//     double groundAltLanding;
//     NAVMODE navMode;
//     double headingRelativeSetpoint;
//     double projectedGroundSpeed;
//     double windSpeedGFS;
//     double windDirGFS;
//     double windSpeedMeasured;
//     double windDirMeasured;
//     double spiralFactor;
//     bool isCloseToHome;
// };

// double azimuthTo(double, double, double, double);
// double angleLoiterTo(double, double, double, double, double);
// double distanceTo(gpsCoord, gpsCoord);
// #endif
