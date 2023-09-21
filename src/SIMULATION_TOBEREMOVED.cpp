// #include "SIMULATION.h"

// const int i = 17;
// const int j = 17;
// const int k = 33;
// const int q = 5;

// static short v[i][j][k][q][3];

// static double x[i];
// static double y[j];
// static double z[k];
// static double t[q];

// simClass::simClass()
// {
// }

// simStatus simClass::get() {
//     simStatus coord;
//     coord.lat = lat;
//     coord.lng = lng;
//     return coord;
// }

// worldLimit simClass::getWorldLimit() {
//     worldLimit limit;
//     limit.latMin = x[0];
//     limit.latMax = x[i-1];
//     limit.lngMin = y[0];
//     limit.lngMax = y[j-1];
//     limit.timeMin = t[0];
//     limit.timeMax = t[q-1];
//     return limit;
// }

// simStatus simClass::run(double latIn, double lngIn, double altIn, double timeIn, double altBurst, double altFinalIn, double ascentSpeed, double descentSpeed) {

//     lat = latIn;
//     lng = lngIn;
//     alt = altIn;
//     time = timeIn;
//     altFinal = altFinalIn;
//     int errorCode = 0;

//     double timeStep = ((alt-altFinal)/500.0);
//     unsigned timeCodeFile = (int(metadata[0])%24)*3600.0;
//     timeStep = constrain(timeStep, 3, 300);

//     worldLimit limit = getWorldLimit();

//     while (alt<altBurst) {
//         xyCoord wind;
//         unsigned timeDiffModel = time - timeCodeFile;
//         wind = computeWind(lat,lng,alt,(timeDiffModel)/3600.0);
//         double U = ascentSpeed;
//         enu2lla(wind.x*timeStep, wind.y*timeStep, U,lat,lng,alt);
//         time += timeStep;

//         if ((time-(metadata[0]*3600.0))/3600.0 < limit.timeMin or 
//             (time-(metadata[0]*3600.0))/3600.0 > limit.timeMax) {
//             errorCode = 1;
//         }
//         else if (lat > limit.latMax or 
//             lat < limit.latMin or 
//             lng > limit.lngMax or 
//             lng < limit.lngMin) {
//             errorCode = 2;
//         }
//     }

//     while (alt>altFinal) {
//         xyCoord wind;
//         unsigned timeDiffModel = time - timeCodeFile;
//         wind = computeWind(lat,lng,alt,(timeDiffModel)/3600.0);

//         double U = trueAirspeed(descentSpeed, pressureSim(alt), temperatureSim(alt));

//         if ((alt-altFinal) < abs(U*timeStep)) {
//           timeStep = (alt-altFinal)/abs(U)+1;
//         }

//         enu2lla(wind.x*timeStep, wind.y*timeStep, U*timeStep,lat,lng,alt);
//         time += timeStep;

//         if ((time-(metadata[0]*3600.0))/3600.0 < limit.timeMin or 
//             (time-(metadata[0]*3600.0))/3600.0 > limit.timeMax) {
//             errorCode = 1;
//         }
//         else if (lat > limit.latMax or 
//             lat < limit.latMin or 
//             lng > limit.lngMax or 
//             lng < limit.lngMin) {
//             errorCode = 2;
//         }
//     }

//     simStatus result;
//     result.lat = lat;
//     result.lng = lng;
//     result.errorCode = errorCode;
//     return result;
// }

// xyCoord simClass::computeWind(double lat, double lon, double alt, double time) { 

//   int pC = pCoord(lat, lon, alt, time);
//   double pLow = z[pC];
//   double pHigh = z[pC+1];
//   xyCoord windLow   = interpWind(lat, lon, pLow, time);
//   xyCoord windHigh  = interpWind(lat, lon, pHigh, time);
//   double altLow    = interpAlt(lat, lon, pLow, time);
//   double altHigh   = interpAlt(lat, lon, pHigh, time);

//   xyCoord windResult;
//   windResult.x = windLow.x + (windHigh.x-windLow.x)/(altHigh-altLow)*(alt-altLow);
//   windResult.y = windLow.y + (windHigh.y-windLow.y)/(altHigh-altLow)*(alt-altLow);

//   return windResult;
// }

// xyCoord simClass::interpWind(double xi, double yi, double zi, double ti) {
//     xyCoord windResult;
//     windResult.x = interp(xi, yi, zi, ti, WIND_U);
//     windResult.y = interp(xi, yi, zi, ti, WIND_V);
//     return windResult;
// }

// double simClass::interpAlt(double xi, double yi, double zi, double ti) {
//     return interp(xi, yi, zi, ti, ALTITUDE);
// }

// double simClass::interp(double xi, double yi, double zi, double ti, DATA_TYPE type) {

//     int xLow  = xCoord(xi);
//     int xHigh = xLow+1;

//     int yLow  = yCoord(yi);
//     int yHigh = yLow+1;

//     int zLow  = zCoord(zi);
//     int zHigh  = zLow+1;

//     int tLow  = tCoord(ti); 
//     int tHigh = tLow+1;

//     double dtVal = (ti-t[tLow]);
//     double dtGrid = (t[tHigh]-t[tLow]);

//     double dzVal = (zi-z[zLow]);
//     double dzGrid = (z[zHigh]-z[zLow]);

//     double dyVal = (yi-y[yLow]);
//     double dyGrid = (y[yHigh]-y[yLow]);

//     double dxVal = (xi-x[xLow]);
//     double dxGrid = (x[xHigh]-x[xLow]);

//     double u1   = v[xLow ][yLow ][zLow ][tLow ][type];
//     double u2   = v[xHigh][yLow ][zLow ][tLow ][type];
//     double u3   = v[xLow ][yHigh][zLow ][tLow ][type];
//     double u4   = v[xHigh][yHigh][zLow ][tLow ][type];
//     double u5   = v[xLow ][yLow ][zHigh][tLow ][type];
//     double u6   = v[xHigh][yLow ][zHigh][tLow ][type];
//     double u7   = v[xLow ][yHigh][zHigh][tLow ][type];
//     double u8   = v[xHigh][yHigh][zHigh][tLow ][type];

//     double u9   = v[xLow ][yLow ][zLow ][tHigh][type];
//     double u10  = v[xHigh][yLow ][zLow ][tHigh][type];
//     double u11  = v[xLow ][yHigh][zLow ][tHigh][type];
//     double u12  = v[xHigh][yHigh][zLow ][tHigh][type];
//     double u13  = v[xLow ][yLow ][zHigh][tHigh][type];
//     double u14  = v[xHigh][yLow ][zHigh][tHigh][type];
//     double u15  = v[xLow ][yHigh][zHigh][tHigh][type];
//     double u16  = v[xHigh][yHigh][zHigh][tHigh][type];

//     double t1 = u1+(((u9-u1)/dtGrid)*dtVal);
//     double t2 = u2+(((u10-u2)/dtGrid)*dtVal);
//     double t3 = u3+(((u11-u3)/dtGrid)*dtVal);
//     double t4 = u4+(((u12-u4)/dtGrid)*dtVal);
//     double t5 = u5+(((u13-u5)/dtGrid)*dtVal);
//     double t6 = u6+(((u14-u6)/dtGrid)*dtVal);
//     double t7 = u7+(((u15-u7)/dtGrid)*dtVal);
//     double t8 = u8+(((u16-u8)/dtGrid)*dtVal);

//     double z1 = t1+(((t5-t1)/dzGrid)*dzVal);
//     double z2 = t2+(((t6-t2)/dzGrid)*dzVal);
//     double z3 = t3+(((t7-t3)/dzGrid)*dzVal);
//     double z4 = t4+(((t8-t4)/dzGrid)*dzVal);

//     double y1 = z1+(((z3-z1)/dyGrid)*dyVal);
//     double y2 = z2+(((z4-z2)/dyGrid)*dyVal);

//     double x1 = y1+(((y2-y1)/dxGrid)*dxVal);

//     switch (type) {
//         case WIND_U:
//         case WIND_V:
//             x1 = x1/100.0;
//         break;
//         case ALTITUDE:
//             x1 = x1;
//         break;
//         default:
//         break;
//     }
    
//     return x1;
// }

// int simClass::xCoordBis(double xi) {
//   for (int l(0); l<i; l++) {
//     if (x[l]>xi) {
//       if (l>0) {
//         return l-1;
//       }
//       else {
//         return l;
//       }
//     }
//   }
//   return i;
// }

// // same as xCoord1 but with a dichotomy search 
// int simClass::xCoord(double xi) {
//   int l(0);
//   int r(i-1);
//   int m;
//   while (l<r) {
//     m = (l+r)/2;
//     if (x[m]>=xi) {
//       r = m;
//     }
//     else {
//       l = m+1;
//     }
//   }
//   return max(l-1,0);
// }

// int simClass::yCoordBis(double yi) {
//   for (int l(0); l<j; l++) {
//     if (y[l]>yi) {
//       if (l>0) {
//         return l-1;
//       }
//       else {
//         return l;
//       }
//     }
//   }
//   return j;
// }

// int simClass::yCoord(double yi) {
//   int l(0);
//   int r(j-1);
//   int m;
//   while (l<r) {
//     m = (l+r)/2;
//     if (y[m]>=yi) {
//       r = m;
//     }
//     else {
//       l = m+1;
//     }
//   }
//   return max(l-1,0);
// }


// int simClass::zCoordBis(double zi) {
//   for (int l(0); l<k; l++) {
//     if (z[l]<zi) {
//       if (l>0) {
//         return l-1;
//       }
//       else {
//         return l;
//       }
//     }
//   }
//   return k;
// }

// int simClass::zCoord(double zi) {
//   int l(0);
//   int r(k-1);
//   int m;
//   while (l<r) {
//     m = (l+r)/2;
//     if (z[m]<=zi) {
//       r = m;
//     }
//     else {
//       l = m+1;
//     }
//   }
//   return max(l-1,0);
// }


// int simClass::pCoordBis(double xi, double yi, double zi, double ti) {
//   for (int l(0); l<k; l++) {
//     if (interpAlt(xi, yi, z[l], ti)>zi) {
//       if (l>0) {
//         return l-1;
//       }
//       else {
//         return l;
//       }
//     }
//   }
//   return k;  
// }

// int simClass::pCoord(double xi, double yi, double zi, double ti) {
//   int l(0);
//   int r(k-1);
//   int m;
//   while (l<r) {
//     m = (l+r)/2;
//     if (interpAlt(xi, yi, z[m], ti)>=zi) {
//       r = m;
//     }
//     else {
//       l = m+1;
//     }
//   }
//   return max(l-1,0);
// }


// int simClass::tCoordBis(double ti) {
//   for (int l(0); l<q; l++) {
//     if (t[l]>ti) {
//       if (l>0) {
//         return l-1;
//       }
//       else {
//         return l;
//       }
//     }
//   }
//   return q;
// }

// int simClass::tCoord(double ti) {
//   int l(0);
//   int r(q-1);
//   int m;
//   while (l<r) {
//     m = (l+r)/2;
//     if (t[m]>=ti) {
//       r = m;
//     }
//     else {
//       l = m+1;
//     }
//   }
//   return max(l-1,0);
// }

// void simClass::parseWind() {
//     File windFile = SD.open("windFile.txt", FILE_READ);  
//     if (windFile) {
//       if (DEBUG) { Serial.println("Wind File Opened"); }
//       if (windFile.available()) {
//         String memory = ""; 
//         char c;
//         for (unsigned a(0); a<(i*j*k*q); a++) {

//           if (DEBUG) { Serial.print("Line number: "); Serial.print(a); }

//           do {
//             c =  windFile.read();
//             if (c!= ',') { memory = memory+c; }
//           } while (c != ',');
          
//           // Time indexes from MATLAB are from 1 to 5 for a 4 hour models
//           // Time indexes in c++ code are from T=0h for the beginning of the file to T+4h for the end
//           // This is why we have to use memory.trim().toFloat()-1 instead of just memory.trim().toFloat().
//           t[a/(i*j*k)] = memory.trim().toFloat()-1;
//           if (DEBUG) { Serial.print(" "); Serial.print(memory); }
//           memory = "";

//           do {
//             c =  windFile.read();
//             if (c!= ',') { memory = memory+c; }
//           } while (c != ',');

//           x[(a/(j*k))%i] = memory.trim().toFloat();
//           if (DEBUG) { Serial.print(" "); Serial.print(memory); }
//           memory = "";

//           do {
//             c =  windFile.read();
//             if (c!= ',') { memory = memory+c; }
//           } while (c != ',');

//           y[(a/k)%j] = (memory.trim().toFloat()); 
//           if (DEBUG) { Serial.print(" "); Serial.print(memory); }
//           memory = "";

//           do {
//             c =  windFile.read();
//             if (c!= ',') { memory = memory+c; }
//           } while (c != ',');

//           z[a%k] = memory.trim().toFloat();
//           if (DEBUG) { Serial.print(" "); Serial.print(memory); }
//           memory = "";

//           do {
//             c =  windFile.read();
//             if (c!= ',') { memory = memory+c; }
//           } while (c != ',');

//           if (memory.trim() == "NaN") { memory = "0.0"; }
//           v[(a/(j*k))%i][(a/k)%j][a%k][a/(i*j*k)][0] = short((memory.trim().toFloat())*100.0);
//           if (DEBUG) { Serial.print(" "); Serial.print(memory); }
//           memory = "";

//           do {
//             c =  windFile.read();
//             if (c!= ',') { memory = memory+c; }
//           } while (c != ',');

//           if (memory.trim() == "NaN") { memory = "0.0"; }
//           v[(a/(j*k))%i][(a/k)%j][a%k][a/(i*j*k)][1] = short((memory.trim().toFloat())*100.0);
//           if (DEBUG) { Serial.print(" "); Serial.print(memory); } 
//           memory = "";

//           do {
//             c =  windFile.read();
//             if (c!= 10) { memory = memory+c; }
//           } while (c != 10);

//           if (memory.trim() == "NaN") { memory = "0.0"; }
//           v[(a/(j*k))%i][(a/k)%j][a%k][a/(i*j*k)][2] = short(constrain(memory.trim().toFloat(),0,32500)); 
//           if (DEBUG) { Serial.print(" "); Serial.print(memory); }
//           memory = "";
          
//           if (DEBUG) { Serial.println(" End of line"); }

//         }
//         if (DEBUG) { Serial.println("Going to the next part"); }

//         if (DEBUG) { Serial.print(" Metadatas : "); }

//         do {
//           c =  windFile.read();
//           if (c!= ',') { memory = memory+c; }
//         } while (c != ',');

//         metadata[0] = (memory.trim().toInt())%24;
//         if (DEBUG) { Serial.print(" "); Serial.print(memory); }
//         memory = "";
         
//         do {
//           c =  windFile.read();
//           if (c!= ',') { memory = memory+c; }
//         } while (c != ',');

//         metadata[1] = memory.trim().toInt(); 
//         if (DEBUG) { Serial.print(" "); Serial.println(memory); }
//         memory = "";
//       }
//       else {
//         if (DEBUG) { Serial.println("Wind File Empty"); }
//       }
//     }
//     else {
//       if (DEBUG) { Serial.println("Wind File Failed to Open"); }
//     }
//     if (DEBUG) { Serial.println("Wind Parsed Successfuly"); }

//     if (metadata[1]) {
//       // It means wind file has longitude data crossing 360°, for example, from 359 to 1°. 
//       for (unsigned a(0); a<j; a++) {
//         y[a] = y[a]-360;
//       }
//     }
//   }

//   int simClass::getHourModel() {
//     return metadata[0];
//   }

//   simStatus simClass::runCircleSimulation(double latIn, double lngIn, double altIn, double timeIn, double altFinalIn, double headingIn, double busrtAlt) {

//   lat = latIn;

//   lng = lngIn;
  
//   if (!metadata[1]) {
//     lng = -lng;
//     lng = lng+180;
//   }
  
//   alt = altIn;
//   time = timeIn;
//   altFinal = altFinalIn;

//   double timeStep = ((alt - altFinal) / 100.0);
//   unsigned timeCodeFile = (int(metadata[0]) % 24) * 3600.0;
//   timeStep = constrain(timeStep, 2, 300);

//   while (alt < busrtAlt) {
//     xyCoord wind;
//     unsigned timeDiffModel = time - timeCodeFile;
//     wind = computeWind(lat, lng, alt, (timeDiffModel) / 3600.0);
//     double U = 5 * timeStep;
//     enu2lla(wind.x * timeStep, wind.y * timeStep, U, lat, lng, alt);
//     time += timeStep;
//   }

//   while (alt > altFinal){
//     xyCoord wind, speed;
//     unsigned timeDiffModel = time - timeCodeFile;
//     wind = computeWind(lat, lng, alt, (timeDiffModel) / 3600.0);
//     speed.x = wind.x + trueAirspeed(SPEED_GLIDE_X,pressureSim(alt),temperatureSim(alt))*sin(heading/180.0*PI);
//     speed.y = wind.y + trueAirspeed(SPEED_GLIDE_X,pressureSim(alt),temperatureSim(alt))*cos(heading/180.0*PI);
//     double U = -trueAirspeed(SPEED_GLIDE_Z, pressureSim(alt), temperatureSim(alt)) * timeStep;
//     enu2lla(speed.x * timeStep, speed.y * timeStep, U, lat, lng, alt);
//     time += timeStep;
//   }

//   simStatus result;
//   result.lat = lat;
//   result.lng = lng;
//   return result;
// }