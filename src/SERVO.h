#ifndef SERVO
#define SERVO

#include "Arduino.h"
#include "CONFIG.h"
#include "PWMServo.h"

// Contains the informations send to the servo motor
struct serStatus {
  int l;
  int r;
  int x1;
  int x2;
};

struct mixStatus {
  double dir;
  double brk;
  double acc;
  double dep;
};

class serClass {
  public:
    serClass();
    void write();
    serStatus get();
    serStatus compute(mixStatus);
    void setDirTrim(int);
  private:
    int l;
    int r;
    int x1;
    int x2;
    PWMServo servoL;
    PWMServo servoR;
    PWMServo servoX1;
    PWMServo servoX2;
    int dirTrim;
};


#endif
