#ifndef OUTPUT_H
#define OUTPUT_H

#include "Arduino.h"
#include "CONFIG.h"

// Contains the informations send to the servo motor
struct outStatus {
  bool solenoid1;
  bool solenoid2;
  bool solenoid3;
  bool solenoid4;
  bool servo1;
  bool servo2;
  bool ignitor;
  bool buzzer;
};

struct mixStatus {
  bool solenoid1;
  bool solenoid2;
  bool solenoid3;
  bool solenoid4;
  bool servo1;
  bool servo2;
  bool ignitor;
  bool buzzer;
};

class outClass {
  public:
    outClass();
    void write();
    outStatus get();
    outStatus compute(mixStatus);
  private:
};


#endif
