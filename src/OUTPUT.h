#ifndef OUTPUT_H
#define OUTPUT_H

#include "Arduino.h"
#include "CONFIG.h"

// Contains the informations send to the servo motor
struct outStatus {
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

class outClass {
  public:
    outClass();
    void write();
    outStatus get();
    outStatus compute(mixStatus);
  private:
};


#endif
