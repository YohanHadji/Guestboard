#ifndef COMMAND_H
#define COMMAND_H

#include "Arduino.h"
#include "CONFIG.h"
#include "Capsule.h"
#include "stdlib.h"
// #include <functional>

enum TLM_CMD {
  NO_CMD = 0,
  SEPARATE_CMD,
  DEPLOY_CMD,
  RECOVER_CMD,
  SPIRAL_CMD,
  ABORT_CMD,
  UNABORT_CMD,
  PING_CMD,
  MANUAL_CMD,
  AUTO_CMD,
  MANUAL_LEFT_CMD,
  MANUAL_RIGHT_CMD
};

struct cmdStatus {
  TLM_CMD tlmCmd;
};

class cmdClass {
  public:
    cmdClass();
    cmdStatus get();
    void update();
    bool isUpdated();
    void begin();
    Capsule<cmdClass> telemetryRadio;
    void handleTlmCmd(uint8_t , uint8_t*, uint32_t);
  private:
    TLM_CMD tlmCmd;
    String buff = "";
    bool newCmd = false;
};



#endif
