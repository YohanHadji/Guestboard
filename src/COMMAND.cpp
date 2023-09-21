#include "COMMAND.h"

cmdClass::cmdClass():telemetryRadio(&cmdClass::handleTlmCmd, this)
{ 
}

void cmdClass::update() {
  // while(TLM_PORT.available()) {
    // telemetryRadio.decode(TLM_PORT.read());
  // }
}

void cmdClass::begin() {
}

cmdStatus cmdClass::get() {
  cmdStatus cmd;
  cmd.tlmCmd = tlmCmd;
  return cmd;
};

bool cmdClass::isUpdated() {
  if (newCmd) {
    newCmd = false;
    return true;
  }
  return false;
}

void cmdClass::handleTlmCmd(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case 0x00:
      // Packet type 0...
      {
        String buff = "";
        for (unsigned i(0); i<len; i++) {
          buff += (char)dataIn[i];
        }

        if (buff == "separate") {
          tlmCmd = SEPARATE_CMD;
        }
        else if (buff == "deploy") {
          tlmCmd = DEPLOY_CMD;
        }
        else if (buff == "ping") {
          tlmCmd = PING_CMD;
        }
        else {
          tlmCmd = NO_CMD;
        }
        newCmd = true;
        buff = "";
      }
    break;
    case 0x01:
      // Packet type 1...
    break;
    default:
    break;
  }
}

