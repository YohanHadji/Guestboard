#include "COMMAND.h"

cmdClass::cmdClass():telemetryRadio(&cmdClass::handleTlmCmd, this),rx(&RX_PORT) 
{ 
}

void cmdClass::update() {
  rx.Read();
  for (unsigned i(0); i<7; i++) {
    ch[i] = rx.rx_channels()[i];
  }
  while(TLM_PORT.available()) {
    telemetryRadio.decode(TLM_PORT.read());
  }
}


void cmdClass::begin() {
  rx.Begin();
  TLM_PORT.begin(TLM_BAUD);
}


cmdStatus cmdClass::get() {
  cmdStatus cmd;
  for (unsigned i(0); i<7; i++) {
    cmd.ch[i] = ch[i];
  }
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
        else if (buff == "recover") {
          tlmCmd = RECOVER_CMD;
        }
        else if (buff == "spiral") {
          tlmCmd = SPIRAL_CMD;
        }
        else if (buff == "abort") {
          tlmCmd = ABORT_CMD;
        }
        else if (buff == "unabort") {
          tlmCmd = UNABORT_CMD;
        }
        else if (buff == "ping") {
          tlmCmd = PING_CMD;
          //TLM_PORT.println(" ..pong");
        }
        else if (buff == "manual") {
          tlmCmd = MANUAL_CMD;
        }
        else if (buff == "auto") {
          tlmCmd = AUTO_CMD; 
        }
        else if (buff == "left") {
          tlmCmd = MANUAL_LEFT_CMD;
        }
        else if (buff == "right") {
          tlmCmd = MANUAL_RIGHT_CMD;
        }
        else {
          tlmCmd = NO_CMD;
        }
        newCmd = true;
        //TLM_PORT.println(buff);
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

