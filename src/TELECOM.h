#ifndef COMMAND_H
#define COMMAND_H

#include "Arduino.h"
#include "CONFIG.h"
#include "Capsule.h"
#include "stdlib.h"
#include <LoopbackStream.h>
#include <LoRa.h>
// #include <functional>

static void handleLoRaUplink(int packetSize);
static void handleLoRaCapsuleUplink(uint8_t packetId, uint8_t *dataIn, uint32_t len); 

static void handleLoRaDownlink(int packetSize);
static void handleLoRaCapsuleDownlink(uint8_t packetId, uint8_t *dataIn, uint32_t len); 

static LoopbackStream LoRaDownlinkBuffer(1024);
static LoopbackStream LoRaUplinkBuffer(1024);

static CapsuleStatic LoRaCapsuleDownlink(handleLoRaCapsuleDownlink);
static CapsuleStatic LoRaCapsuleUplink(handleLoRaCapsuleUplink);

// So basically, the LoRaUplink HAS to be the global LoRa defined in LoRa.h,
// because we want to use interrupts when a packet from the GS is sent to the GSE,
// that means that everywhere in the code where we use LoRaUplink.something() we actually do 
// LoRa.something() because LoRaUplink is just a reference to LoRa.
static LoRaClass LoRaDownlink;
#define LoRaUplink LoRa

struct comStatus {
  CMD_ID cmdId;
  uint8_t cmdValue;
};

class comClass {
  public:
    comClass();
    comStatus get();
    void update();
    bool isUpdated();
    void begin();
    void sendTelemetry(uint8_t packetId, uint8_t *dataOut, uint32_t len);
  private:
    CMD_ID cmdId;
    uint8_t cmdValue;
    bool newCmd = false;
};



#endif
