#include "TELECOM.h"

static PacketAvUplink lastPacket;
static bool newCmdReceived = false;

comClass::comClass()
{ 
  lastPacket.cmdId = DEFAULT;
  lastPacket.cmdValue = INACTIVE;
}

void comClass::update() {
  while (LoRaDownlinkBuffer.available()) {
    LoRaCapsuleDownlink.decode(LoRaDownlinkBuffer.read());
  }
  while (LoRaUplinkBuffer.available()) {
    LoRaCapsuleUplink.decode(LoRaUplinkBuffer.read());
  }
}

void comClass::begin() {
  {
    SPI1.begin();
    SPI1.setMISO(LORA_DOWNLINK_MISO);
    SPI1.setMOSI(LORA_DOWNLINK_MOSI);
    SPI1.setSCK(LORA_DOWNLINK_SCK);

    LoRaDownlink.setPins(LORA_DOWNLINK_CS, LORA_DOWNLINK_RST, LORA_DOWNLINK_INT0);
    LoRaDownlink.setSPI(SPI1);
    
    if (!LoRaDownlink.begin(LORA_DOWNLINK_FREQ)) {
      if (DEBUG) {
        Serial.println("Starting LoRa Downlink failed!");
      }
    }
    else {
      if (DEBUG) {
        Serial.println("Starting LoRa Downlink success!");
      }
    }

    LoRaDownlink.setTxPower(LORA_DOWNLINK_POWER);
    LoRaDownlink.setSpreadingFactor(LORA_DOWNLINK_SF);
    LoRaDownlink.setSignalBandwidth(LORA_DOWNLINK_BW);
    LoRaDownlink.setCodingRate4(LORA_DOWNLINK_CR);
    LoRaDownlink.setPreambleLength(LORA_DOWNLINK_PREAMBLE_LEN);
  #if (LORA_DOWNLINK_CRC)
    LoRaDownlink.enableCrc();  // not necessary to work with miaou, even if miaou enbale it...:-|
  #else
    LoRaDownlink.disableCrc();
  #endif
  #if (LORA_DOWNLINK_INVERSE_IQ)
    LoRaDownlink.enableInvertIQ();
  #else
    LoRaDownlink.disableInvertIQ();
  #endif

    LoRaDownlink.receive();  
    //LoRaDownlink.onReceive(handleLoRaDownlink);
  }
  
  {
    SPI.begin(); 
    LoRaUplink.setPins(LORA_UPLINK_CS, LORA_UPLINK_RST, LORA_UPLINK_INT0);
    LoRaUplink.setSPI(SPI);
    
    if (!LoRaUplink.begin(LORA_UPLINK_FREQ)) {
      if (DEBUG) {
        Serial.println("Starting LoRa Uplink failed!");
      }
    }
    else {
      if (DEBUG) {
        Serial.println("Starting LoRa Uplink success!");
      }
    }

    
    LoRaUplink.setTxPower(LORA_UPLINK_POWER);
    LoRaUplink.setSpreadingFactor(LORA_UPLINK_SF);
    LoRaUplink.setSignalBandwidth(LORA_UPLINK_BW);
    LoRaUplink.setCodingRate4(LORA_UPLINK_CR);
    LoRaUplink.setPreambleLength(LORA_UPLINK_PREAMBLE_LEN);
  #if (LORA_UPLINK_CRC)
    LoRaUplink.enableCrc();  // not necessary to work with miaou, even if miaou enbale it...:-|
  #else
    LoRaUplink.disableCrc();
  #endif
  #if (LORA_UPLINK_INVERSE_IQ)
    LoRaUplink.enableInvertIQ();
  #else
    LoRaUplink.disableInvertIQ();
  #endif

    LoRaUplink.receive(); 
    LoRaUplink.onReceive(handleLoRaUplink); 
  }
}

comStatus comClass::get() {
  comStatus com;
  com.cmdId = cmdId;
  com.cmdValue = cmdValue;
  return com;
};

bool comClass::isUpdated() {
  if (newCmdReceived) {
    newCmdReceived = false;
    cmdId = (CMD_ID)lastPacket.cmdId;
    cmdValue = lastPacket.cmdValue;
    return true;
  }
  return false;
}

void handleLoRaCapsuleUplink(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  switch (packetId) {
    case CAPSULE_ID::GS_CMD:
      memcpy(&lastPacket, dataIn, len);
      newCmdReceived = true;
    break;
  }
}

// We never "receive" anything with the downlink radio.. we just send stuff, still the capsule 
// object needs its callback function to be initialised because Capsule is designed to be
// bidirectional so we just leave it empty.
void handleLoRaCapsuleDownlink(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  if (DEBUG) {
    Serial.println("Received packet on the downlink radio.. shouldn't happen");
  }
}


void handleLoRaDownlink(int packetSize) {
  for (int i = 0; i < packetSize; i++) {
    LoRaDownlinkBuffer.write(LoRaDownlink.read());
  }
}

void handleLoRaUplink(int packetSize) {
  for (int i = 0; i < packetSize; i++) {
    LoRaUplinkBuffer.write(LoRaUplink.read());
  }
}
