#include "TELECOM.h"

static av_uplink_t lastPacket;
static bool newCmdReceived = false;

comClass::comClass()
{
  lastPacket.order_id = 0x00;
  lastPacket.order_value = 0x00;
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
    // LORA_DOWNLINK_PORT.begin();

    LORA_DOWNLINK_PORT.setMISO(LORA_DOWNLINK_MISO);
    LORA_DOWNLINK_PORT.setMOSI(LORA_DOWNLINK_MOSI);
    LORA_DOWNLINK_PORT.setSCK(LORA_DOWNLINK_SCK);

    LoRaDownlink.setPins(LORA_DOWNLINK_CS, LORA_DOWNLINK_RST, LORA_DOWNLINK_INT0);
    LoRaDownlink.setSPI(LORA_DOWNLINK_PORT);
    
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

    // LoRaDownlink.receive();  
    //LoRaDownlink.onReceive(handleLoRaDownlink);
  }
  
  {
    // LORA_UPLINK_PORT.begin(); 

    LORA_UPLINK_PORT.setMISO(LORA_UPLINK_MISO);
    LORA_UPLINK_PORT.setMOSI(LORA_UPLINK_MOSI);
    LORA_UPLINK_PORT.setSCK(LORA_UPLINK_SCK);

    LoRaUplink.setPins(LORA_UPLINK_CS, LORA_UPLINK_RST, LORA_UPLINK_INT0);
    LoRaUplink.setSPI(LORA_UPLINK_PORT);
    
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

void comClass::sendTelemetry(uint8_t packetId, uint8_t *data, uint32_t len) {

  av_downlink_t packetToSend;

  memcpy(&packetToSend, data, len);

  // Serial.print("Sending telemetry ");
  // Serial.print(packetToSend.timestamp);
  // Serial.print(" ");
  // Serial.print(packetToSend.engine_state.pressurize);
  // Serial.print(" ");
  // Serial.print(packetToSend.engine_state.servo_N2O);
  // Serial.print(" ");
  // Serial.print(packetToSend.engine_state.servo_fuel);
  // Serial.print(" ");
  // Serial.print(packetToSend.engine_state.vent_N2O);
  // Serial.print(" ");
  // Serial.println(packetToSend.engine_state.vent_fuel);

  Serial.println("Sending that we are in mode: " + String(packetToSend.av_state));

  uint8_t *codedBuffer = LoRaCapsuleDownlink.encode(packetId, data, len);
  size_t codedLen = LoRaCapsuleDownlink.getCodedLen(len);

  LoRaDownlink.beginPacket();
  LoRaDownlink.write(codedBuffer, codedLen);
  LoRaDownlink.endPacket(true);

  // for (int i = 0; i<codedLen; i++) {
  //   Serial.print((int)codedBuffer[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();

  delete[] codedBuffer;
}

bool comClass::isUpdated() {
  if (newCmdReceived) {
    newCmdReceived = false;
    cmdId = (CMD_ID)lastPacket.order_id;
    cmdValue = lastPacket.order_value;
    return true;
  }
  return false;
}

void comClass::resetCmd() {
  cmdId = (CMD_ID)(0);
  cmdValue = 0x00;
  lastPacket.order_id = 0x00;
  lastPacket.order_value = 0x00;
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

