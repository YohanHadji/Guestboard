#include "../ERT_RF_Protocol_Interface/PacketDefinition.h"
#include "../ERT_RF_Protocol_Interface/ParameterDefinition.h"

//---------- CONFIG ----------//

#define DEBUG           false

#define TLM_MONITOR     false // Show telemetry on serial monitor 

#define TLM_MAIN        true  // Send telemetry on main telem port 
#define TLM_MAIN_RATE       0.25    // Rate of telemetry sent to radiosonde (in Hz)

#define LOW_RATE        true  // Additional low rate saving on a second file 
#define LOW_RATE_RATE   1.0     // Rate of low rate saving (in Hz)

#define DEP_MODE            1    // If 0 = Timer, 1 = Timer AND Altitude used to deploy. So will wait minimum X seconds and then will wait for good altitude.

// Deployment Timer(s) (in seconds)
#define DESCENT_TIMER       10.0  // Time before deploying the wing once in descent mode 

#define DEP_ALT_MODE    0     // If 1, altitudes are above ground (LANDING), if 0, altitudes are ASL
#define DEP_ALT         16000 // m 

#define SEP_ALT_MODE    0     // If 1, altitudes are above ground (LAUNCH), if 0, altitudes are ASL
#define SEP_ALT         24000 // m 

#define VUP             2.5     // m/s 
#define VDOWN           -2.5    // m/s

// PORT WIRING
#define GPS_MAIN_PORT Serial3
#define GPS_MAIN_BAUD 115200

#define GPS_AUX_PORT Serial3
#define GPS_AUX_BAUD 115200

#define PROPSENSOR_I2C_PORT Wire2
#define PROPSENSOR_I2C_ADDR 0x68

// MISC WIRING 
#define SOLENOID_1_PIN  1
#define SOLENOID_2_PIN  2
#define SOLENOID_3_PIN  3
#define SOLENOID_4_PIN  4
#define SERVO_1_PIN     5
#define SERVO_2_PIN     6
#define INGNITOR_PIN    7
#define BUZZER_PIN      8

#define BAT_PIN        A0

#define LED_A_PIN     3 
#define LED_B_PIN     4

// PROPULSION TIMING
#define IGNITER_COUNTER     1.0
#define IGNITION_COUNTER    1.0
#define THRUST_COUNTER      1.0
#define SHUTDOWN_COUNTER    1.0

// LOGIC LEVELS 
#define SOLENOID1_OPEN  HIGH
#define SOLENOID2_OPEN  HIGH
#define SOLENOID3_OPEN  HIGH
#define SOLENOID4_OPEN  HIGH
#define SERVO1_OPEN     HIGH
#define SERVO2_OPEN     HIGH
#define IGNITOR_ACTIVE  HIGH
#define BUZZER_ACTIVE   HIGH

// LORA WIRING & SETTINGS 
#define LORA_UPLINK_FREQ         UPLINK_FREQUENCY
#define LORA_UPLINK_POWER        UPLINK_POWER
#define LORA_UPLINK_BW           UPLINK_BW
#define LORA_UPLINK_SF           UPLINK_SF
#define LORA_UPLINK_CR           UPLINK_CR
#define LORA_UPLINK_PREAMBLE_LEN UPLINK_PREAMBLE_LEN
#define LORA_UPLINK_SYNC_WORD    UPLINK_SYNC_WORD
#define LORA_UPLINK_CRC          UPLINK_CRC
#define LORA_UPLINK_INVERSE_IQ   UPLINK_INVERSE_IQ

#define LORA_DOWNLINK_FREQ         AV_DOWNLINK_FREQUENCY
#define LORA_DOWNLINK_POWER        AV_DOWNLINK_POWER
#define LORA_DOWNLINK_BW           AV_DOWNLINK_BW
#define LORA_DOWNLINK_SF           AV_DOWNLINK_SF
#define LORA_DOWNLINK_CR           AV_DOWNLINK_CR
#define LORA_DOWNLINK_PREAMBLE_LEN AV_DOWNLINK_PREAMBLE_LEN
#define LORA_DOWNLINK_SYNC_WORD    AV_DOWNLINK_SYNC_WORD
#define LORA_DOWNLINK_CRC          AV_DOWNLINK_CRC
#define LORA_DOWNLINK_INVERSE_IQ   AV_DOWNLINK_INVERSE_IQ

// PIN/GPIO Definition
#define LORA_UPLINK_SCK                13
#define LORA_UPLINK_MOSI               11
#define LORA_UPLINK_MISO               12
#define LORA_UPLINK_CS                 10
#define LORA_UPLINK_INT0               22
//#define LORA_UPLINK_INT5             39
#define LORA_UPLINK_RST                36

#define LORA_DOWNLINK_SCK                27
#define LORA_DOWNLINK_MOSI               26
#define LORA_DOWNLINK_MISO               39
#define LORA_DOWNLINK_CS                 38
#define LORA_DOWNLINK_INT0               23
//#define LORA_UPLINK_INT5               39
#define LORA_DOWNLINK_RST                37

