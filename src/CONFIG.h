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

// MISC WIRING 
#define SOLENOID_1_PIN  1
#define SOLENOID_2_PIN  2
#define SOLENOID_3_PIN  3
#define SOLENOID_4_PIN  4
#define SERVO_1_PIN     5
#define SERVO_2_PIN     6
#define INGNITOR_PIN    7
#define BUZZER_PIN      8

#define LED_A_PIN     3 
#define LED_B_PIN     4

