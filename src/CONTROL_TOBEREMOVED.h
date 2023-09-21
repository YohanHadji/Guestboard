// #ifndef CONTROL
// #define CONTROL

// #include "math.h"
// #include "Arduino.h"
// #include "CONFIG.h"
// #include "PHY.h"

// struct conStatus {
//   double input;
//   double setpoint;
//   double output;
// };

// struct conTime {
//   unsigned int second;
//   unsigned int nanosecond;
// };

// class conClass {
//   public:
//     conClass();
//     conStatus get();
//     void set(conStatus);
//     void setInput(double);
//     void setPoint(double);
//     void setTime(unsigned int, unsigned int);
//     void compute();
//     void reset();
//     void isActive(bool mode);
//   private:
//     double timeDiff(conTime time1, conTime time2);
//     double input;
//     double setpoint;
//     double output;
//     double outputSum;
//     double lastError;
//     double lastInput;
//     conTime time;
//     conTime lastTime;
//     bool active;
// };

// #endif
