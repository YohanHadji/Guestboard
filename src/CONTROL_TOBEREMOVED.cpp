// #include "CONTROL.h"

// conClass::conClass()
//   : output(0), outputSum(0), lastError(0), lastTime({0,0}), active(false)
// {
// }

// void conClass::reset() {
//   outputSum = 0;
// }

// void conClass::isActive(bool mode) {
//   if (mode) {
//     active = true;
//   } else {
//     active = false;
//     reset();
//   }
// }

// void conClass::compute() {

//   double inputRaw = input;
//   input = -getAngle(input, setpoint);
//   setpoint = 0;

//   double timeChange = timeDiff(time, lastTime);
//   double SampleTimeInSec = ((double)PID_SAMPLE_TIME)/1000.0;

//   double kp = NKP;
//   double ki = NKI * SampleTimeInSec;
//   double kd = NKD / SampleTimeInSec;

//   if(timeChange>=(SampleTimeInSec*0.9) && active) {
//       /*Compute all the working error variables*/
//       double error = setpoint - input;
    
//       outputSum+= (ki * error);
//       constrain(outputSum, -45, 45);

//       if (PID_D_MODE == 0) {
//         double dError = (error - lastError);
//         output = kp * error + outputSum + kd * dError;
//         /*
//         Serial.print(kp*error);
//         Serial.print(",");
//         Serial.println(kd*dError);
//         */
//       }
//       else {
//         double dInput = (inputRaw - lastInput);
//         output = kp * error + outputSum - kd * dInput;
//         /*
//         Serial.print(kp*error);
//         Serial.print(",");
//         Serial.println(-kd*dInput);
//         */
//       }

//       /*Remember some variables for next time*/
//       lastError = error;
//       lastInput = inputRaw;
//       lastTime = time;
//   }
// }

// conStatus conClass::get() {
//   conStatus conOut;
//   conOut.input = input;
//   conOut.setpoint = setpoint;
//   conOut.output = output;
//   return conOut;
// }

// void conClass::set(conStatus conIn) {
//   input = conIn.input;
//   setpoint = conIn.setpoint;
//   output = conIn.output;
// }

// void conClass::setInput(double inputIn) {
//   input = inputIn;
// }

// void conClass::setPoint(double setpointIn) {
//   setpoint = setpointIn;
// }

// void conClass::setTime(unsigned int secondIn, unsigned int nanosecondIn) {
//   time.second = secondIn;
//   time.nanosecond = nanosecondIn;
// }

// double conClass::timeDiff(conTime time1, conTime time2) {
//     double timeDiff = (time1.second - time2.second) + ((time1.nanosecond - time2.nanosecond)/1000000000.0);
//     return timeDiff;
// }