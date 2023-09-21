# include "SERVO.h"

// Transform the data in a way the servo motor can compute them
serStatus serClass::compute(mixStatus mixIn) {

  if (mixIn.dir < -TRIG) {
    l = map(mixIn.dir, -100, -TRIG, STEERING_MAX, STEERING_MIN-OFFSET);
    r = STEERING_MIN;
  }
  if (mixIn.dir > TRIG) {
    r = map(mixIn.dir, TRIG, 100, STEERING_MIN-OFFSET, STEERING_MAX);
    l = STEERING_MIN;
  }
  if (mixIn.dir < TRIG && mixIn.dir > -TRIG) {
    l = STEERING_MIN;
    r = STEERING_MIN;
  }

  l = l + (map(mixIn.brk, 0, 100, STEERING_MIN, STEERING_MAX)-STEERING_MIN);
  r = r + (map(mixIn.brk, 0, 100, STEERING_MIN, STEERING_MAX)-STEERING_MIN);

  int trimSign = (STEERING_MAX-STEERING_MIN)/abs(STEERING_MAX-STEERING_MIN);

  if (STEERING_MIN>STEERING_CUT) {
    l = constrain(l, STEERING_CUT, STEERING_MIN);
    r = constrain(r, STEERING_CUT, STEERING_MIN);
  }
  else {
    l = constrain(l, STEERING_MIN, STEERING_CUT);
    r = constrain(r, STEERING_MIN, STEERING_CUT);
  }

  l = l + trimSign*abs(min(dirTrim,0));
  r = r + trimSign*abs(max(dirTrim,0));

  if (STEERING_MIN>STEERING_MAX) {
    l = constrain(l, STEERING_MAX, STEERING_MIN);
    r = constrain(r, STEERING_MAX, STEERING_MIN);
  }
  else {
    l = constrain(l, STEERING_MIN, STEERING_MAX);
    r = constrain(r, STEERING_MIN, STEERING_MAX);
  }

  x1 = map(mixIn.dep, 0, 100, AUX1_MIN, AUX1_MAX);
  x2 = map(mixIn.acc, 0, 100, AUX2_MIN, AUX2_MAX);

  serStatus serOut;
  serOut.l = l;
  serOut.r = r;
  serOut.x1 = x1;
  serOut.x2 = x2;

  /*
  Serial.print("Servo output should be : ");
  Serial.print(l);
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print(x1);
  Serial.print(" ");
  Serial.println(x2);
  */

  return serOut;
}

void serClass::setDirTrim(int trim) {
  if (trim>TRIM_MIN && trim<TRIM_MAX) {
    dirTrim = trim;
  }
}

// Send the informations to the servo motor
void serClass::write() {

  servoL.write(map(l, STEERING_MIN, STEERING_MAX, 0, 180));
  servoR.write(map(r, STEERING_MIN, STEERING_MAX, 0, 180));
  servoX1.write(map(x1, AUX1_MIN, AUX1_MAX, 0, 180));
  servoX2.write(map(x2, AUX2_MIN, AUX2_MAX, 0, 180));
}

serClass::serClass() 
: l(STEERING_MIN), r(STEERING_MIN), x1(AUX1_MAX), x2(AUX2_MIN)
{
  servoL.attach(LEFT, STEERING_MIN, STEERING_MAX);
  servoR.attach(RIGHT, STEERING_MIN, STEERING_MAX);
  servoX1.attach(X1, AUX1_MIN, AUX1_MAX);
  servoX2.attach(X2, AUX2_MIN, AUX2_MAX);
  dirTrim = 0;
}

serStatus serClass::get() {
  serStatus serOut;
  serOut.l = l;
  serOut.r = r;
  serOut.x1 = x1;
  serOut.x2 = x2;
  return serOut;
}
