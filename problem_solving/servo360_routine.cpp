#include <Servo.h>
#include <math.h>

Servo servoR;
Servo servoO;
int sPinR = 10;
int sPinO = 11;
int i = 3, t = 500;
void setup() {
  servoR.attach(sPinR);
  servoO.attach(sPinO);
}

void loop() {
  if(i == 3) {
    servoR.write(90);
    servoO.write(90);
    delay(1000);
    i = 0;
  }
  else if(i == 0) {
    servoR.write(180*sin((0.09*t)*3.14159265/180));
    servoO.write(180*sin((0.09*t)*3.14159265/180));
    i = t == 1000 ? (i == 0 ? 1 : 0) : i;
    t = t < 1000 ? t + 50 : 0;
  }
  else {
    servoR.write(180*cos((0.09*t)*3.14159265/180));
    servoO.write(180*cos((0.09*t)*3.14159265/180));
    i = t == 1000 ? (i == 0 ? 1 : 0) : i;
    t = t < 1000 ? t + 50 : 0;
  }
  delay(50);
}