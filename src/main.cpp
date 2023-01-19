#include "flight_computer.h"

FlightComputer fc;
//Vector3 servoAngle;

void setup() {
  fc.init();
}

void  loop() {
  fc.update();
}