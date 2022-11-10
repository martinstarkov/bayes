#include "flight_computer.h"

FlightComputer fc;

void setup() {
    Serial.begin(9600);
    fc.Init();
}

void loop() {
    fc.Update();
}