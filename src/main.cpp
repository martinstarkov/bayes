#include "vector3.h"
#include "flight_computer.h"

    Servo pitchServo, rollServo;
    FlightComputer fc;
    Vector3 servoAngle;
    //Creating an IMU object.
    Adafruit_BNO055 bayesIMU = Adafruit_BNO055();

void setup() {
    Serial.begin(115200);
    fc.init();

    pitchServo.attach();
    rollServo.attach();
    rollServo.write(90);
    delay(20);
    rollServo.write(90);
    delay(20);
}

void loop() {
    servoAngle = fc.update();
    pitchServo.write(servoAngle.x);
    delay(20);
    rollServo.write(servoAngle.y);
    delay(20);
}