#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include <Servo.h>
#include "imu.h"
#include "filter.h"
#include "controller.h"

Servo pitchServo, rollServo;

class FlightComputer {
public:
    void init() {
        Serial.begin(115200);
        bayes_imu.init();
        millisNew = millis();
        bayes_imu.getCalibration();
        rollServo.attach(11);
        pitchServo.attach(10);
    }
    void update() {
      //Microcontroller time step calculation
      millisOld = millisNew;
      millisNew = millis();
      dt = (millisNew - millisOld)/1000;
    
      //Finding angles data
      quat = bayes_imu.getQuaternion();
      angles = quat.anglesToDegrees(quat);
      gyro = bayes_imu.getAngularVelocity();
    
      roll = kal_filter1.kalmanFilterAngle(angles.x, gyro.x, dt);
      pitch = kal_filter2.kalmanFilterAngle(angles.y, gyro.y, dt);
      roll_servo = pid_cont1.PID(roll, 1.6f, 0.75f, 1.38f, 0.00005f);
      pitch_servo = pid_cont2.PID(pitch, 1.8f, 0.75f, 1.38f, 0.00005f);
      rollServo.write(roll_servo);
      pitchServo.write(pitch_servo);
      delay(20);
      Serial.print(roll);
      Serial.print(", ");
      Serial.print(pitch);
      Serial.print(", ");
      Serial.print(roll_servo);
      Serial.print(", ");
      Serial.println(pitch_servo);
    }
private:
    IMU bayes_imu;
    Filter kal_filter1, kal_filter2;
    Controller pid_cont1, pid_cont2;
    
    Quaternion quat;
    Vector3 gyro, angles;
    float roll, pitch, dt;
    float roll_servo, pitch_servo;
    unsigned long millisNew, millisOld;
};

#endif