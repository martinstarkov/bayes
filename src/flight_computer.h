#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include <Servo.h>
#include "imu.h"
#include "filter.h"
#include "controller.h"

Servo outerServo, innerServo;   //outer servo = pitch, inner servo = roll

class FlightComputer {
    private:
    IMU bayesIMU;
    Filter pitchKFilter, rollKFilter;
    Controller pitchPID, rollPID;
    Quaternion orientation;
    Vector3 angle, angular_vel;
    float roll, pitch, dt;
    float roll_servo, pitch_servo;
    unsigned long millisNew, millisOld;

    public:
        void init() {
            Serial.begin(115200);
            bayes_imu.init();
            millisNew = millis();
            bayes_imu.getCalibration();
            innerServo.attach(11);
            outerServo.attach(10);
        }
        void update() {
            //Microcontroller time step calculation
            millisOld = millisNew;
            millisNew = millis();
            dt = (millisNew - millisOld)/1000;
            
            //Finding angles data
            orientation = bayesIMU.getQuaternion();
            angle = orientation.toDegrees();
            angular_vel = bayesIMU.getAngularVelocity();
            
            roll = rollKFilter.filterAngle(angle.x, angular_vel.x, dt);
            pitch = pitchKFilter.filterAngle(angle.y, angular_vel.y, dt);
            roll_servo = rollPID.servoPID(roll, 1.6f, 0.75f, 1.38f, 0.00005f);
            pitch_servo = pitchPID.servoPID(pitch, 1.8f, 0.75f, 1.38f, 0.00005f);
            innerServo.write(roll_servo);
            outerServo.write(pitch_servo);
            delay(20);
            Serial.print(roll);
            Serial.print(", ");
            Serial.print(pitch);
            Serial.print(", ");
            Serial.print(roll_servo);
            Serial.print(", ");
            Serial.println(pitch_servo);
        }
};

#endif