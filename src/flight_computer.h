#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "initial.h"
#include "sensor.h"
#include "filter.h"
#include "controller.h"

Servo outerServo, innerServo;   //outer servo = pitch, inner servo = roll

class FlightComputer {
    private:
    outerServo.attach(10);      //Servo1 pins   
    innerServo.attach(11);      //Servo2 pins
    
    IMU bayesIMU(false);
    Gimble gimble;
    Filter pitchKFilter, rollKFilter;
    PID pitchPID, rollPID;
    Quaternion orientation;
    Vector3 angle, ang_vel;
    float roll, pitch, dt;
    float roll_servo, pitch_servo;
    unsigned long millisNew, millisOld;

    public:
        void init() {
            Serial.begin(115200); 
            bayesIMU.init();
            gimble.testROM();
            millisNew = millis();
            bayesIMU.calibrate();
        }

        void update() {
            //Microcontroller time step calculation
            millisOld = millisNew;
            millisNew = millis();
            dt = (millisNew - millisOld)/1000;
            
            //Finding angles data
            orientation = bayesSensor.getQuaternion();
            angle = orientation.toDegrees();
            angular_vel = bayesSensor.getAngularVelocity();
            
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
