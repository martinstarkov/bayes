#ifndef FLIGHTCOMPUTER_H
#define FLIGHTCOMPUTER_H

#include "initial.h"
#include "sensor.h"
#include "filter.h"
#include "controller.h"

Servo outerServo, innerServo;   //outer servo = pitch, inner servo = roll

class FlightComputer {
    outerServo.attach(10);      //Servo1 pins   
    innerServo.attach(11);      //Servo2 pins
    
    IMU bayesIMU(false);
    PID pitchPID(0.1, 0.01, 0.01, 7);
    PID rollPID(0.1, 0.01, 0.01, 7);
    Gimble gimble;
    Filter pitchKFilter;
    Filter rollKFilter;
    Quaternion orientation;
    Vector3 angle, ang_vel;
    float roll, pitch;
    float roll_servo, pitch_servo;
    unsigned long millisNew, millisOld;
    float dt;
    
    void print() {
        Serial.print("Filtered Sensor Data: roll = ");
        Serial.print(roll);
        Serial.print(", pitch = ");
        Serial.print(pitch);
        Serial.println(".");
        Serial.print("PID Output: roll = ");
        Serial.print(roll_servo);
        Serial.print(", pitch = ");
        Serial.println(pitch_servo);
        Serial.println(".");
    }

    public:
        void init() {
            Serial.begin(115200); 
            bayesIMU.init();
            bayesIMU.calibrate();
            gimble.testROM();
            millisNew = millis();
        }

        void loop() {
            millisOld = millisNew;
            millisNew = millis();
            dt = (millisNew - millisOld)/1000;
            
            orientation = bayesIMU.getQuaternion();
            angle = orientation.toDegrees();
            ang_vel = bayesIMU.getAngularVelocity();
            
            roll = rollKFilter.filterAngle(angle.x, angular_vel.x, dt);
            pitch = pitchKFilter.filterAngle(angle.y, angular_vel.y, dt);
            roll_servo = 90 + rollPID.controller(roll);
            pitch_servo = 90 + pitchPID.controller(pitch)
            innerServo.write(roll_servo);
            outerServo.write(pitch_servo);
            delay(20);

        }
};

#endif
