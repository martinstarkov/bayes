#ifndef INITIAL_H
#define INITIAL_H

#include <Servo.h>

Servo outerServo, innerServo;   //outer servo = pitch, inner servo = roll

class StartSequence {
public:
    void init() {
        initServos();
        delay(2000);    // The calm before the storm    
        
        // Check outer servo's range of motion
        checkRangeOfMotion(outerServo, &StartSequence::romSequence2, 1250, 1250);
        
        // Check inner servo's range of motion
        checkRangeOfMotion(innerServo, &StartSequence::romSequence2, 1000, 1000);
        
        // Move both servos to starting position and rotate 360 degrees
        moveBothServos(&StartSequence::romSequence1, &StartSequence::romSequence2, 5000, 1000);
    }

private:
    // Initialize the servos
    void initServos() {
        outerServo.attach(10);      //Servo1 pins   
        innerServo.attach(11);      //Servo2 pins
    }

    // Check the range of motion of a servo
    void checkRangeOfMotion(Servo& servo, float (StartSequence::*romSequence)(int, int), int tMax, int T) {
        int t = 0;
        servo.write(0);
        while (t <= tMax) {
            servo.write((this->*romSequence)(t, T));
            t += 1;
            delay(1);
        }
        servo.write(90);
    }

    // Move both servos to starting position and rotate 360 degrees
    void moveBothServos(float (StartSequence::*romSequence1)(int, int), float (StartSequence::*romSequence2)(int, int), int tMax, int T) {
        int t = 0;
        outerServo.write(90);
        innerServo.write(0);
        while (t < tMax) {
            innerServo.write((this->*romSequence1)(t, T));
            outerServo.write((this->*romSequence2)(t, T));
            t += 1;
            delay(1);
        }
        outerServo.write(90);
        innerServo.write(90);
    }

    // Individual servo range of motion check
    float romSequence1(int t, int T) {
        return 90*(1-cos((2*PI*t)/T));
    }

    // Both servo's range of motion check - 360 rotation
    float romSequence2(int t, int T) {
        return 90*(1+sin((2*PI*t)/T));
    }
};

#endif