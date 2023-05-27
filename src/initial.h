#ifndef INITIAL_H
#define INITIAL_H

#include <Servo.h>

Servo outerServo, innerServo;

class Gimble {
private:
    int rom_duration = 1000;
    int rev_duration = 5000;
    outerServo.attach(10);  
    innerServo.attach(11);
    
    float sequence1(int t, int T) {
        return 90*(1-cos((2*PI*t)/T)); 
    }

    float sequence2(int t, int T) {
        return 90*(1+sin((2*PI*t)/T));
    }
    
    void test_ROM(Servo& servo) {
        int static counter = 0;
        int static angle = 0;
        while (counter < rom_duration) {
            angle = sequence1(counter, rom_duration);
            servo.write(angle);
            counter++;
            delay(1);
        }
        init();
    }
    
public:
    void init() {
        innerServo.write(90);
        outerServo.write(90);
    }
    
    void innerROM() {
        test_ROM(innerServo);
    }
    
    void outerROM() {
        test_ROM(outerServo);
    }
    
    void testROM() {
        
    }
    
    void init() {
        delay(2000);
        
        // Check outer servo's range of motion
        checkRangeOfMotion(outerServo, &StartSequence::romSequence2, 1250, 1250);
        
        // Check inner servo's range of motion
        checkRangeOfMotion(innerServo, &StartSequence::romSequence2, 1000, 1000);
        
        // Move both servos to starting position and rotate 360 degrees
        moveBothServos(&StartSequence::romSequence1, &StartSequence::romSequence2, 5000, 1000);
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


};

#endif
