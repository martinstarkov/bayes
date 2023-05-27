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
        static int counter = 0;
        static int angle = 0;
        servo.write(0);
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
    
    void testROM() {
        static int counter = 0;
        static int innerAngle = 0;
        static int outerAngle = 0;
        innerServo.write(0);
        outerServo.write(90);
        while(counter < rev_duration) {
            innerAngle = sequence1(counter, rom_duration);
            outerAangle = sequence2(counter, rom_duration);
            innerServo.write(innerAngle);
            outerServo.write(outerAngle);
            counter++;
            delay(1);
        }
        init();
    }
    
    void innerROM() {
        test_ROM(innerServo);
    }
    
    void outerROM() {
        test_ROM(outerServo);
    }
};

#endif
