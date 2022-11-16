#include <math.h>
#include <Servo.h>

class Routine {
public:
  Servo servo1, servo2;
  int servo_Pin1 = 10, servo_Pin2 = 11;
  float servo1_Angle, servo2_Angle;

    void servo360(float servo1_Angle, float servo2_Angle) {
    static int counter;

    counter = (counter <= 360) ? counter + 1 : 0;
    (servo1_Angle) = sin(counter);
    (servo2_Angle) = cos(counter);
  }

    void gyroScope()
};

Routine servo_Module;

void setup() {
    Serial.begin(9600);
    servo_Module.servo1.attach(servo_Module.servo_Pin1);
    servo_Module.servo2.attach(servo_Module.servo_Pin2);
}

void loop() {
    servo_Module.servo360(servo_Module.servo1_Angle, servo_Module.servo2_Angle);
    servo_Module.servo1.write(servo_Module.servo1_Angle);
    servo_Module.servo2.write(servo_Module.servo2_Angle);
    delay(100);
}