#include <Servo.h>

//Servo1 = 10
//Servo2 = 11
//Servo 3 = 12

int servoPin = 10

// Create a servo object 
Servo Servo1; 
void setup() { 
   // We need to attach the servo to the used pin number 
   Servo1.attach(servoPin); 
}
void loop(){ 
   // Make servo go to 0 degrees 
   Servo1.write(0); 
   delay(1000); 
   // Make servo go to 90 degrees 
   Servo1.write(90); 
   delay(1000); 
   // Make servo go to 180 degrees 
   Servo1.write(180); 
   delay(1000); 
}



// sertvio with input from BNO


#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
sensors_event_t event;

Servo Servo1;
Servo Servo2;
/* Initialise the sensor */


  void setup(void) {
    Serial.begin(115200);
    
    if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    
    Servo1.attach(10);
    Servo2.attach(11);
  }

  void loop() {
    bno.getEvent(&event);
    Servo1.write(event.orientation.y);
    Servo2.write(event.orientation.z);
  }
