#ifndef IMU_H
#define IMU_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include "quaternion.h"

// Inertial measurement unit (IMU)
class IMU {
private:
  Adafruit_BNO055 bayesIMU = Adafruit_BNO055();
public:
  void init() {
    bayesIMU.begin();        //make sure serial.begin() is set to 115200 before this point.
    delay(1000);
    bayesIMU.setExtCrystalUse(true);       //the crystal produces clock signals, so we are using the arduino clk instead of the one in BNO055
  }

  void getCalibration() {
    static int counter = 0;
    uint8_t system, gyros, accel, mg = 0;
    bayesIMU.getCalibration(&system, &gyros, &accel, &mg);
    Serial.println("Calibrating...");
    while ((gyros != 3) or (accel != 3)) {
      counter += 1;
      bayesIMU.getCalibration(&system, &gyros, &accel, &mg);
      delay(1000);
      if (counter == 10) {
        counter = 0;
        Serial.print("Gyro calibration: ");
        Serial.print(gyros);
        Serial.print(" of 3, Accelerometer calibration: ");
        Serial.print(accel);
        Serial.println(" of 3");
      }
    }
    Serial.println("Calibrated!");
  }
  
  Vector3 getOrientation() {
    Vector3 ori_vector;         //creates a vector3 variable to return
    imu::Vector<3> ori = bayesIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
    ori_vector.x = ori.x(), ori_vector.y = ori.y(), ori_vector.z = ori.z();
    return ori_vector;
  }

  Vector3 getAcceleration() {
    Vector3 acc_vector;         //creates a vector3 variable to return
    imu::Vector<3> acc = bayesIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    acc_vector.x = acc.x(), acc_vector.y = acc.y(), acc_vector.z = acc.z();
    return acc_vector;
  }
  
  Vector3 getAngularVelocity() {
    Vector3 gyro_vector;        //creates a vector3 variable to return
    imu::Vector<3> gyro = bayesIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gyro_vector.x = gyro.x(), gyro_vector.y = gyro.y(), gyro_vector.z = gyro.z();
    return gyro_vector;
  }
  
  Vector3 getMagnetometer() {
    Vector3 mag_vector;        //creates a vector3 variable to return
    imu::Vector<3> mag = bayesIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    mag_vector.x = mag.x(), mag_vector.y = mag.y(), mag_vector.z = mag.z();
    return mag_vector;
  }
  Quaternion getQuaternion() {
    Quaternion quaternion;     //creates a quaternion variable to return
    imu::Quaternion quat = bayesIMU.getQuat();
    quaternion.w = quat.w(), quaternion.x = quat.x(), quaternion.y = quat.y(), quaternion.z = quat.z();
    return quaternion;
  }
};

#endif