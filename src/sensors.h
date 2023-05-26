#ifndef SENSOR_H
#define SENSOR_H

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include "quaternion.h"

// Inertial measurement unit (IMU)
class IMU {
    Adafruit_BNO055 imu = Adafruit_BNO055();
    bool calibration_state;

public:
    IMU(bool state) : calibration_state(state) {}
    ~IMU() {}
    
    void init() {
        imu.begin();        //make sure serial.begin() is set to 115200 before this point.
        delay(1000);
        imu.setExtCrystalUse(true);       //the crystal produces clock signals, so we are using the arduino clk instead of the one in BNO055
    }

    void getCalibration() {
        static int counter = 0;
        int system, gyros, accel, mg = 0;

        imu.getCalibration(&system, &gyros, &accel, &mg);
        Serial.println("Calibrating...");
        while ((gyros != 3) or (accel != 3)) {
            counter += 1;
            imu.getCalibration(&system, &gyros, &accel, &mg);
            delay(1000);
            Serial.print(counter);
            Serial.print(" -> ");
            if (counter == 10) {
                counter = 0;
                Serial.println("Gyro calibration: ");
                Serial.print(gyros);
                Serial.print(" of 3, Accelerometer calibration: ");
                Serial.print(accel);
                Serial.println(" of 3");
            }
        }
        Serial.println("Calibrated!");
    }
  
    Vector3 getOrientation() {
        imu::Vector<3> ori = imu.getVector(Adafruit_BNO055::VECTOR_EULER);
        return Vector3(ori.x(), ori.y(), ori.z(););
    }

    Vector3 getAcceleration() {
        imu::Vector<3> acc = imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        return Vector3(acc.x(), acc.y(), acc.z(););
    }
    
    Vector3 getAngularVelocity() {
        imu::Vector<3> gyro = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        return Vector3(gyro.x(), gyro.y(), gyro.z());
    }
    
    Vector3 getMagnetometer() {
        imu::Vector<3> mag = imu.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
        return Vector3(mag.x(), mag.y(), mag.z());
    }
    
    Quaternion getQuaternion() {
        imu::Quaternion quat = imu.getQuat();
        return Quaternion(quat.w(), quat.x(), quat.y(), quat.z());
    }

    // Dat aquired as 1) tempreature 2) pressure reading 3) alltitude
    Vector3 getBMPReading() {
        return Vector3(bayesBMP.readTemperature(), bayesBMP.readPressure(), bayesBMP.readAltitude(1013.25););
    }
};

class BMP {
    Adafruit_BMP280 bmp;
    
    public:
    void bmpInit(uint8_t address) {
        if (!bmp.begin(address)) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring"));
            while (1);
        }
    }
};
#endif
