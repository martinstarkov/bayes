// TODO: Add header include guards.
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "vector3.h"
#include "quaternion.h"

// Inertial measurement unit (IMU)
class IMU {
public:
    void init() {
        bayesIMU.begin();        //make sure serial.begin() is set to 115200 before this point.
        delay(1000);
        bayesIMU.setExtCrystalUse(true);       //the crystal produces clock signals, so we are using the arduino clk instead of the one in BNO055
    }

    Vector3 getOrientation() {
        Vector3 ori_vector;         //creates a vector3 variable to return
        imu::Vector<3> ori = bayesIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
        ori_vector.x = ori.x(), ori_vector.y = ori.y(), ori_vector.z = ori.z();
        return ori_vector;
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
        return gyro_vector;
    }
    
    Quaternion getQuaternion() {
        Quaternion quaternion;     //creates a quaternion variable to return
        imu::Quaternion quat = bayesIMU.getQuat();
        quaternion.w = quat.w(), quaternion.x = quat.x(), quaternion.y = quat.y(), quaternion.z = quat.z();
        return quaternion;
    }
};