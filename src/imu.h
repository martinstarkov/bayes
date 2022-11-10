// TODO: Add header include guards.

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Inertial measurement unit (IMU)
class IMU {
public:
    void Init(int32_t sensorID = -1, uint8_t address = 0x28, TwoWire *theWire = &Wire) {
        bno = Adafruit_BNO055(sensorID, address, theWire);
        if (!bno.begin()) {
            /* There was a problem detecting the BNO055 ... check your connections */
            Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
            while (1);
        }
    }
    Vector3 GetOrientation() {
        sensors_event_t e;
        bno.getEvent(&e, Adafruit_BNO055::VECTOR_EULER);
        if (e->type == SENSOR_TYPE_ORIENTATION) {
            return { e->orientation.x, e->orientation.y, e->orientation.z };
        }
        return {};
    }
    Vector3 GetAngularVelocity() {
        sensors_event_t e;
        bno.getEvent(&e, Adafruit_BNO055::VECTOR_GYROSCOPE);
        if (e->type == SENSOR_TYPE_GYROSCOPE) {
            return { e->gyro.x, e->gyro.y, e->gyro.z };
        }
        return {};
    }
    Vector3 GetLinearAcceleration() {
        sensors_event_t e;
        bno.getEvent(&e, Adafruit_BNO055::VECTOR_LINEARACCEL);
        if (e->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
            return { e->acceleration.x, e->acceleration.y, e->acceleration.z };
        }
        return {};
    }
    Vector3 GetGravityAcceleration() {
        sensors_event_t e;
        bno.getEvent(&e, Adafruit_BNO055::VECTOR_GRAVITY);
        if (e->type == SENSOR_TYPE_GRAVITY) {
            return { e->acceleration.x, e->acceleration.y, e->acceleration.z };
        }
        return {};
    }
    Vector3 GetAngularAcceleration() {
        sensors_event_t e;
        bno.getEvent(&e, Adafruit_BNO055::VECTOR_ACCELEROMETER);
        if (e->type == SENSOR_TYPE_ACCELEROMETER) {
            return { e->acceleration.x, e->acceleration.y, e->acceleration.z };
        }
        return {};
    }
private:
    Adafruit_BNO055 bno;
};