#include <Servo.h>
#include <utility/imumaths.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h> // Adafruit_BMP280

/* DEFINES */
// For performing arithmetic.
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif
#ifndef HALF_PI
#define HALF_PI 1.5707963267948966192313216916398f
#endif
#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559f
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

float Clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

class Vector3 {
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    
    Vector3() = default;
	~Vector3() = default;
    Vector3(float x, float y, float z) : x{ x }, y{ y }, z{ z } {}
	
    /*=
	operator String() const {
		return "[" + String(x) + "x," + String(y) + "y," + String(z) + "z]";
	}
	*/

    Vector3 ToRadians() {
        return { x * DEG_TO_RAD, y * DEG_TO_RAD, z * DEG_TO_RAD };
    }
    Vector3 ToDegrees() {
        return { x * RAD_TO_DEG, y * RAD_TO_DEG, z * RAD_TO_DEG };
    }
};

class Quaternion {
public:
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    
    Quaternion() = default;
	~Quaternion() = default;
    Quaternion(float w, float x, float y, float z) : w{ w }, x{ x }, y{ y }, z{ z } {}
	
    /*
	operator String() const {
		return "[" + String(w) + "," + String(x) + "i," + String(y) + "j," + String(z) + "k]";
	}
	*/
    Quaternion& operator+=(const Quaternion& rhs) {
        w += rhs.w;
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
    Quaternion& operator-=(const Quaternion& rhs) {
        w -= rhs.w;
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    Quaternion operator*(float f) {
		return { w * f, x * f, y * f, z * f };
	}
	Quaternion operator/(float f) {
		return { w / f, x / f, y / f, z / f };
	}	
    Quaternion operator*(const Quaternion& q) {
        Quaternion quaternion;
        quaternion.w = w * q.w - x * q.x - y * q.y - z * q.z;
        quaternion.x = w * q.x + x * q.w + y * q.z - z * q.y;
        quaternion.y = w * q.y - x * q.z + y * q.w + z * q.x;
        quaternion.z = w * q.z + x * q.y - y * q.x + z * q.w;
        return quaternion;
    }
	
    operator Vector3() {
        Vector3 vector;
        // Roll (x-axis rotation).
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        vector.x = atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation).
        float sinp = 2.0f * (w * y - z * x);
        if (abs(sinp) >= 1.0f) {
            vector.y = copysign(HALF_PI, sinp); // use 90 degrees if out of range
        } else {
            vector.y = asin(sinp);
        }
        // Yaw (z-axis rotation).
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        vector.z = atan2(siny_cosp, cosy_cosp);
        
        return vector;
	}
    
    float Magnitude() {
		return sqrtf(w * w + x * x + y * y + z * z);
	}
    Quaternion Normalized() {
		return *this / Magnitude();
	}
    // w is vector with respect to which the quaternion is differentiated.
    Quaternion Differentiated(const Vector3& w) {
        Quaternion quaternion{ 0, w.x, w.y, w.z };
        return quaternion * (*this) / 2.0f;
    }
};

// TODO: Make filter actually return something.
// Currently filter acts as a empty pass-through container.
class Filter {
public:
    Filter() = default;
    Filter(float value) : value{ value }, filtered_value{ value } {}
    void UpdateValue(float value) {
        this->value = value;
        // TODO: Replace this.
        this->filtered_value = this->value;
    }
    float GetValue() {
        return filtered_value;
    }
private:
    float value = 0.0f;
    float filtered_value = 0.0f;
    float integral = 0.0f;
};

class Controller {
public:
    static float LQR(float angular_position, float angular_velocity) {
        float U_pos = -TVC_POSITION_GAIN * angular_position;
        float U_vel = -TVC_VELOCITY_GAIN * angular_velocity;
        return U_pos + U_vel;
    }
private:
    static constexpr float INTEGRAL_GAIN = 2.0f;
    static constexpr float TVC_POSITION_GAIN = 0.7278f;
    static constexpr float TVC_VELOCITY_GAIN = 0.3640f;
};

class PressureSensor {
public:
    void Init(uint8_t address) {
        bool status = bmp.begin(address);
        if (!status) {
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring"));
            Serial.print("SensorID was: 0x"); 
            Serial.println(bmp.sensorID(), 16);
            while (1);
        }

        // TODO: Check if this is necessary. What does it even do?
        // Default settings from datasheet.
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }
    float GetTemperature() {
        return bmp.readTemperature();
    }
    float GetPressure() {
        return bmp.readPressure();
    }
    float GetAltitude(float sea_level_hPa = 1013.25) {
        return bmp.readAltitude(sea_level_hPa);
    }
private:
    Adafruit_BMP280 bmp;
};

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

class FlightComputer {
public:
    void Init() {
        pressure_sensor.Init();
        imu.Init();
    }
    void Update() {
        // Microcontroller time step calculation.
        // TODO: Change this to compute time from last update.
        float dt = 0.0001f;
    
        // Raw sensor data retrieval.
        float raw_ang_pos = imu.GetAngularPosition().ToRadians();
        float raw_ang_vel = imu.GetAngularVelocity().ToRadians();
        
        // Filtering of raw sensor data.
        y_filter.UpdateValue(raw_ang_vel.y);
        z_filter.UpdateValue(raw_ang_vel.z);
        Vector3 processed_ang_vel{ raw_ang_vel.x, y_filter.GetValue(), z_filter.GetValue() };
        
        // Orientation resolution to prevent gimbal lock.
        Quaternion q_dot = base.Differentiated(processed_ang_vel);
        base += q_dot * dt;
        // TODO: Research, this normalization may not be required.
        base = base.Normalized();
        Vector3 ang_pos = Vector3(base).ToDegrees();
    
        // Integrated control gains.
        integral.y += Controller::INTEGRAL_GAIN * ang_pos.y * dt;
        integral.z += Controller::INTEGRAL_GAIN * ang_pos.z * dt;
        
        // LQR.
        float control_y = Controller::LQR(ang_pos.y, processed_ang_vel.y);
        float control_z = Controller::LQR(ang_pos.z, processed_ang_vel.z);
        control_y += integral.y;
        control_z += integral.z;
        // Ensure control angle is never above or below the maximum controllability.
        control_y = Clamp(control_y, -maximum_angle, maximum_angle); 
        control_z = Clamp(control_z, -maximum_angle, maximum_angle);

        // Update servos with latest values.
        // TODO: Make this write the calculated control angles:
        // Servos.write(U_y, U_z)
    }
private:
    PressureSensor pressure_sensor;
    IMU imu;
    Vector3 integral;
    Filter y_filter;
    Filter z_filter;
    // Initial orientation quaternion of the hopper.
    Quaternion base{ 0.71f, 0.71f, 0.0f, 0.0f };
    // 0.122173f radians is 7 in degrees.
    float maximum_angle = 0.122173f; // unit: radians.
};

FlightComputer fc;

void setup() {
    Serial.begin(9600);
    fc.Init();
}

void loop() {
    fc.Update();
}