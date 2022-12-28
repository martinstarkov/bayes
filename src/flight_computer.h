#include <Servo.h>
#include "imu.h"
#include "pressure_sensor.h"
#include "filter.h"
#include "vector3.h"
#include "quaternion.h"
#include "controller.h"

class FlightComputer {
public:
    void init() {
        millisNew = millis();
        imu.init();
        pressure_sensor.Init();
    }
    Vector3 update() {
        static Vector3 raw_measuredAng;
        static Vector3 raw_angVelocity;
        static Quaternion raw_quaternion;
        static float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
        static Vector3 servoAngle;

        //Microcontroller time step calculation
        millisOld = millisNew;
        millisNew = millis();
        dt = millisNew - millisOld;
    
        //Raw sensor data retrieval
        raw_angVelocity = imu.getAngularVelocity();
        raw_angVelocity = raw_angVelocity.toRadians(raw_angVelocity);
        raw_quaternion = imu.getQuaternion();
        raw_measuredAng = raw_measuredAng.anglesToRadians(raw_quaternion);

        //Filtering of raw sensor data
        pitch = pitch_filter.kalmanFilterAngle(raw_measuredAng.x, raw_angVelocity.y, dt);
        roll = roll_filter.kalmanFilterAngle(raw_measuredAng.y, raw_angVelocity.x, dt);
        yaw = yaw_filter.kalmanFilterAngle(raw_measuredAng.z, raw_angVelocity.z, dt);
        
        //Determining servo values from PID controller
        servoAngle = anglePID.PID(pitch, roll, dt);
        return servoAngle;
        /*
        //Orientation resolution to prevent gimbal lock.
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
        // servo.write(U_y, U_z);
        */
    }
private:
    unsigned long millisOld, millisNew;
    static float dt;
    float Clamp(float value, float low, float high) {
        return value < low ? low : (value > high ? high : value);
    }
    // Servo servo1;
    IMU imu;
    PressureSensor pressure_sensor;
    Filter pitch_filter, roll_filter, yaw_filter;
    Controller anglePID;
    /*
    Vector3 integral;
    // Initial orientation quaternion of the hopper.
    Quaternion base{ 0.71f, 0.71f, 0.0f, 0.0f };
    // 0.122173f radians is 7 in degrees.
    float maximum_angle = 0.122173f; // unit: radians.
    */
};