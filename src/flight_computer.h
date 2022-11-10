// TODO: Add header include guards.

#include <Servo.h>
#include "quaternion.h"
#include "pressure_sensor.h"
#include "imu.h"
#include "filter.h"
#include "controller.h"

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
        // servo.write(U_y, U_z);
    }
private:
    float Clamp(float value, float low, float high) {
        return value < low ? low : (value > high ? high : value);
    }
    // Servo servo1;
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