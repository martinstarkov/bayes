#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <math.h>
class PID {
  float Kp;
  float Ki;
  float Kd;
  float error;
  float differential_error;
  float derivative error;
  float integral_error;
  
public:
  PID(float new_Kp, float new_Ki, float new_Kd): Kp(new_Kp)
  
  float servoPID(float sensor_data, float Kp, float Ki, float Kd) {
    static float angle = 90, nominal_angle = 90, max_offset = 7, dt = 0.001; //In radians 7 is 0.122173
    static float error = 0.0f, old_error = 0.0f, error_differential = 0.0f, error_derivative = 0.0f, error_integral = 0.0f;

    //Measuring all error terms in angles
    old_error = error;
    error = 0.0f - sensor_data;
    error_differential = error - old_error;
    error_derivative = error_differential/dt;
    error_integral = error_integral + error*dt;

    //Control servo value
    angle = angle + Kp*error + Ki*(error_integral) + Kd*(error_derivative);
    //Limiting the servo values to the maximum angle of EDF.
    if(abs(angle) > nominal_angle + max_offset) {
        angle = 90 + nominal_angle + max_offset;
    }
    return angle;
  }
};

#endif
