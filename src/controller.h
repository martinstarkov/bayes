#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <math.h>
class Controller {
public:
  float PID(float m_angle, float servo_edf_ratio, float PROPORTIONAL_GAIN, float DERIVATIVE_GAIN, float INTEGRAL_GAIN) {
    static float ang_servo = 90, dt = 0.011;
    static float error = 0.0f, error_change = 0.0f, error_derivative = 0.0f, error_integral = 0.0f, old_error = 0.0f;
    float MAXIMUM_ANGLE = 7;   //In radians 0.122173

    //Measuring all error terms in angles
    old_error = error;
    error = 0.0 - m_angle;
    error_change = error - old_error;
    error_derivative = error_change/dt;
    error_integral = error_integral + error*dt;

    //Control servo value
    ang_servo = ang_servo + PROPORTIONAL_GAIN*(error) + DERIVATIVE_GAIN*(error_derivative) + INTEGRAL_GAIN*(error_integral);
    //Limiting the servo values to the maximum angle of EDF.
    if(abs(ang_servo) > 90 + MAXIMUM_ANGLE*servo_edf_ratio) {
        ang_servo = 90 + MAXIMUM_ANGLE*servo_edf_ratio;
    }
    return ang_servo;
  }
};

#endif