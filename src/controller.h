#include <math.h>
#include "defines.h"
#include "vector3.h"

//For a better understanding check https://toptechboy.com/9-axis-imu-lesson-25-proportional-control-system-for-self-leveling-platform/
class Controller {
public:
    static float rollServoVal = HALF_PI, pitchServoVal = HALF_PI;
    Vector3 PID(float pitchActual, float rollActual, float dt) {
        Vector3 servoAngle;

        //Measuring all error terms in roll
        rollErrorOld = rollError;
        rollError = rollTarget - rollActual;
        rollErrorChange = rollError - rollErrorOld;
        rollDerivativeError = rollErrorChange/dt;
        rollIntegralError = rollIntegralError + rollError*dt;

        //Measuring all error terms in pitch
        rollErrorOld = rollError;
        rollError = rollTarget - rollActual;
        rollErrorChange = rollError - rollErrorOld;
        rollDerivativeError = rollErrorChange/dt;
        rollIntegralError = rollIntegralError + rollError*dt;

        //Control servo value
        rollServoVal = rollServoVal + PROPORTIONAL_GAIN*(rollError) + DERIVATIVE_GAIN*(rollDerivativeError) + INTEGRAL_GAIN*(rollIntegralError);
        pitchServoVal = pitchServoVal + PROPORTIONAL_GAIN*(pitchError) + DERIVATIVE_GAIN*(pitchDerivativeError) + INTEGRAL_GAIN*(pitchIntegralError);
        //Limiting the servo values to the maximum angle of EDF.
        if(abs(rollServoVal) >= MAXIMUM_ANGLE*SERVO_EDF_RATIO1) {
            rollServoVal = MAXIMUM_ANGLE*SERVO_EDF_RATIO1;
        }
        if(abs(pitchServoVal) >= MAXIMUM_ANGLE*SERVO_EDF_RATIO2) {
            pitchServoVal = MAXIMUM_ANGLE*SERVO_EDF_RATIO2;
        }
        servoAngle.x = rollServoVal, servoAngle.y = pitchServoVal, servoAngle.z = 0;
        servoAngle = servoAngle.toDegrees(servoAngle);
        return servoAngle;
    }
private:
    static float rollTarget = 0.0f, rollError = 0.0f, rollErrorChange = 0.0f, rollDerivativeError = 0.0f, rollIntegralError = 0.0f, rollErrorOld = 0.0f;
    static float pitchTarget = 0.0f, pitchError = 0.0f, pitchErrorChange = 0.0f, pitchDerivativeError = 0.0f, pitchIntegralError = 0.0f, pitchErrorOld = 0.0f;
    static constexpr float MAXIMUM_ANGLE = 0.122173f;
    static constexpr float SERVO_EDF_RATIO1, SERVO_EDF_RATIO2;
    static constexpr float PROPORTIONAL_GAIN = 0.7278f;
    static constexpr float DERIVATIVE_GAIN = 2.0f;
    static constexpr float INTEGRAL_GAIN = 0.3640f;
};