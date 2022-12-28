#include <math.h>
#include "defines.h"

//Creating a new custom data type Quaternion
class Quaternion {
public:
    float w = 0.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    
    Quaternion() = default;
	~Quaternion() = default;
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    //Useful to convert quaternions to pitch, roll and yaw in that order of type vector 3 in radians
    Vector3 anglesToRadians(Quaternion q) {
        Vector3 angles;
        float pitch = -1*PI/2 + 2*atan2((1 + 2*(q.w*q.y - q.x*q.z))**0.5, (1 - 2*(q.w*q.y - q.x*q.z))**0.5);
        float roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x**2 + q.y**2));
        float yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2));
        angles.x = pitch, angles.y = roll, angles.z = yaw;
        return angles;
    }

    //Useful to convert quaternions to pitch, roll and yaw in that order of type vector 3 in degrees
    Vector3 anglesToDegrees(Quaternion q) {
        Vector3 angles;
        angles = angles.toRadians(q);
        angles.x *= RAD_TO_DEG, angles.y *= RAD_TO_DEG, angles.z *= RAD_TO_DEG;
        return angles;
    }
};

/* 
operator String() const {
return "[" + String(x) + "x," + String(y) + "y," + String(z) + "z]";
}
*/