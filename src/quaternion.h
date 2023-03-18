#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "vector.h"

//Creating a new custom data type Quaternion
class Quaternion {
public:
  float w = 0.0f, x = 0.0f, y = 0.0f, z = 0.0f;
  
  Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

  //Useful to convert quaternions to pitch, roll and yaw in that order of type vector 3 in radians
  Vector3 toRadians() const{
    float pitch = -1*PI/2 + 2*atan2(sqrt(1 + 2*(w*y - x*z)), sqrt(1 - 2*(w*y - x*z)));
    float roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
    float yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
    return Vector3(pitch, roll, yaw);
  }

  //Useful to convert quaternions to pitch, roll and yaw in that order of type vector 3 in degrees
  Vector3 toDegrees() const{
    Vector3 angles = toRadians();
    angles.x *= RAD_TO_DEG, angles.y *= RAD_TO_DEG, angles.z *= RAD_TO_DEG;
    return angles;
  }
};

#endif