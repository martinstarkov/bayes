#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include "vector.h"
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
    float pitch = -1*PI/2 + 2*atan2(sqrt(1 + 2*(q.w*q.y - q.x*q.z)), sqrt(1 - 2*(q.w*q.y - q.x*q.z)));
    float roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
    float yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    angles.x = pitch, angles.y = roll, angles.z = yaw;
    return angles;
  }

  //Useful to convert quaternions to pitch, roll and yaw in that order of type vector 3 in degrees
  Vector3 anglesToDegrees(Quaternion q) {
    Vector3 angles = q.anglesToRadians(q);
    angles.x *= RAD_TO_DEG, angles.y *= RAD_TO_DEG, angles.z *= RAD_TO_DEG;
    return angles;
  }
};

#endif