#ifndef VECTOR3_H
#define VECTOR3_H

#include "defines.h"

//Creating a new custom data type Vector3
class Vector3 {
public:
  float x = 0.0f, y = 0.0f, z = 0.0f;
    
  //Constructor
  Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

  //Converts vector3 type variable to radians
  Vector3 toRadians() const{
    return Vector3(x * DEG_TO_RAD, y * DEG_TO_RAD, z * DEG_TO_RAD);
      //note to use this on some variable x, the code becomes x = x.toRadians();
  }
  //Converts vector3 type variable to degrees
  Vector3 toDegrees() const{
    return Vector3(x * RAD_TO_DEG, y * RAD_TO_DEG, z * RAD_TO_DEG);
      //note to use this on some variable x, the code becomes x = x.toDegrees();
  }
};

#endif