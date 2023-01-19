#ifndef VECTOR_H
#define VECTOR_H

#include "defines.h"
//Creating a new custom data type Vector3
class Vector3 {
public:
  float x = 0.0f, y = 0.0f, z = 0.0f;
    
  //Default constructor and destructor
  Vector3() = default;
  ~Vector3() = default;
  Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

  //Converts vector3 type variable to radians
  Vector3 toRadians(Vector3 vector) {
      vector.x *= DEG_TO_RAD, vector.y *= DEG_TO_RAD, vector.z *= DEG_TO_RAD;
      return vector;
      //note to use this on some variable x, the code becomes x = x.toRadians(x);
  }
  //Converts vector3 type variable to degrees
  Vector3 toDegrees(Vector3 vector) {
      vector.x *= RAD_TO_DEG, vector.y *= RAD_TO_DEG, vector.z *= RAD_TO_DEG;
      return vector;
      //note to use this on some variable x, the code becomes x = x.toDegrees(x);
  }
};

#endif