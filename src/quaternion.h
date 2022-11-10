// TODO: Add header include guards.
#include "vector3.h"
#include <math.h>

class Quaternion {
public:
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    
    Quaternion() = default;
	 ~Quaternion() = default;
    Quaternion(float w, float x, float y, float z) : w{ w }, x{ x }, y{ y }, z{ z } {}
	
    /*
    operator String() const {
      return "[" + String(w) + "," + String(x) + "i," + String(y) + "j," + String(z) + "k]";
    }
    */
    Quaternion& operator+=(const Quaternion& rhs) {
        w += rhs.w;
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
    Quaternion& operator-=(const Quaternion& rhs) {
        w -= rhs.w;
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    Quaternion operator*(float f) {
      return { w * f, x * f, y * f, z * f };
    }
    Quaternion operator/(float f) {
      return { w / f, x / f, y / f, z / f };
    }	
    Quaternion operator*(const Quaternion& q) {
        Quaternion quaternion;
        quaternion.w = w * q.w - x * q.x - y * q.y - z * q.z;
        quaternion.x = w * q.x + x * q.w + y * q.z - z * q.y;
        quaternion.y = w * q.y - x * q.z + y * q.w + z * q.x;
        quaternion.z = w * q.z + x * q.y - y * q.x + z * q.w;
        return quaternion;
    }
	
    operator Vector3() {
      Vector3 vector;
      // Roll (x-axis rotation).
      float sinr_cosp = 2.0f * (w * x + y * z);
      float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
      vector.x = atan2(sinr_cosp, cosr_cosp);

      // Pitch (y-axis rotation).
      float sinp = 2.0f * (w * y - z * x);
      if (abs(sinp) >= 1.0f) {
          vector.y = copysign(HALF_PI, sinp); // use 90 degrees if out of range
      } else {
          vector.y = asin(sinp);
      }
      // Yaw (z-axis rotation).
      float siny_cosp = 2.0f * (w * z + x * y);
      float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
      vector.z = atan2(siny_cosp, cosy_cosp);
      
      return vector;
    }
    
    float Magnitude() {
		   return sqrtf(w * w + x * x + y * y + z * z);
	  }
    Quaternion Normalized() {
		  return *this / Magnitude();
	  }
    // w is vector with respect to which the quaternion is differentiated.
    Quaternion Differentiated(const Vector3& w) {
        Quaternion quaternion{ 0, w.x, w.y, w.z };
        return quaternion * (*this) / 2.0f;
    }
};