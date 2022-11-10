// TODO: Add header include guards.

#include "defines.h"

class Vector3 {
public:
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    
    Vector3() = default;
	 ~Vector3() = default;
    Vector3(float x, float y, float z) : x{ x }, y{ y }, z{ z } {}
	
    /*=
	operator String() const {
		return "[" + String(x) + "x," + String(y) + "y," + String(z) + "z]";
	}
	*/

    Vector3 ToRadians() {
        return { x * DEG_TO_RAD, y * DEG_TO_RAD, z * DEG_TO_RAD };
    }
    Vector3 ToDegrees() {
        return { x * RAD_TO_DEG, y * RAD_TO_DEG, z * RAD_TO_DEG };
    }
};