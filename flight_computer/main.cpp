#include <cmath>
// TEMP:
#include <iostream>

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#ifndef HALF_PI
#define HALF_PI 1.5707963267948966192313216916398
#endif
#ifndef TWO_PI
#define TWO_PI 6.283185307179586476925286766559
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#ifndef sensor_warning
#define sensor_warning "Sensor type must be present in SensorPackage template list"
#endif
#ifndef constructor_warning
#define constructor_warning "Sensor type must be default constructible"
#endif

// https://en.cppreference.com/w/cpp/types/integral_constant
template<class T, T v>
struct integral_constant {
    static constexpr T value = v;
    using value_type = T;
    using type = integral_constant;
    constexpr operator value_type() const noexcept { return value; }
    constexpr value_type operator()() const noexcept { return value; }
};

using true_type = integral_constant<bool, true>;
using false_type = integral_constant<bool, false>;

// https://en.cppreference.com/w/cpp/types/void_t
template<typename... Ts> struct make_void { typedef void type; };
template<typename... Ts> using void_t = typename make_void<Ts...>::type;

// https://stackoverflow.com/a/46136532
template<typename T, typename = void>
struct is_default_constructible : false_type { };

template<typename T>
struct is_default_constructible<T, void_t<decltype(T())>> : true_type { };

// https://stackoverflow.com/a/39294584
template <typename T, typename... Args>
struct contains;

template <typename T>
struct contains<T> : false_type {};

template <typename T, typename... Args>
struct contains<T, T, Args...> : true_type {};

template <typename T, typename A, typename... Args>
struct contains<T, A, Args...> : contains<T, Args...> {};

template <typename L, typename... R> struct SensorPackage;

template <typename T>
T Clamp(const T& value, const T& low, const T& high) {
    return value < low ? low : (value > high ? high : value);
}

template <typename T>
struct Vector3 {
    T x{ 0 }, y{ 0 }, z{ 0 };
    
    Vector3() = default;
	~Vector3() = default;
    Vector3(T x, T y, T z) : x{ x }, y{ y }, z{ z } {}
	
    /*=
	operator String() const {
		return "[" + String(x) + "x," + String(y) + "y," + String(z) + "z]";
	}
	*/

    Vector3 DegreesToRadians() {
        return { x * DEG_TO_RAD, y * DEG_TO_RAD, z * DEG_TO_RAD };
    }
    Vector3 RadiansToDegrees() {
        return { x * RAD_TO_DEG, y * RAD_TO_DEG, z * RAD_TO_DEG };
    }
};

template <typename T>
struct Quaternion {
    T w{ 1 }, x{ 0 }, y{ 0 }, z{ 0 };
    
    Quaternion() = default;
	~Quaternion() = default;
    Quaternion(T w, T x, T y, T z) : w{ w }, x{ x }, y{ y }, z{ z } {}
	
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
    Quaternion operator*(T f) {
		return { w * f, x * f, y * f, z * f };
	}
	Quaternion operator/(T f) {
		return { w / f, x / f, y / f, z / f };
	}	
    Quaternion operator*(const Quaternion& q) {
        Quaternion<T> quaternion;
        quaternion.w = w * q.w - x * q.x - y * q.y - z * q.z;
        quaternion.x = w * q.x + x * q.w + y * q.z - z * q.y;
        quaternion.y = w * q.y - x * q.z + y * q.w + z * q.x;
        quaternion.z = w * q.z + x * q.y - y * q.x + z * q.w;
        return quaternion;
    }
	
    operator Vector3<T>() {
        Vector3<T> vector;
        // Roll (x-axis rotation).
        auto sinr_cosp = T(2) * (w * x + y * z);
        auto cosr_cosp = T(1) - T(2) * (x * x + y * y);
        vector.x = atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation).
        auto sinp = T(2) * (w * y - z * x);
        if (abs(sinp) >= T(1)) {
            vector.y = copysign(HALF_PI, sinp); // use 90 degrees if out of range
        } else {
            vector.y = asin(sinp);
        }
        // Yaw (z-axis rotation).
        auto siny_cosp = T(2) * (w * z + x * y);
        auto cosy_cosp = T(1) - T(2) * (y * y + z * z);
        vector.z = atan2(siny_cosp, cosy_cosp);
        
        return vector;
	}
    
    T Magnitude() {
		return sqrt(w * w + x * x + y * y + z * z);
	}
    Quaternion Normalized() {
		return *this / Magnitude();
	}
    // w is vector with respect to which the quaternion is differentiated.
    Quaternion Differentiated(const Vector3<T>& w) {
        Quaternion quaternion{ 0, w.x, w.y, w.z };
        return quaternion * (*this) / T(2);
    }
};

template <typename L>
struct SensorPackage<L> {
    static_assert(is_default_constructible<L>::value, constructor_warning);
    template <typename T>
    T& Get() {
        static_assert(contains<T, L>::value, sensor_warning);
        return sensor;
    }
protected:
    L sensor;
};

template <typename L, typename ...R>
struct SensorPackage : public SensorPackage<L>, public SensorPackage<R...> {
    template <typename T>
    T& Get() {
        static_assert(contains<T, L, R...>::value, sensor_warning);
        return this->SensorPackage<T>::template Get<T>();
    }
};

// TODO: Make filter actually return something.
// Currently filter acts as a empty pass-through container.
template <typename T>
struct Filter {
    Filter() = default;
    Filter(T value) : value{ value }, filtered_value{ value } {}
    T value{ 0 };
    T filtered_value{ 0 };
    T integral{ 0 };
    void UpdateValue(T value) {
        this->value = value;
        // TODO: Replace this.
        this->filtered_value = this->value;
    }
    T GetValue() {
        return filtered_value;
    }
};

struct Controller {
    static constexpr double INTEGRAL_GAIN = 2.0;
    static constexpr double TVC_POSITION_GAIN = 0.7278;
    static constexpr double TVC_VELOCITY_GAIN = 0.3640;
    template <typename T>
    static T LQR(T angular_position, T angular_velocity) {
        auto U_pos = -TVC_POSITION_GAIN * angular_position;
        auto U_vel = -TVC_VELOCITY_GAIN * angular_velocity;
        return U_pos + U_vel;
    }
};

struct BMP {
    void Init() {
        // TODO: Implement BMP instance here.
    }
    int GetTemperature() {
        return 30;
    }
    int GetAltitude() {
        return 2000;
    }
private:
    // ...
};

struct BNO {
    void Init() {
        // TODO: Implement BNO instance here.
    }
    Vector3<double> GetAngularVelocity() {
        return { 0.01, 0.01, 0.01 };
    }
    Vector3<double> GetAngularPosition() {
        return { 5, 5, 5 };
    }
private:
    // ...
};

struct FlightComputer {
    SensorPackage<BMP, BNO> sensors;
    Vector3<double> integral;
    Filter<double> y_filter;
    Filter<double> z_filter;
    // Initial orientation quaternion of the hopper.
    Quaternion<double> base{ 0.71, 0.71, 0.0, 0.0 };
    // 0.122173 radians is 7 in degrees.
    double maximum_angle = 0.122173; // unit: radians.
    
    void Init() {
        sensors.Get<BMP>().Init();
        sensors.Get<BNO>().Init();
    }
    
    void Update() {
        // Microcontroller time step calculation.
        // TODO: Change this to compute time from last update.
        auto dt = 0.0001;
    
        // Raw sensor data retrieval.
        auto raw_ang_pos = sensors.Get<BNO>().GetAngularPosition().DegreesToRadians();
        auto raw_ang_vel = sensors.Get<BNO>().GetAngularVelocity().DegreesToRadians();
        
        // Filtering of raw sensor data.
        y_filter.UpdateValue(raw_ang_vel.y);
        z_filter.UpdateValue(raw_ang_vel.z);
        Vector3<double> processed_ang_vel{ 
            raw_ang_vel.x, 
            y_filter.GetValue(), 
            z_filter.GetValue() 
        };
        
        // Orientation resolution to prevent gimbal lock.
        auto q_dot = base.Differentiated(processed_ang_vel);
        base += q_dot * dt;
        // TODO: Research, this normalization may not be required.
        base = base.Normalized();
        auto ang_pos = Vector3<double>(base).RadiansToDegrees();
    
        // Integrated control gains.
        integral.y += Controller::INTEGRAL_GAIN * ang_pos.y * dt;
        integral.z += Controller::INTEGRAL_GAIN * ang_pos.z * dt;
        
        // LQR.
        auto control_y = Controller::LQR(ang_pos.y, processed_ang_vel.y);
        auto control_z = Controller::LQR(ang_pos.z, processed_ang_vel.z);
        control_y += integral.y;
        control_z += integral.z;
        // Ensure control angle is never above or below the maximum controllability.
        control_y = Clamp(control_y, -maximum_angle, maximum_angle); 
        control_z = Clamp(control_z, -maximum_angle, maximum_angle);

        // Update servos with latest values.
        // TODO: Make this write the calculated control angles:
        // Servos.write(U_y, U_z)

        // TEMP:
        std::cout << control_y << std::endl;
        std::cout << control_z << std::endl;
    }
};

FlightComputer fc;

void setup() {
    fc.Init();
}

void loop() {
    fc.Update();
}

int main() {
    setup();
    loop();
    return 0;
}