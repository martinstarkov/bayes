// TODO: Add header include guards.

class Controller {
public:
    static float LQR(float angular_position, float angular_velocity) {
        float U_pos = -TVC_POSITION_GAIN * angular_position;
        float U_vel = -TVC_VELOCITY_GAIN * angular_velocity;
        return U_pos + U_vel;
    }
private:
    static constexpr float INTEGRAL_GAIN = 2.0f;
    static constexpr float TVC_POSITION_GAIN = 0.7278f;
    static constexpr float TVC_VELOCITY_GAIN = 0.3640f;
};