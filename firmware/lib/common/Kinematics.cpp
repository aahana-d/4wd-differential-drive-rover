#include "Kinematics.h"

namespace rover_kinematics {

void differentialDriveTargets(float linear_mps, float angular_radps,
                               float track_width_m, float wheel_radius_m,
                               float& leftWheelRadPerSec, float& rightWheelRadPerSec) {
    // Standard differential-drive inverse kinematics:
    //   v_left  = v - (w * L / 2)
    //   v_right = v + (w * L / 2)
    const float halfTrack = track_width_m * 0.5f;
    const float leftLinear  = linear_mps - angular_radps * halfTrack;
    const float rightLinear = linear_mps + angular_radps * halfTrack;

    // Convert linear wheel-surface velocity (m/s) into wheel angular velocity (rad/s).
    leftWheelRadPerSec  = leftLinear  / wheel_radius_m;
    rightWheelRadPerSec = rightLinear / wheel_radius_m;
}

} // namespace rover_kinematics
