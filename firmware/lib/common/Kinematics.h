#pragma once
// Pure differential-drive inverse-kinematics math, shared between the firmware's DriveController and the host-side native unit tests.

namespace rover_kinematics {

// Converts a desired body-frame velocity command into per-side wheel angular-velocity targets for a differential-drive 4WD rover.

//   linear_mps        - desired forward speed of the rover body (m/s)
//   angular_radps      - desired yaw rate, positive = counter-clockwise (rad/s)
//   track_width_m      - distance between the left and right wheel centerlines (m)
//   wheel_radius_m     - wheel radius (m)
void differentialDriveTargets(float linear_mps, float angular_radps,
                               float track_width_m, float wheel_radius_m,
                               float& leftWheelRadPerSec, float& rightWheelRadPerSec);

} // namespace rover_kinematics
