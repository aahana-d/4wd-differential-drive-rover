// Host-run unit tests for the differential-drive inverse kinematics.

// Run with: pio test -e native -f test_kinematics

#include <unity.h>
#include "Kinematics.h"

using namespace rover_kinematics;

void test_straight_line_targets_are_equal() {
    float left, right;
    differentialDriveTargets(0.5f, 0.0f, 0.18f, 0.0325f, left, right);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, left, right);
}

void test_pure_rotation_targets_are_opposite_sign() {
    float left, right;
    differentialDriveTargets(0.0f, 2.0f, 0.18f, 0.0325f, left, right);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -left, right);
    TEST_ASSERT_TRUE(right > 0.0f); // positive angular = CCW = right side spins forward
}

void test_combined_motion_matches_hand_calculation() {
    float left, right;
    const float v = 0.4f, w = 1.0f, trackWidth = 0.2f, wheelRadius = 0.03f;
    differentialDriveTargets(v, w, trackWidth, wheelRadius, left, right);

    const float expectedLeft  = (v - w * trackWidth / 2.0f) / wheelRadius;
    const float expectedRight = (v + w * trackWidth / 2.0f) / wheelRadius;

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expectedLeft, left);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, expectedRight, right);
}

void test_zero_command_yields_zero_targets() {
    float left, right;
    differentialDriveTargets(0.0f, 0.0f, 0.18f, 0.0325f, left, right);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, left);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, right);
}

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_straight_line_targets_are_equal);
    RUN_TEST(test_pure_rotation_targets_are_opposite_sign);
    RUN_TEST(test_combined_motion_matches_hand_calculation);
    RUN_TEST(test_zero_command_yields_zero_targets);
    return UNITY_END();
}
