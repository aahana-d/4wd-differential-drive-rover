// Host-run unit tests for PIDController

// Run with: pio test -e native -f test_pid

#include <unity.h>
#include "PIDController.h"

void test_pid_converges_to_setpoint_on_stable_plant() {
    PIDController pid(0.5f, 1.0f, 0.0f, -1.0f, 1.0f);
    float measured = 0.0f;
    const float setpoint = 5.0f; // rad/s
    const float dt = 0.02f;

    for (int i = 0; i < 500; ++i) {
        const float out = pid.compute(setpoint, measured, dt);
        measured += out * 0.5f; // arbitrary stable first-order response
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, setpoint, measured);
}

void test_pid_output_is_saturated() {
    PIDController pid(100.0f, 0.0f, 0.0f, -1.0f, 1.0f); // huge Kp forces saturation
    float out = pid.compute(10.0f, 0.0f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, out);

    out = pid.compute(-10.0f, 0.0f, 0.02f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, out);
}

void test_pid_reset_clears_integral_windup() {
    PIDController pid(0.0f, 2.0f, 0.0f, -1.0f, 1.0f);
    for (int i = 0; i < 50; ++i) pid.compute(1.0f, 0.0f, 0.02f); // wind up the integral term
    pid.reset();
    const float out = pid.compute(0.0f, 0.0f, 0.02f); // zero error right after reset
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out);
}

void test_pid_handles_invalid_dt_gracefully() {
    PIDController pid(1.0f, 1.0f, 1.0f, -1.0f, 1.0f);
    const float first = pid.compute(5.0f, 0.0f, 0.02f);
    const float second = pid.compute(5.0f, 0.0f, 0.0f); // dt == 0 must not crash or NaN
    TEST_ASSERT_EQUAL_FLOAT(first, second); // guarded path returns the previous output unchanged
}

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_converges_to_setpoint_on_stable_plant);
    RUN_TEST(test_pid_output_is_saturated);
    RUN_TEST(test_pid_reset_clears_integral_windup);
    RUN_TEST(test_pid_handles_invalid_dt_gracefully);
    return UNITY_END();
}
