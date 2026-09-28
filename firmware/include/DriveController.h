#pragma once
#include "Encoder.h"
#include "MotorDriver.h"
#include "PIDController.h"

// Owns all four wheel subsystems (encoder + motor + PID) and converts a body-frame velocity command into synchronized, closed-loop per-wheel control, with an optional battery-voltage feed-forward term.

enum WheelIndex { WHEEL_FL = 0, WHEEL_RL = 1, WHEEL_FR = 2, WHEEL_RR = 3, WHEEL_COUNT = 4 };

struct WheelTelemetry {
    float targetRadPerSec = 0.0f;
    float actualRadPerSec = 0.0f;
    float commandOut = 0.0f; // final normalized motor command, post voltage-compensation
};

class DriveController {
public:
    void begin();

    // Body-frame velocity command, typically forwarded from an inbound SET_VELOCITY packet.
    void setVelocityCommand(float linear_mps, float angular_radps);

    // Zeroes target velocities and immediately cuts motor output; used both for an explicit STOP command and for the comm-loss failsafe.
    void stop();

    // Runs one control-loop iteration: samples encoders, runs the 4 PID loops, applies voltage compensation, and writes motor commands.
    void update(float dt, float compensationFactor);

    void setPIDGains(int wheelIdx, float kp, float ki, float kd);

    const WheelTelemetry& telemetry(int wheelIdx) const { return telemetry_[wheelIdx]; }

private:
    Encoder encoders_[WHEEL_COUNT];
    MotorDriver motors_[WHEEL_COUNT];
    PIDController* pids_[WHEEL_COUNT] = {nullptr, nullptr, nullptr, nullptr}; // heap-allocated: PIDController has no default ctor
    WheelTelemetry telemetry_[WHEEL_COUNT];

    float commandedLinear_ = 0.0f;
    float commandedAngular_ = 0.0f;
};
