#pragma once
// Standard discrete PID controller with integral clamping (anti-windup) and output saturation.
// Deliberately pure C++ so it can be exercised by host-side native unit tests as well as run on the MCU
    // see firmware/test/test_native/test_pid.cpp.

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outMin, float outMax)
        : kp_(kp), ki_(ki), kd_(kd), outMin_(outMin), outMax_(outMax) {}

    void setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

    void reset() {
        integral_ = 0.0f;
        prevError_ = 0.0f;
        firstRun_ = true;
    }

    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return lastOutput_; // guard against a bad/zero dt

        const float error = setpoint - measured;

        integral_ += error * dt;
        // clamp the integral term itself so it can never by itself drive the output past outMax_/outMin_
        const float kiSafe = ki_ > 1e-6f ? ki_ : 1e-6f;
        const float integralLimit = (outMax_ - outMin_) / kiSafe;
        if (integral_ > integralLimit) integral_ = integralLimit;
        if (integral_ < -integralLimit) integral_ = -integralLimit;

        float derivative = 0.0f;
        if (!firstRun_) {
            derivative = (error - prevError_) / dt;
        }
        firstRun_ = false;
        prevError_ = error;

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        if (output > outMax_) output = outMax_;
        if (output < outMin_) output = outMin_;

        lastOutput_ = output;
        return output;
    }

private:
    float kp_, ki_, kd_;
    float outMin_, outMax_;
    float integral_ = 0.0f;
    float prevError_ = 0.0f;
    float lastOutput_ = 0.0f;
    bool firstRun_ = true;
};
