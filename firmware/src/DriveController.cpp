#include "DriveController.h"
#include "Kinematics.h"
#include "config.h"

namespace {
// Left-side wheels share one target angular velocity; right-side wheels share the other, per differential-drive kinematics.
inline bool isLeftWheel(int idx) { return idx == WHEEL_FL || idx == WHEEL_RL; }
}

void DriveController::begin() {
    encoders_[WHEEL_FL].begin(ENC_FL.a, ENC_FL.b);
    encoders_[WHEEL_RL].begin(ENC_RL.a, ENC_RL.b);
    encoders_[WHEEL_FR].begin(ENC_FR.a, ENC_FR.b);
    encoders_[WHEEL_RR].begin(ENC_RR.a, ENC_RR.b);

    motors_[WHEEL_FL].begin(MOTOR_FL.pwm, MOTOR_FL.in1, MOTOR_FL.in2, MOTOR_FL.ledcChannel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    motors_[WHEEL_RL].begin(MOTOR_RL.pwm, MOTOR_RL.in1, MOTOR_RL.in2, MOTOR_RL.ledcChannel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    motors_[WHEEL_FR].begin(MOTOR_FR.pwm, MOTOR_FR.in1, MOTOR_FR.in2, MOTOR_FR.ledcChannel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);
    motors_[WHEEL_RR].begin(MOTOR_RR.pwm, MOTOR_RR.in1, MOTOR_RR.in2, MOTOR_RR.ledcChannel, PWM_FREQ_HZ, PWM_RESOLUTION_BITS);

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        pids_[i] = new PIDController(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, PID_OUT_MIN, PID_OUT_MAX);
    }
}

void DriveController::setVelocityCommand(float linear_mps, float angular_radps) {
    commandedLinear_ = linear_mps;
    commandedAngular_ = angular_radps;
}

void DriveController::stop() {
    commandedLinear_ = 0.0f;
    commandedAngular_ = 0.0f;
    for (int i = 0; i < WHEEL_COUNT; ++i) {
        pids_[i]->reset();
        motors_[i].stop(); // immediate, don't wait for the next PID cycle
        telemetry_[i].targetRadPerSec = 0.0f;
        telemetry_[i].commandOut = 0.0f;
    }
}

void DriveController::update(float dt, float compensationFactor) {
    float leftTarget = 0.0f, rightTarget = 0.0f;
    rover_kinematics::differentialDriveTargets(commandedLinear_, commandedAngular_,
                                                TRACK_WIDTH_M, WHEEL_RADIUS_M,
                                                leftTarget, rightTarget);

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        const float target = isLeftWheel(i) ? leftTarget : rightTarget;
        const float actual = encoders_[i].sampleAngularVelocity(dt, ENCODER_TICKS_PER_REV);

        const float pidOut = pids_[i]->compute(target, actual, dt);

        // Voltage-drop feed-forward compensation: as the battery sags, compensationFactor > 1.0 boosts the commanded duty cycle so the effective voltage delivered to the motor stays closer to what it would be at nominal pack voltage.
        float compensatedOut = pidOut * compensationFactor;
        if (compensatedOut > PID_OUT_MAX) compensatedOut = PID_OUT_MAX;
        if (compensatedOut < PID_OUT_MIN) compensatedOut = PID_OUT_MIN;

        motors_[i].setCommand(compensatedOut);

        telemetry_[i].targetRadPerSec = target;
        telemetry_[i].actualRadPerSec = actual;
        telemetry_[i].commandOut = compensatedOut;
    }
}

void DriveController::setPIDGains(int wheelIdx, float kp, float ki, float kd) {
    if (wheelIdx < 0 || wheelIdx >= WHEEL_COUNT) return;
    pids_[wheelIdx]->setGains(kp, ki, kd);
}
