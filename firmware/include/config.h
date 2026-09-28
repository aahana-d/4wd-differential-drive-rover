#pragma once
// Central place for pins, physical constants, and tuning parameters.

#include <cstdint>

// kinematic constants 
constexpr float WHEEL_RADIUS_M        = 0.0325f;  // 65 mm diameter wheels
constexpr float TRACK_WIDTH_M         = 0.180f;   // distance between left/right wheel centerlines
constexpr int   ENCODER_TICKS_PER_REV = 660;      // motor + gearbox + encoder CPR after gear reduction

// Control loop timing 
constexpr uint32_t CONTROL_LOOP_PERIOD_MS = 20;   // 50 Hz PID update
constexpr uint32_t TELEMETRY_PERIOD_MS    = 200;  // 5 Hz telemetry push
constexpr uint32_t COMM_TIMEOUT_MS        = 500;  // failsafe: stop if no valid packet within this window
constexpr uint32_t HW_WATCHDOG_TIMEOUT_S  = 2;    // hardware watchdog timeout

// Motion limits
constexpr float MAX_LINEAR_MPS   = 1.0f;
constexpr float MAX_ANGULAR_RADS = 4.0f;

// Default PID gains
constexpr float DEFAULT_KP  = 2.2f;
constexpr float DEFAULT_KI  = 6.0f;
constexpr float DEFAULT_KD  = 0.02f;
constexpr float PID_OUT_MIN = -1.0f; // normalized motor command
constexpr float PID_OUT_MAX =  1.0f;

// Battery / voltage compensation
constexpr float BATTERY_NOMINAL_V   = 11.1f; // 3S LiPo nominal
constexpr float BATTERY_MIN_V       = 9.0f;  // below this, treat as fully sagged / low-battery warning
constexpr float COMP_FACTOR_MIN     = 1.0f;  // never reduce PWM below the nominal-voltage baseline
constexpr float COMP_FACTOR_MAX     = 1.6f;  // clamp so we never over-drive the motors/electronics when heavily sagged
constexpr int   VBATT_ADC_PIN       = 34;    // ADC1 channel fed by the voltage divider
constexpr float VBATT_DIVIDER_RATIO = 3.0f;  // (R1+R2)/R2 of the divider feeding the ADC
constexpr float ADC_REF_V           = 3.3f;
constexpr int   ADC_MAX_COUNTS      = 4095;  // 12-bit ADC

// Motor driver pins (2x TB6612FNG => 4 channels)
// Each wheel: one PWM pin (LEDC channel) + two direction pins.
struct MotorPins { int pwm; int in1; int in2; int ledcChannel; };

constexpr MotorPins MOTOR_FL = {25, 26, 27, 0}; // front-left
constexpr MotorPins MOTOR_RL = {14, 12, 13, 1}; // rear-left
constexpr MotorPins MOTOR_FR = {33, 32, 15, 2}; // front-right
constexpr MotorPins MOTOR_RR = {2,   4,  5, 3}; // rear-right

constexpr int PWM_FREQ_HZ         = 20000; // above audible range
constexpr int PWM_RESOLUTION_BITS = 10;    // 0-1023 duty

// Encoder pins (channel A must be interrupt-capable)
struct EncoderPins { int a; int b; };

constexpr EncoderPins ENC_FL = {36, 39};
constexpr EncoderPins ENC_RL = {34, 35};
constexpr EncoderPins ENC_FR = {16, 17};
constexpr EncoderPins ENC_RR = {18, 19};

// NOTE (TODO): pins 34-39 on most ESP32 dev boards are input-only, which is fine for encoder channel A/B (we only ever read them / attach interrupts), but they cannot drive outputs.
// Double-check this pin map against dev-board silkscreen and confirm no conflicts with VBATT_ADC_PIN before wiring the harness.

// Bluetooth
constexpr char BLUETOOTH_DEVICE_NAME[] = "ROVER-4WD-01";
