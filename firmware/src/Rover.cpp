#include "Rover.h"
#include "config.h"
#include <Arduino.h>

void Rover::begin() {
    battery_.begin(VBATT_ADC_PIN, VBATT_DIVIDER_RATIO, ADC_REF_V, ADC_MAX_COUNTS);
    drive_.begin();
    commandProcessor_.begin(&drive_, &battery_);
    watchdog_.begin(HW_WATCHDOG_TIMEOUT_S);

    lastControlMs_ = millis();
    lastTelemetryMs_ = millis();
}

void Rover::update() {
    const uint32_t now = millis();

    // Drain/parse any pending Bluetooth bytes every loop() iteration.
    commandProcessor_.poll(now);

    // Communication-loss failsafe 
    const bool commLost = commandProcessor_.isCommLost(now, COMM_TIMEOUT_MS);
    if (commLost && !failsafeLatched_) {
        drive_.stop(); // immediate motor shutdown, doesn't wait for the next control tick
        failsafeLatched_ = true;
    } else if (!commLost && failsafeLatched_) {
        failsafeLatched_ = false; // link recovered; normal commands may resume
    }

    // Fixed-rate control loop 
    if (now - lastControlMs_ >= CONTROL_LOOP_PERIOD_MS) {
        const float dt = static_cast<float>(now - lastControlMs_) / 1000.0f;
        lastControlMs_ = now;

        battery_.update();

        if (!failsafeLatched_) {
            const float comp = battery_.compensationFactor(BATTERY_NOMINAL_V, COMP_FACTOR_MIN, COMP_FACTOR_MAX);
            drive_.update(dt, comp);
        }
        // deliberately skip drive_.update() so the PID loops don't wind up against stationary wheels
    }

    if (now - lastTelemetryMs_ >= TELEMETRY_PERIOD_MS) {
        lastTelemetryMs_ = now;
        commandProcessor_.sendTelemetry(now);
    }

    // Hardware watchdog: if anything above ever blocks for longer than HW_WATCHDOG_TIMEOUT_S, the MCU resets rather than leaving the motors in an uncontrolled state.
    watchdog_.feed();
}
