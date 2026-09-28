#pragma once
#include "DriveController.h"
#include "BatteryMonitor.h"
#include "CommandProcessor.h"
#include "Watchdog.h"
#include <cstdint>

// Owns every subsystem and schedules the fixed control / telemetry / failsafe / watchdog cadence.

class Rover {
public:
    void begin();
    void update();

private:
    DriveController drive_;
    BatteryMonitor battery_;
    CommandProcessor commandProcessor_;
    Watchdog watchdog_;

    uint32_t lastControlMs_ = 0;
    uint32_t lastTelemetryMs_ = 0;
    bool failsafeLatched_ = false;
};
