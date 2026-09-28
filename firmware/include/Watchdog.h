#pragma once
#include <cstdint>

// Thin wrapper around the ESP32 hardware/task watchdog
// Separate safety layer from the communication-loss failsafe in CommandProcessor:
//   - CommandProcessor handles "the Bluetooth link dropped but the firmware itself is still running fine".
//   - This handles "the firmware itself locked up" (e.g. stuck in an ISR, a blocking call that never returns), which would otherwise leave motors driving on stale commands forever with no way to recover except a manual power cycle.

class Watchdog {
public:
    void begin(uint32_t timeoutSeconds);
    void feed();
};
