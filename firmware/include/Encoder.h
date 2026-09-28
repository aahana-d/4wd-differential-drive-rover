#pragma once
#include <cstdint>

// Quadrature encoder reader using a single hardware interrupt on channel A; direction is determined by reading channel B at the edge (1x decode).
// TODO: For finer resolution, switch to full 4x decoding or the ESP32's dedicated PCNT (pulse counter) peripheral.

class Encoder {
public:
    // pinA MUST be interrupt-capable on the target MCU.
    void begin(int pinA, int pinB);

    // Call once per control-loop tick with the elapsed time since the previous call.
    float sampleAngularVelocity(float dt, int ticksPerRev);

    int32_t totalTicks() const { return tickCount_; }

private:
    static void IRAM_ATTR isrTrampoline0();
    static void IRAM_ATTR isrTrampoline1();
    static void IRAM_ATTR isrTrampoline2();
    static void IRAM_ATTR isrTrampoline3();
    void IRAM_ATTR handleInterrupt();

    int pinA_ = -1;
    int pinB_ = -1;
    volatile int32_t tickCount_ = 0;
    int32_t lastTickCount_ = 0;

    static Encoder* instances_[4];
    static int instanceCount_;
};
