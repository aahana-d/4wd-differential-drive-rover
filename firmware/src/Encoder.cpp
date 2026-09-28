#include "Encoder.h"
#include <Arduino.h>

Encoder* Encoder::instances_[4] = {nullptr, nullptr, nullptr, nullptr};
int Encoder::instanceCount_ = 0;

void Encoder::begin(int pinA, int pinB) {
    pinA_ = pinA;
    pinB_ = pinB;
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);

    if (instanceCount_ >= 4) return; // safety guard; shouldn't happen on a 4WD build
    const int idx = instanceCount_++;
    instances_[idx] = this;

    switch (idx) {
        case 0: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline0, RISING); break;
        case 1: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline1, RISING); break;
        case 2: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline2, RISING); break;
        case 3: attachInterrupt(digitalPinToInterrupt(pinA_), isrTrampoline3, RISING); break;
    }
}

void IRAM_ATTR Encoder::isrTrampoline0() { if (instances_[0]) instances_[0]->handleInterrupt(); }
void IRAM_ATTR Encoder::isrTrampoline1() { if (instances_[1]) instances_[1]->handleInterrupt(); }
void IRAM_ATTR Encoder::isrTrampoline2() { if (instances_[2]) instances_[2]->handleInterrupt(); }
void IRAM_ATTR Encoder::isrTrampoline3() { if (instances_[3]) instances_[3]->handleInterrupt(); }

void IRAM_ATTR Encoder::handleInterrupt() {
    // Quadrature direction: if B is HIGH on the rising edge of A, the shaft is turning "forward" per our wiring convention
    if (digitalRead(pinB_) == HIGH) {
        tickCount_++;
    } else {
        tickCount_--;
    }
}

float Encoder::sampleAngularVelocity(float dt, int ticksPerRev) {
    if (dt <= 0.0f || ticksPerRev <= 0) return 0.0f;

    noInterrupts();
    const int32_t current = tickCount_;
    interrupts();

    const int32_t delta = current - lastTickCount_;
    lastTickCount_ = current;

    const float revolutions = static_cast<float>(delta) / static_cast<float>(ticksPerRev);
    const float radiansPerSec = (revolutions * 2.0f * 3.14159265f) / dt;
    return radiansPerSec;
}
