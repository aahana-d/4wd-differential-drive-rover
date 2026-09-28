#include "BatteryMonitor.h"
#include <Arduino.h>
#include <algorithm>

void BatteryMonitor::begin(int adcPin, float dividerRatio, float adcRefV, int adcMaxCounts) {
    adcPin_ = adcPin;
    dividerRatio_ = dividerRatio;
    adcRefV_ = adcRefV;
    adcMaxCounts_ = adcMaxCounts;
    pinMode(adcPin_, INPUT);
}

void BatteryMonitor::update() {
    const int raw = analogRead(adcPin_);
    const float pinVoltage = (static_cast<float>(raw) / static_cast<float>(adcMaxCounts_)) * adcRefV_;
    const float battVoltage = pinVoltage * dividerRatio_;

    if (firstSample_) {
        filteredVoltage_ = battVoltage;
        firstSample_ = false;
    } else {
        filteredVoltage_ = EMA_ALPHA * battVoltage + (1.0f - EMA_ALPHA) * filteredVoltage_;
    }
}

float BatteryMonitor::compensationFactor(float nominalV, float compMin, float compMax) const {
    if (!enabled_ || filteredVoltage_ <= 0.1f) return 1.0f; // guard against sensor fault / div-by-~0

    float factor = nominalV / filteredVoltage_;
    factor = std::clamp(factor, compMin, compMax);
    return factor;
}
