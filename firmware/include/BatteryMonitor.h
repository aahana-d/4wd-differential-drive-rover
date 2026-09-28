#pragma once
// Reads pack voltage through a resistor divider and computes a feed-forward compensation factor used to counteract voltage-drop- induced motor drift as the battery discharges under load.

class BatteryMonitor {
public:
    void begin(int adcPin, float dividerRatio, float adcRefV, int adcMaxCounts);

    // Call once per control-loop tick; applies an EMA filter to the raw ADC reading.
    void update();

    float voltage() const { return filteredVoltage_; }

    // Returns a multiplier (>= compMin) to apply to the PID's normalized motor command so that the effective voltage delivered to the motor stays closer to what it would be at nominal pack voltage, rather than relying solely on the PID integral term
    // Clamped to [compMin, compMax] to avoid overdriving the hardware when the pack is deeply sagged or the sensor glitches.
    float compensationFactor(float nominalV, float compMin, float compMax) const;

    // Lets CommandProcessor toggle this via SET_COMP_MODE for A/B bench testing.
    void setEnabled(bool enabled) { enabled_ = enabled; }

private:
    int adcPin_ = -1;
    float dividerRatio_ = 1.0f;
    float adcRefV_ = 3.3f;
    int adcMaxCounts_ = 4095;
    float filteredVoltage_ = 0.0f;
    bool firstSample_ = true;
    bool enabled_ = true;

    static constexpr float EMA_ALPHA = 0.1f;
};
