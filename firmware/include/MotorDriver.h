#pragma once
// Wraps a single TB6612FNG driver channel: one PWM (ESP32 LEDC) pin plus two direction pins.

class MotorDriver {
public:
    void begin(int pwmPin, int in1Pin, int in2Pin, int ledcChannel,
               int freqHz, int resolutionBits);

    // command in [-1.0, 1.0]; sign selects direction, magnitude sets duty cycle.
    void setCommand(float command);

    void stop();  // coast: both direction pins low, duty 0
    void brake(); // active short-brake: both direction pins high, duty 0

private:
    int in1_ = -1;
    int in2_ = -1;
    int ledcChannel_ = -1;
    int maxDuty_ = 1023;
};
