#include "MotorDriver.h"
#include <Arduino.h>
#include <cmath>
#include <algorithm>

void MotorDriver::begin(int pwmPin, int in1Pin, int in2Pin, int ledcChannel,
                         int freqHz, int resolutionBits) {
    in1_ = in1Pin;
    in2_ = in2Pin;
    ledcChannel_ = ledcChannel;
    maxDuty_ = (1 << resolutionBits) - 1;

    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);

    ledcSetup(ledcChannel_, freqHz, resolutionBits);
    ledcAttachPin(pwmPin, ledcChannel_);
    ledcWrite(ledcChannel_, 0);
}

void MotorDriver::setCommand(float command) {
    command = std::clamp(command, -1.0f, 1.0f);

    if (command >= 0.0f) {
        digitalWrite(in1_, HIGH);
        digitalWrite(in2_, LOW);
    } else {
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, HIGH);
    }

    const int duty = static_cast<int>(fabsf(command) * static_cast<float>(maxDuty_));
    ledcWrite(ledcChannel_, duty);
}

void MotorDriver::stop() {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
    ledcWrite(ledcChannel_, 0);
}

void MotorDriver::brake() {
    digitalWrite(in1_, HIGH);
    digitalWrite(in2_, HIGH);
    ledcWrite(ledcChannel_, 0);
}
