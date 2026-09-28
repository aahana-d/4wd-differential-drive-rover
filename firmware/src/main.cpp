#include <Arduino.h>
#include "Rover.h"

Rover rover;

void setup() {
    Serial.begin(115200);
    rover.begin();
}

void loop() {
    rover.update();
}
