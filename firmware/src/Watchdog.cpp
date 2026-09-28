#include "Watchdog.h"
#include <esp_task_wdt.h>

void Watchdog::begin(uint32_t timeoutSeconds) {
    esp_task_wdt_init(timeoutSeconds, true /* panic + reset the MCU on timeout */);
    esp_task_wdt_add(nullptr); // subscribe the currently running (loop) task
}

void Watchdog::feed() {
    esp_task_wdt_reset();
}
