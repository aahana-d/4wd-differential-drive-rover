#pragma once
#include <cstdint>
#include <cstddef>

// Thin wrapper around the ESP32 classic-Bluetooth SPP library
// Keeps the dependency surface small and makes it easy to swap in BLE or a wired UART later without touching CommandProcessor.

class BluetoothTransport {
public:
    void begin(const char* deviceName);
    bool hasClient() const;
    int available();
    int readByte();               // returns -1 if nothing available
    size_t write(const uint8_t* data, size_t len);
};
