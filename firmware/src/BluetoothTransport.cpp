#include "BluetoothTransport.h"
#include <BluetoothSerial.h>

namespace {
BluetoothSerial gBtSerial;
}

void BluetoothTransport::begin(const char* deviceName) {
    gBtSerial.begin(deviceName);
}

bool BluetoothTransport::hasClient() const {
    return gBtSerial.hasClient();
}

int BluetoothTransport::available() {
    return gBtSerial.available();
}

int BluetoothTransport::readByte() {
    if (gBtSerial.available() <= 0) return -1;
    return gBtSerial.read();
}

size_t BluetoothTransport::write(const uint8_t* data, size_t len) {
    return gBtSerial.write(data, len);
}
