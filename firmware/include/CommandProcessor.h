#pragma once
#include "Protocol.h"
#include "BluetoothTransport.h"
#include "DriveController.h"
#include "BatteryMonitor.h"
#include <cstdint>

// Owns the transport + streaming frame parser, validates every inbound command, dispatches valid ones to DriveController/BatteryMonitor, and tracks the last-good-packet timestamp used by the communication-loss failsafe (see isCommLost()).

class CommandProcessor {
public:
    void begin(DriveController* drive, BatteryMonitor* battery);

    // Call every loop() iteration: drains available bytes, parses frames, validates + dispatches
    void poll(uint32_t nowMs);

    bool isCommLost(uint32_t nowMs, uint32_t timeoutMs) const;

    // Builds and transmits a TELEMETRY_DATA frame from the current DriveController + BatteryMonitor state.
    void sendTelemetry(uint32_t nowMs);

private:
    void handlePacket(const rover_proto::Packet& pkt);
    void sendAck(uint8_t originalCmd);
    void sendNack(uint8_t originalCmd, rover_proto::NackReason reason);

    BluetoothTransport transport_;
    rover_proto::FrameParser parser_;
    DriveController* drive_ = nullptr;
    BatteryMonitor* battery_ = nullptr;

    uint32_t lastValidPacketMs_ = 0;
    bool everConnected_ = false; // avoid tripping the failsafe before first contact
};
