#include "CommandProcessor.h"
#include "config.h"
#include <cstring>

using namespace rover_proto;

void CommandProcessor::begin(DriveController* drive, BatteryMonitor* battery) {
    drive_ = drive;
    battery_ = battery;
    transport_.begin(BLUETOOTH_DEVICE_NAME);
}

void CommandProcessor::poll(uint32_t nowMs) {
    int available = transport_.available();
    // Bound per-call work so a burst of bytes can't stall the control loop.
    const int MAX_BYTES_PER_POLL = 64;
    int processed = 0;

    while (available-- > 0 && processed++ < MAX_BYTES_PER_POLL) {
        const int b = transport_.readByte();
        if (b < 0) break;

        Packet pkt;
        if (parser_.feed(static_cast<uint8_t>(b), pkt)) {
            lastValidPacketMs_ = nowMs;
            everConnected_ = true;
            handlePacket(pkt);
        }
    }
}

bool CommandProcessor::isCommLost(uint32_t nowMs, uint32_t timeoutMs) const {
    if (!everConnected_) return false;
    return (nowMs - lastValidPacketMs_) > timeoutMs;
}

void CommandProcessor::handlePacket(const Packet& pkt) {
    switch (static_cast<CommandId>(pkt.cmd)) {

        case CommandId::SET_VELOCITY: {
            if (pkt.length != sizeof(float) * 2) {
                sendNack(pkt.cmd, NackReason::BAD_LENGTH);
                return;
            }
            float linear, angular;
            memcpy(&linear, &pkt.payload[0], sizeof(float));
            memcpy(&angular, &pkt.payload[4], sizeof(float));

            if (linear < -MAX_LINEAR_MPS || linear > MAX_LINEAR_MPS ||
                angular < -MAX_ANGULAR_RADS || angular > MAX_ANGULAR_RADS) {
                sendNack(pkt.cmd, NackReason::OUT_OF_RANGE);
                return;
            }
            drive_->setVelocityCommand(linear, angular);
            sendAck(pkt.cmd);
            break;
        }

        case CommandId::STOP: {
            drive_->stop();
            sendAck(pkt.cmd);
            break;
        }

        case CommandId::HEARTBEAT: {
            // No-op besides refreshing lastValidPacketMs_, already done in poll().
            sendAck(pkt.cmd);
            break;
        }

        case CommandId::SET_PID_GAINS: {
            if (pkt.length != sizeof(uint8_t) + sizeof(float) * 3) {
                sendNack(pkt.cmd, NackReason::BAD_LENGTH);
                return;
            }
            const uint8_t wheelIdx = pkt.payload[0];
            float kp, ki, kd;
            memcpy(&kp, &pkt.payload[1], sizeof(float));
            memcpy(&ki, &pkt.payload[5], sizeof(float));
            memcpy(&kd, &pkt.payload[9], sizeof(float));

            if (wheelIdx >= WHEEL_COUNT) {
                sendNack(pkt.cmd, NackReason::OUT_OF_RANGE);
                return;
            }
            drive_->setPIDGains(wheelIdx, kp, ki, kd);
            sendAck(pkt.cmd);
            break;
        }

        case CommandId::TELEMETRY_REQUEST: {
            // ack receipt of sendTelemetry()
            sendAck(pkt.cmd);
            break;
        }

        case CommandId::SET_COMP_MODE: {
            if (pkt.length != 1) {
                sendNack(pkt.cmd, NackReason::BAD_LENGTH);
                return;
            }
            battery_->setEnabled(pkt.payload[0] != 0);
            sendAck(pkt.cmd);
            break;
        }

        default:
            sendNack(pkt.cmd, NackReason::UNKNOWN_CMD);
            break;
    }
}

void CommandProcessor::sendAck(uint8_t originalCmd) {
    uint8_t payload[1] = { originalCmd };
    uint8_t frame[8];
    const size_t n = encodeFrame(static_cast<uint8_t>(CommandId::ACK), payload, 1, frame, sizeof(frame));
    if (n > 0) transport_.write(frame, n);
}

void CommandProcessor::sendNack(uint8_t originalCmd, NackReason reason) {
    uint8_t payload[2] = { originalCmd, static_cast<uint8_t>(reason) };
    uint8_t frame[8];
    const size_t n = encodeFrame(static_cast<uint8_t>(CommandId::NACK), payload, 2, frame, sizeof(frame));
    if (n > 0) transport_.write(frame, n);
}

void CommandProcessor::sendTelemetry(uint32_t nowMs) {
    // Packed telemetry payload layout (60 bytes):
    //   uint32 nowMs, float batteryVoltage, float compFactor,
    //   then per wheel (FL, RL, FR, RR): float target, float actual, float cmdOut
    uint8_t payload[60];
    size_t off = 0;

    memcpy(&payload[off], &nowMs, sizeof(uint32_t)); off += 4;

    const float voltage = battery_->voltage();
    memcpy(&payload[off], &voltage, sizeof(float)); off += 4;

    const float comp = battery_->compensationFactor(BATTERY_NOMINAL_V, COMP_FACTOR_MIN, COMP_FACTOR_MAX);
    memcpy(&payload[off], &comp, sizeof(float)); off += 4;

    for (int i = 0; i < WHEEL_COUNT; ++i) {
        const WheelTelemetry& t = drive_->telemetry(i);
        memcpy(&payload[off], &t.targetRadPerSec, sizeof(float)); off += 4;
        memcpy(&payload[off], &t.actualRadPerSec, sizeof(float)); off += 4;
        memcpy(&payload[off], &t.commandOut, sizeof(float)); off += 4;
    }

    uint8_t frame[80];
    const size_t n = encodeFrame(static_cast<uint8_t>(CommandId::TELEMETRY_DATA), payload,
                                  static_cast<uint8_t>(off), frame, sizeof(frame));
    if (n > 0) transport_.write(frame, n);
}
