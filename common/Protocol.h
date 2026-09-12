#pragma once
// Platform-independent packet framing for the rover control link, used by the embedded firmware (ESP32) and the host-side basestation telemetry logger

#include <cstdint>
#include <cstddef>

namespace rover_proto {

// ---- Framing constants --------------------------------------------------
// Frame layout on the wire (after byte-stuffing is undone):
//   START | LEN | CMD | PAYLOAD[0..LEN-1] | CRC | END
// LEN, CMD, PAYLOAD and CRC are all byte-stuffed
// START/END never are, which is what lets the parser resynchronize on any START byte even if a previous frame was corrupted or truncated.
constexpr uint8_t START_BYTE  = 0x7E;
constexpr uint8_t END_BYTE    = 0x7F;
constexpr uint8_t ESCAPE_BYTE = 0x7D;
constexpr uint8_t ESCAPE_XOR  = 0x20;

constexpr size_t MAX_PAYLOAD_BYTES = 32;

// ---- Command IDs ----------------------------------------------------------
enum class CommandId : uint8_t {
    SET_VELOCITY      = 0x01, // payload: float linear_mps, float angular_radps
    STOP              = 0x02, // payload: none
    HEARTBEAT         = 0x03, // payload: none
    SET_PID_GAINS     = 0x04, // payload: uint8 wheel_idx, float kp, float ki, float kd
    TELEMETRY_REQUEST = 0x05, // payload: none
    TELEMETRY_DATA    = 0x06, // payload: packed telemetry (see CommandProcessor::sendTelemetry)
    ACK               = 0x07, // payload: uint8 original_cmd
    NACK              = 0x08, // payload: uint8 original_cmd, uint8 reason_code
    SET_COMP_MODE     = 0x09  // payload: uint8 (0=off,1=on) - voltage-compensation A/B toggle
};

enum class NackReason : uint8_t {
    BAD_LENGTH   = 0x01,
    OUT_OF_RANGE = 0x02,
    BAD_CRC      = 0x03, // informational only; a bad-CRC frame is dropped, not NACKed
    UNKNOWN_CMD  = 0x04,
    COMM_TIMEOUT_RECOVERY = 0x05
};

// fully decoded, CRC-verified packet
struct Packet {
    uint8_t cmd = 0;
    uint8_t payload[MAX_PAYLOAD_BYTES] = {0};
    uint8_t length = 0; // number of valid bytes in payload
};

// CRC-8 (Dallas/Maxim polynomial 0x31, init 0x00), computed over CMD+PAYLOAD.
uint8_t crc8(const uint8_t* data, size_t len);

// Encodes cmd+payload into a fully framed, byte-stuffed buffer ready to transmit. Returns the number of bytes written, or 0 if it doesn't fit.
size_t encodeFrame(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen,
                    uint8_t* out, size_t outCapacity);

// Streaming decoder: feed raw bytes one at a time as they arrive from Bluetooth/serial. Returns true exactly once a complete, CRC-valid frame has been assembled; the decoded frame is left in `result`.
// Any START_BYTE resynchronizes the parser, so noise/dropped bytes on the link cannot permanently wedge it.
class FrameParser {
public:
    bool feed(uint8_t byte, Packet& result);

    uint32_t crcErrorCount() const { return crcErrors_; }
    uint32_t framingErrorCount() const { return framingErrors_; }

private:
    enum class State { WAIT_START, READ_DATA };

    State state_ = State::WAIT_START;
    bool escaping_ = false;

    // Holds unstuffed bytes for the frame currently being assembled:
    // [LEN][CMD][PAYLOAD...][CRC]
    uint8_t buffer_[MAX_PAYLOAD_BYTES + 3];
    size_t bufIndex_ = 0;

    uint32_t crcErrors_ = 0;
    uint32_t framingErrors_ = 0;

    void reset();
};

} // namespace rover_proto
