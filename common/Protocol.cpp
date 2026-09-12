#include "Protocol.h"
#include <cstring>

namespace rover_proto {

uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return crc;
}

namespace {
// Appends `b` to out[0..outLen), escaping it first if it collides with a framing byte (START/END/ESCAPE). Returns false if it would overflow outCapacity.
bool stuffByte(uint8_t b, uint8_t* out, size_t& outLen, size_t outCapacity) {
    const bool needsEscape = (b == START_BYTE || b == END_BYTE || b == ESCAPE_BYTE);
    if (needsEscape) {
        if (outLen + 2 > outCapacity) return false;
        out[outLen++] = ESCAPE_BYTE;
        out[outLen++] = static_cast<uint8_t>(b ^ ESCAPE_XOR);
    } else {
        if (outLen + 1 > outCapacity) return false;
        out[outLen++] = b;
    }
    return true;
}
} // namespace

size_t encodeFrame(uint8_t cmd, const uint8_t* payload, uint8_t payloadLen,
                    uint8_t* out, size_t outCapacity) {
    if (payloadLen > MAX_PAYLOAD_BYTES) return 0;

    // CRC covers cmd + payload only (never the length byte or the framing bytes).
    uint8_t crcBuf[1 + MAX_PAYLOAD_BYTES];
    crcBuf[0] = cmd;
    if (payloadLen > 0) memcpy(&crcBuf[1], payload, payloadLen);
    const uint8_t crc = crc8(crcBuf, static_cast<size_t>(1) + payloadLen);

    size_t outLen = 0;
    if (outLen + 1 > outCapacity) return 0;
    out[outLen++] = START_BYTE; // delimiter itself - never stuffed

    if (!stuffByte(payloadLen, out, outLen, outCapacity)) return 0;
    if (!stuffByte(cmd, out, outLen, outCapacity)) return 0;
    for (uint8_t i = 0; i < payloadLen; ++i) {
        if (!stuffByte(payload[i], out, outLen, outCapacity)) return 0;
    }
    if (!stuffByte(crc, out, outLen, outCapacity)) return 0;

    if (outLen + 1 > outCapacity) return 0;
    out[outLen++] = END_BYTE; // delimiter itself - never stuffed

    return outLen;
}

void FrameParser::reset() {
    state_ = State::WAIT_START;
    escaping_ = false;
    bufIndex_ = 0;
}

bool FrameParser::feed(uint8_t byte, Packet& result) {
    // a fresh START always (re)synchronizes the parser, even mid-frame.
    if (!escaping_ && byte == START_BYTE) {
        reset();
        state_ = State::READ_DATA;
        return false;
    }

    if (state_ == State::WAIT_START) {
        return false; // ignore any noise before the first START
    }

    if (!escaping_ && byte == ESCAPE_BYTE) {
        escaping_ = true;
        return false;
    }

    if (!escaping_ && byte == END_BYTE) {
        // Frame complete. buffer_ holds unstuffed: [LEN][CMD][PAYLOAD...][CRC]
        if (bufIndex_ < 2) { // need at minimum LEN + CMD + CRC = 3 bytes; <2 is certainly broken
            framingErrors_++;
            reset();
            return false;
        }
        const uint8_t declaredLen = buffer_[0];
        const size_t expectedTotal = 1 /*LEN*/ + 1 /*CMD*/ + declaredLen + 1 /*CRC*/;
        if (declaredLen > MAX_PAYLOAD_BYTES || bufIndex_ != expectedTotal) {
            framingErrors_++;
            reset();
            return false;
        }

        const uint8_t cmd = buffer_[1];
        const uint8_t* payload = &buffer_[2];
        const uint8_t receivedCrc = buffer_[2 + declaredLen];

        uint8_t crcBuf[1 + MAX_PAYLOAD_BYTES];
        crcBuf[0] = cmd;
        if (declaredLen > 0) memcpy(&crcBuf[1], payload, declaredLen);
        const uint8_t computedCrc = crc8(crcBuf, static_cast<size_t>(1) + declaredLen);

        reset();

        if (computedCrc != receivedCrc) {
            crcErrors_++;
            // the sender's failsafe/heartbeat logic handles recovery, so we don't need a NACK for a frame we can't trust the CMD field of in the first place.
            return false;
        }

        result.cmd = cmd;
        result.length = declaredLen;
        if (declaredLen > 0) memcpy(result.payload, payload, declaredLen);
        return true;
    }

    // Ordinary data byte (possibly the second half of an escape sequence).
    uint8_t actual = byte;
    if (escaping_) {
        actual = static_cast<uint8_t>(byte ^ ESCAPE_XOR);
        escaping_ = false;
    }

    if (bufIndex_ >= sizeof(buffer_)) {
        // Oversized/malformed frame -> resync and wait for the next START.
        framingErrors_++;
        reset();
        return false;
    }
    buffer_[bufIndex_++] = actual;
    return false;
}

} // namespace rover_proto
