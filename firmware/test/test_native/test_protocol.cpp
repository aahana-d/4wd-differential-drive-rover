// Host-run (no hardware) unit tests for the packet framing/CRC layer.

// Run with: pio test -e native -f test_protocol

#include <unity.h>
#include "Protocol.h"
#include <cstring>

using namespace rover_proto;

void test_encode_decode_roundtrip_no_special_bytes() {
    uint8_t payload[4] = {0x01, 0x02, 0x03, 0x04};
    uint8_t frame[16];
    const size_t n = encodeFrame(0x10, payload, 4, frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, n);

    FrameParser parser;
    Packet result;
    bool gotPacket = false;
    for (size_t i = 0; i < n; ++i) {
        if (parser.feed(frame[i], result)) gotPacket = true;
    }
    TEST_ASSERT_TRUE(gotPacket);
    TEST_ASSERT_EQUAL_UINT8(0x10, result.cmd);
    TEST_ASSERT_EQUAL_UINT8(4, result.length);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, result.payload, 4);
}

void test_encode_decode_roundtrip_with_escaped_bytes() {
    // Deliberately include bytes equal to START/END/ESCAPE in the payload to exercise the byte-stuffing logic end-to-end.
    uint8_t payload[3] = { START_BYTE, END_BYTE, ESCAPE_BYTE };
    uint8_t frame[16];
    const size_t n = encodeFrame(0x20, payload, 3, frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, n);

    FrameParser parser;
    Packet result;
    bool gotPacket = false;
    for (size_t i = 0; i < n; ++i) {
        if (parser.feed(frame[i], result)) gotPacket = true;
    }
    TEST_ASSERT_TRUE(gotPacket);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, result.payload, 3);
}

void test_corrupted_payload_is_rejected_by_crc() {
    uint8_t payload[2] = {0xAA, 0xBB};
    uint8_t frame[16];
    const size_t n = encodeFrame(0x30, payload, 2, frame, sizeof(frame));
    TEST_ASSERT_GREATER_THAN(0, n);

    // Flip a bit inside the payload (index 3 = first payload byte, after START, LEN, CMD) to simulate line noise; CRC should catch it.
    frame[3] ^= 0xFF;

    FrameParser parser;
    Packet result;
    bool gotPacket = false;
    for (size_t i = 0; i < n; ++i) {
        if (parser.feed(frame[i], result)) gotPacket = true;
    }
    TEST_ASSERT_FALSE(gotPacket);
    TEST_ASSERT_EQUAL_UINT32(1, parser.crcErrorCount());
}

void test_truncated_frame_is_ignored() {
    uint8_t payload[2] = {0x01, 0x02};
    uint8_t frame[16];
    const size_t n = encodeFrame(0x40, payload, 2, frame, sizeof(frame));

    FrameParser parser;
    Packet result;
    bool gotPacket = false;
    // Feed everything except the final END_BYTE - the frame must never complete.
    for (size_t i = 0; i + 1 < n; ++i) {
        if (parser.feed(frame[i], result)) gotPacket = true;
    }
    TEST_ASSERT_FALSE(gotPacket);
}

void test_resync_after_garbage_bytes() {
    uint8_t payload[1] = {0x7A};
    uint8_t frame[16];
    const size_t n = encodeFrame(0x50, payload, 1, frame, sizeof(frame));

    FrameParser parser;
    Packet result;
    bool gotPacket = false;

    // Noise before a valid frame must not prevent that frame from decoding once a real START_BYTE shows up.
    const uint8_t noise[3] = {0x00, 0xFF, 0x11};
    for (uint8_t b : noise) parser.feed(b, result);
    for (size_t i = 0; i < n; ++i) {
        if (parser.feed(frame[i], result)) gotPacket = true;
    }
    TEST_ASSERT_TRUE(gotPacket);
    TEST_ASSERT_EQUAL_UINT8(0x50, result.cmd);
}

void setUp() {}
void tearDown() {}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_encode_decode_roundtrip_no_special_bytes);
    RUN_TEST(test_encode_decode_roundtrip_with_escaped_bytes);
    RUN_TEST(test_corrupted_payload_is_rejected_by_crc);
    RUN_TEST(test_truncated_frame_is_ignored);
    RUN_TEST(test_resync_after_garbage_bytes);
    return UNITY_END();
}
