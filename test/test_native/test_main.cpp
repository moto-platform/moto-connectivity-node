#include <unity.h>
#include "../../src/SystemState.h"
#include "../../src/BLETelemetryPacket.h"

// G5.1/G5.2 -- Native (host-compiled, no ESP32/device needed) tests for the
// hardware-independent pieces of the telemetry stack: SystemState's staleness
// helper and the BLE packet's exact wire layout. Run with:
//   pio test -e native
//
// HondaCANModule's actual UDS protocol logic (request state machine, NRC
// handling, ISO-TP frame checking, bus-off recovery) is covered separately in
// test/test_can_protocol against a MockCanBus -- see that suite for those tests.

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// SystemState / isStale()
// ---------------------------------------------------------------------------

void test_system_state_initial_values(void) {
    SystemState state;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm);
    TEST_ASSERT_EQUAL_UINT8(0, state.engine.speed);
    TEST_ASSERT_EQUAL_INT16(0, state.engine.coolantTemp);
    TEST_ASSERT_FALSE(state.engine.ecuPresent);
    TEST_ASSERT_FALSE(state.telematics.phoneConnected);
}

void test_isStale_never_updated_is_always_stale(void) {
    test_setMillis(10000);
    // lastUpdateMs == 0 means "never written since boot" -- must read as stale
    // regardless of how much time has passed.
    TEST_ASSERT_TRUE(isStale(0));
}

void test_isStale_fresh_value_is_not_stale(void) {
    test_setMillis(1000);
    uint32_t lastUpdate = 900; // 100ms ago, under the 500ms default threshold
    TEST_ASSERT_FALSE(isStale(lastUpdate));
}

void test_isStale_old_value_is_stale(void) {
    test_setMillis(2000);
    uint32_t lastUpdate = 1000; // 1000ms ago, over the 500ms default threshold
    TEST_ASSERT_TRUE(isStale(lastUpdate));
}

void test_isStale_respects_custom_threshold(void) {
    test_setMillis(1000);
    uint32_t lastUpdate = 100; // 900ms ago
    TEST_ASSERT_FALSE(isStale(lastUpdate, 1000)); // under a 1000ms threshold
    TEST_ASSERT_TRUE(isStale(lastUpdate, 500));   // over a 500ms threshold
}

// ---------------------------------------------------------------------------
// BLETelemetryPacket (G3.3 -- versioned, 15-byte wire format)
// ---------------------------------------------------------------------------

void test_ble_packet_size_and_offsets(void) {
    // Must match docs/ble_telemetry_packet_schema.json exactly. A mismatch here
    // means the C++ struct, the schema, and the two mobile-app decoders have
    // silently drifted apart.
    TEST_ASSERT_EQUAL(15, sizeof(BLETelemetryPacket));
    TEST_ASSERT_EQUAL(0,  offsetof(BLETelemetryPacket, version));
    TEST_ASSERT_EQUAL(1,  offsetof(BLETelemetryPacket, seq));
    TEST_ASSERT_EQUAL(2,  offsetof(BLETelemetryPacket, rpm));
    TEST_ASSERT_EQUAL(4,  offsetof(BLETelemetryPacket, speed));
    TEST_ASSERT_EQUAL(5,  offsetof(BLETelemetryPacket, coolantTemp));
    TEST_ASSERT_EQUAL(6,  offsetof(BLETelemetryPacket, throttlePos));
    TEST_ASSERT_EQUAL(7,  offsetof(BLETelemetryPacket, batteryVolt));
    TEST_ASSERT_EQUAL(9,  offsetof(BLETelemetryPacket, leanAngle));
    TEST_ASSERT_EQUAL(11, offsetof(BLETelemetryPacket, maxLeanRight));
    TEST_ASSERT_EQUAL(13, offsetof(BLETelemetryPacket, maxLeanLeft));
}

void test_ble_packet_field_values(void) {
    BLETelemetryPacket packet;
    packet.version = BLE_PACKET_VERSION;
    packet.seq = 42;
    packet.rpm = 4500;
    packet.speed = 65;
    packet.coolantTemp = 92;
    packet.throttlePos = 45;
    packet.batteryVolt = 13800; // 13.8V
    packet.leanAngle = -125;    // -12.5 deg
    packet.maxLeanRight = 245;  // 24.5 deg
    packet.maxLeanLeft = -180;  // -18.0 deg

    TEST_ASSERT_EQUAL_UINT8(BLE_PACKET_VERSION, packet.version);
    TEST_ASSERT_EQUAL_UINT8(42, packet.seq);
    TEST_ASSERT_EQUAL_UINT16(4500, packet.rpm);
    TEST_ASSERT_EQUAL_UINT8(65, packet.speed);
    TEST_ASSERT_EQUAL_INT8(92, packet.coolantTemp);
    TEST_ASSERT_EQUAL_UINT8(45, packet.throttlePos);
    TEST_ASSERT_EQUAL_UINT16(13800, packet.batteryVolt);
    TEST_ASSERT_EQUAL_INT16(-125, packet.leanAngle);
    TEST_ASSERT_EQUAL_INT16(245, packet.maxLeanRight);
    TEST_ASSERT_EQUAL_INT16(-180, packet.maxLeanLeft);
}

void test_ble_packet_raw_byte_layout_is_little_endian(void) {
    // G1.1 -- reproduces exactly what app.js/telemetry_data.dart decode: builds a
    // packet, reinterprets it as raw bytes, and checks the multi-byte fields land
    // as little-endian, matching the schema's explicit "endianness": "little".
    BLETelemetryPacket packet;
    packet.version = 1;
    packet.seq = 0;
    packet.rpm = 0x1234;
    packet.speed = 0;
    packet.coolantTemp = 0;
    packet.throttlePos = 0;
    packet.batteryVolt = 0;
    packet.leanAngle = 0;
    packet.maxLeanRight = 0;
    packet.maxLeanLeft = 0;

    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&packet);
    // rpm lives at offset 2-3; little-endian means the low byte comes first.
    TEST_ASSERT_EQUAL_HEX8(0x34, raw[2]);
    TEST_ASSERT_EQUAL_HEX8(0x12, raw[3]);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_system_state_initial_values);
    RUN_TEST(test_isStale_never_updated_is_always_stale);
    RUN_TEST(test_isStale_fresh_value_is_not_stale);
    RUN_TEST(test_isStale_old_value_is_stale);
    RUN_TEST(test_isStale_respects_custom_threshold);
    RUN_TEST(test_ble_packet_size_and_offsets);
    RUN_TEST(test_ble_packet_field_values);
    RUN_TEST(test_ble_packet_raw_byte_layout_is_little_endian);
    return UNITY_END();
}
