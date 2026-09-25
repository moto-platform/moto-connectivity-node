#include <unity.h>
#include "SystemState.h"
#include "BLETelemetryPacket.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_system_state_initialization(void) {
    SystemState state;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm);
    TEST_ASSERT_EQUAL_UINT8(0, state.engine.speed);
    TEST_ASSERT_EQUAL_INT16(0, state.engine.coolantTemp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.throttlePos);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.batteryVoltage);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.dynamics.leanAngle);
    TEST_ASSERT_FALSE(state.telematics.phoneConnected);
}

void test_ble_telemetry_packet_packing(void) {
    // G3.3 -- this used to assert 12 bytes with no version/seq fields; the packet
    // grew to 15 bytes when those were added (see docs/ble_telemetry_packet_schema.json).
    // This stale assertion would have failed against current firmware.
    BLETelemetryPacket packet;
    packet.version = BLE_PACKET_VERSION;
    packet.seq = 7;
    packet.rpm = 4500;
    packet.speed = 65;
    packet.coolantTemp = 92;
    packet.throttlePos = 45;
    packet.batteryVolt = 13800; // 13.8V
    packet.leanAngle = -125;   // -12.5 deg
    packet.maxLeanRight = 245;  // 24.5 deg
    packet.maxLeanLeft = -180;  // -18.0 deg

    // Total size of BLE telemetry packet MUST be exactly 15 bytes
    TEST_ASSERT_EQUAL(15, sizeof(BLETelemetryPacket));
    TEST_ASSERT_EQUAL_UINT8(BLE_PACKET_VERSION, packet.version);
    TEST_ASSERT_EQUAL_UINT8(7, packet.seq);
    TEST_ASSERT_EQUAL_UINT16(4500, packet.rpm);
    TEST_ASSERT_EQUAL_UINT8(65, packet.speed);
    TEST_ASSERT_EQUAL_INT8(92, packet.coolantTemp);
    TEST_ASSERT_EQUAL_INT16(-125, packet.leanAngle);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_system_state_initialization);
    RUN_TEST(test_ble_telemetry_packet_packing);
    UNITY_END();
}

void loop() {
    // Unit test completes in setup()
}
