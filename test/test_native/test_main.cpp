#include <unity.h>
#include <string.h>
#include "../../src/SystemState.h"
#include "../../src/BLETelemetryPacket.h"
#include "../../src/TelemetryJson.h"

// [env:native] builds no src/ and no library, so this suite pulls in the implementation
// files it needs as one translation unit (same pattern as test/test_can_protocol).
#include "../../src/TelemetryJson.cpp"
#include "../../external/moto-vehicle-defs/gen/c/conn/vehicle_cl250.c"

// G5.1/G5.2 -- Native (host-compiled, no ESP32/device needed) tests for the
// hardware-independent pieces of the telemetry stack: SystemState's staleness
// helpers, the BLE packet's exact wire layout and the Wi-Fi JSON builder. Run with:
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

void test_did_stale_threshold_comes_from_generated_table(void) {
    // Every DID uses its generated stale_after_ms, which is longer than its poll period,
    // so a value never flickers to "stale" between two regular polls.
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL_UINT32(vehicle_cl250_dids[i].stale_after_ms, didStaleThresholdMs(i));
        TEST_ASSERT_TRUE(didStaleThresholdMs(i) > vehicle_cl250_dids[i].poll_period_ms);
    }
}

// ---------------------------------------------------------------------------
// BLETelemetryPacket (G3.3 -- versioned wire format, docs/ble_telemetry_packet_schema.json v2)
// ---------------------------------------------------------------------------

void test_ble_packet_size_and_offsets(void) {
    // Must match docs/ble_telemetry_packet_schema.json exactly. A mismatch here
    // means the C++ struct, the schema, and the moto-mobile decoder have drifted apart.
    TEST_ASSERT_EQUAL(2, BLE_PACKET_VERSION);
    TEST_ASSERT_EQUAL(16, sizeof(BLETelemetryPacket));
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
    TEST_ASSERT_EQUAL(15, offsetof(BLETelemetryPacket, flags));
    TEST_ASSERT_EQUAL_HEX8(0x01, BLE_FLAG_RPM_VALID);
    TEST_ASSERT_EQUAL_HEX8(0x02, BLE_FLAG_SPEED_VALID);
    TEST_ASSERT_EQUAL_HEX8(0x04, BLE_FLAG_COOLANT_VALID);
    TEST_ASSERT_EQUAL_HEX8(0x08, BLE_FLAG_THROTTLE_VALID);
    TEST_ASSERT_EQUAL_HEX8(0x10, BLE_FLAG_BATTERY_VALID);
    TEST_ASSERT_EQUAL_HEX8(0x20, BLE_FLAG_LEAN_VALID);
    TEST_ASSERT_EQUAL_HEX8(0x40, BLE_FLAG_ECU_PRESENT);
    TEST_ASSERT_EQUAL_INT16(-32768, BLE_LEAN_NOT_AVAILABLE);
}

static SystemState freshState(unsigned long now) {
    SystemState state;
    state.engine.ecuPresent = true;
    state.engine.rpm = 4500.0f;
    state.engine.rpmUpdatedMs = now;
    state.engine.speed = 65;
    state.engine.speedUpdatedMs = now;
    state.engine.coolantTemp = 92;
    state.engine.coolantTempUpdatedMs = now;
    state.engine.throttlePos = 45.0f;
    state.engine.throttlePosUpdatedMs = now;
    state.engine.batteryVoltage = 13.8f;
    state.engine.batteryVoltageUpdatedMs = now;
    return state;
}

void test_build_packet_from_fresh_state(void) {
    test_setMillis(10000);
    BLETelemetryPacket p = buildTelemetryPacket(freshState(10000), 42);
    TEST_ASSERT_EQUAL_UINT8(BLE_PACKET_VERSION, p.version);
    TEST_ASSERT_EQUAL_UINT8(42, p.seq);
    TEST_ASSERT_EQUAL_UINT16(4500, p.rpm);
    TEST_ASSERT_EQUAL_UINT8(65, p.speed);
    TEST_ASSERT_EQUAL_INT8(92, p.coolantTemp);
    TEST_ASSERT_EQUAL_UINT8(45, p.throttlePos);
    TEST_ASSERT_UINT16_WITHIN(1, 13800, p.batteryVolt);
    // Lean has no source on this node (D-023).
    TEST_ASSERT_EQUAL_INT16(BLE_LEAN_NOT_AVAILABLE, p.leanAngle);
    TEST_ASSERT_EQUAL_INT16(BLE_LEAN_NOT_AVAILABLE, p.maxLeanRight);
    TEST_ASSERT_EQUAL_INT16(BLE_LEAN_NOT_AVAILABLE, p.maxLeanLeft);
    TEST_ASSERT_EQUAL_HEX8(0x5F, p.flags); // all five engine values + ECU present, no lean
}

void test_build_packet_flags_follow_staleness(void) {
    SystemState state = freshState(10000);
    // Just past engine speed's generated stale_after_ms: RPM is stale, vehicle speed
    // (longer stale_after_ms) is still valid.
    uint32_t rpmStale = vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms;
    TEST_ASSERT_TRUE(rpmStale < vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms);
    test_setMillis(10000 + rpmStale + 1);
    BLETelemetryPacket p = buildTelemetryPacket(state, 0);
    TEST_ASSERT_EQUAL_HEX8(0, p.flags & BLE_FLAG_RPM_VALID);
    TEST_ASSERT_EQUAL_HEX8(BLE_FLAG_SPEED_VALID, p.flags & BLE_FLAG_SPEED_VALID);
    // Never received at all -> invalid, and ECU absent.
    SystemState empty;
    p = buildTelemetryPacket(empty, 0);
    TEST_ASSERT_EQUAL_HEX8(0x00, p.flags);
}

void test_build_packet_clamps_to_field_ranges(void) {
    test_setMillis(1000);
    SystemState state = freshState(1000);
    state.engine.coolantTemp = 215; // DID maximum, above int8
    state.engine.throttlePos = 100.4f;
    state.engine.rpm = -5.0f;
    BLETelemetryPacket p = buildTelemetryPacket(state, 0);
    TEST_ASSERT_EQUAL_INT8(127, p.coolantTemp);
    TEST_ASSERT_EQUAL_UINT8(100, p.throttlePos);
    TEST_ASSERT_EQUAL_UINT16(0, p.rpm);
}

void test_ble_packet_raw_byte_layout_is_little_endian(void) {
    // G1.1 -- reinterprets a packet as raw bytes and checks the multi-byte fields land
    // little-endian, matching the schema's explicit "endianness": "little".
    test_setMillis(1000);
    SystemState state = freshState(1000);
    state.engine.rpm = (float)0x1234;
    BLETelemetryPacket packet = buildTelemetryPacket(state, 0);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&packet);
    TEST_ASSERT_EQUAL_HEX8(0x34, raw[2]);
    TEST_ASSERT_EQUAL_HEX8(0x12, raw[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, raw[9]);  // INT16_MIN little-endian: 00 80
    TEST_ASSERT_EQUAL_HEX8(0x80, raw[10]);
}

// ---------------------------------------------------------------------------
// TelemetryJson (Wi-Fi /api/telemetry, rewritten without Arduino String)
// ---------------------------------------------------------------------------

void test_json_fresh_values(void) {
    test_setMillis(10000);
    SystemState state = freshState(10000);
    state.telematics.phoneConnected = true;
    char buf[TELEMETRY_JSON_MAX_LEN];
    size_t len = buildTelemetryJson(state, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(strlen(buf), len);
    TEST_ASSERT_EQUAL_STRING(
        "{\"rpm\":4500.0,\"speed\":65,\"coolantTemp\":92,\"throttlePos\":45.0,"
        "\"batteryVoltage\":13.80,\"leanAngle\":null,\"maxLeanLeft\":null,\"maxLeanRight\":null,"
        "\"ecuPresent\":true,\"phoneConnected\":true,\"songTitle\":\"Not Connected\","
        "\"artistName\":\"N/A\"}",
        buf);
}

void test_json_stale_values_are_null(void) {
    test_setMillis(10000);
    SystemState state;
    char buf[TELEMETRY_JSON_MAX_LEN];
    buildTelemetryJson(state, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"rpm\":null,\"speed\":null,\"coolantTemp\":null,"
                                     "\"throttlePos\":null,\"batteryVoltage\":null,"));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"ecuPresent\":false"));
}

void test_json_escapes_phone_strings_and_fits_worst_case(void) {
    test_setMillis(10000);
    SystemState state = freshState(10000);
    // G4.3: quotes/backslashes escaped, control and non-ASCII bytes dropped.
    strcpy(state.telematics.songTitle, "a\"b\\c\x01\xff");
    char buf[TELEMETRY_JSON_MAX_LEN];
    buildTelemetryJson(state, buf, sizeof(buf));
    TEST_ASSERT_NOT_NULL(strstr(buf, "\"songTitle\":\"a\\\"b\\\\c\""));
    // Worst case: both strings full of characters that need escaping.
    memset(state.telematics.songTitle, '"', sizeof(state.telematics.songTitle) - 1);
    memset(state.telematics.artistName, '\\', sizeof(state.telematics.artistName) - 1);
    state.engine.rpm = 16383.75f;
    TEST_ASSERT_TRUE(buildTelemetryJson(state, buf, sizeof(buf)) > 0);
}

void test_json_reports_overflow_instead_of_truncating(void) {
    test_setMillis(10000);
    SystemState state = freshState(10000);
    char small[40];
    TEST_ASSERT_EQUAL(0, buildTelemetryJson(state, small, sizeof(small)));
    TEST_ASSERT_EQUAL_STRING("", small);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_system_state_initial_values);
    RUN_TEST(test_isStale_never_updated_is_always_stale);
    RUN_TEST(test_isStale_fresh_value_is_not_stale);
    RUN_TEST(test_isStale_old_value_is_stale);
    RUN_TEST(test_isStale_respects_custom_threshold);
    RUN_TEST(test_did_stale_threshold_comes_from_generated_table);
    RUN_TEST(test_ble_packet_size_and_offsets);
    RUN_TEST(test_build_packet_from_fresh_state);
    RUN_TEST(test_build_packet_flags_follow_staleness);
    RUN_TEST(test_build_packet_clamps_to_field_ranges);
    RUN_TEST(test_ble_packet_raw_byte_layout_is_little_endian);
    RUN_TEST(test_json_fresh_values);
    RUN_TEST(test_json_stale_values_are_null);
    RUN_TEST(test_json_escapes_phone_strings_and_fits_worst_case);
    RUN_TEST(test_json_reports_overflow_instead_of_truncating);
    return UNITY_END();
}
