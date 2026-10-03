#include <unity.h>
#include <string.h>
#include <thread>
#include "../../src/SystemState.h"
#include "../../src/BLETelemetryPacket.h"
#include "../../src/ImuBlockPacket.h"
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
// BLETelemetryPacketV2 (G3.3 -- low-MTU fallback, schema `lowMtuFallback`)
// ---------------------------------------------------------------------------

void test_ble_packet_size_and_offsets(void) {
    // Must match docs/ble_telemetry_packet_schema.json exactly. A mismatch here
    // means the C++ struct, the schema, and the moto-mobile decoder have drifted apart.
    TEST_ASSERT_EQUAL(2, BLE_PACKET_VERSION_LEGACY);
    TEST_ASSERT_EQUAL(16, sizeof(BLETelemetryPacketV2));
    TEST_ASSERT_EQUAL(0,  offsetof(BLETelemetryPacketV2, version));
    TEST_ASSERT_EQUAL(1,  offsetof(BLETelemetryPacketV2, seq));
    TEST_ASSERT_EQUAL(2,  offsetof(BLETelemetryPacketV2, rpm));
    TEST_ASSERT_EQUAL(4,  offsetof(BLETelemetryPacketV2, speed));
    TEST_ASSERT_EQUAL(5,  offsetof(BLETelemetryPacketV2, coolantTemp));
    TEST_ASSERT_EQUAL(6,  offsetof(BLETelemetryPacketV2, throttlePos));
    TEST_ASSERT_EQUAL(7,  offsetof(BLETelemetryPacketV2, batteryVolt));
    TEST_ASSERT_EQUAL(9,  offsetof(BLETelemetryPacketV2, leanAngle));
    TEST_ASSERT_EQUAL(11, offsetof(BLETelemetryPacketV2, maxLeanRight));
    TEST_ASSERT_EQUAL(13, offsetof(BLETelemetryPacketV2, maxLeanLeft));
    TEST_ASSERT_EQUAL(15, offsetof(BLETelemetryPacketV2, flags));
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
    BLETelemetryPacketV2 p = buildTelemetryPacketV2(freshState(10000), 42);
    TEST_ASSERT_EQUAL_UINT8(BLE_PACKET_VERSION_LEGACY, p.version);
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
    // Just past vehicle speed's generated stale_after_ms: speed is stale, engine speed
    // (longer stale_after_ms since D-053: 300 vs 330 ms) is still valid.
    uint32_t speedStale = vehicle_cl250_dids[VEHICLE_CL250_IDX_VEHICLE_SPEED].stale_after_ms;
    TEST_ASSERT_TRUE(speedStale < vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms);
    test_setMillis(10000 + speedStale + 1);
    BLETelemetryPacketV2 p = buildTelemetryPacketV2(state, 0);
    TEST_ASSERT_EQUAL_HEX8(0, p.flags & BLE_FLAG_SPEED_VALID);
    TEST_ASSERT_EQUAL_HEX8(BLE_FLAG_RPM_VALID, p.flags & BLE_FLAG_RPM_VALID);
    // Never received at all -> invalid, and ECU absent.
    SystemState empty;
    p = buildTelemetryPacketV2(empty, 0);
    TEST_ASSERT_EQUAL_HEX8(0x00, p.flags);
}

void test_build_packet_clamps_to_field_ranges(void) {
    test_setMillis(1000);
    SystemState state = freshState(1000);
    state.engine.coolantTemp = 215; // DID maximum, above int8
    state.engine.throttlePos = 100.4f;
    state.engine.rpm = -5.0f;
    BLETelemetryPacketV2 p = buildTelemetryPacketV2(state, 0);
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
    BLETelemetryPacketV2 packet = buildTelemetryPacketV2(state, 0);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&packet);
    TEST_ASSERT_EQUAL_HEX8(0x34, raw[2]);
    TEST_ASSERT_EQUAL_HEX8(0x12, raw[3]);
    TEST_ASSERT_EQUAL_HEX8(0x00, raw[9]);  // INT16_MIN little-endian: 00 80
    TEST_ASSERT_EQUAL_HEX8(0x80, raw[10]);
}

// ---------------------------------------------------------------------------
// BLETelemetryPacketV3 (D-032 -- schema top-level `fields`)
// ---------------------------------------------------------------------------

void test_ble_v3_size_and_offsets(void) {
    TEST_ASSERT_EQUAL(3, BLE_PACKET_VERSION);
    TEST_ASSERT_EQUAL(37, sizeof(BLETelemetryPacketV3));
    TEST_ASSERT_EQUAL(0,  offsetof(BLETelemetryPacketV3, version));
    TEST_ASSERT_EQUAL(1,  offsetof(BLETelemetryPacketV3, seq));
    TEST_ASSERT_EQUAL(2,  offsetof(BLETelemetryPacketV3, deviceTimeMs));
    TEST_ASSERT_EQUAL(6,  offsetof(BLETelemetryPacketV3, rpm));
    TEST_ASSERT_EQUAL(8,  offsetof(BLETelemetryPacketV3, speed));
    TEST_ASSERT_EQUAL(9,  offsetof(BLETelemetryPacketV3, coolantTemp));
    TEST_ASSERT_EQUAL(10, offsetof(BLETelemetryPacketV3, throttlePos));
    TEST_ASSERT_EQUAL(11, offsetof(BLETelemetryPacketV3, batteryVolt));
    TEST_ASSERT_EQUAL(13, offsetof(BLETelemetryPacketV3, leanAngle));
    TEST_ASSERT_EQUAL(15, offsetof(BLETelemetryPacketV3, maxLeanRight));
    TEST_ASSERT_EQUAL(17, offsetof(BLETelemetryPacketV3, maxLeanLeft));
    TEST_ASSERT_EQUAL(19, offsetof(BLETelemetryPacketV3, flags));
    TEST_ASSERT_EQUAL(20, offsetof(BLETelemetryPacketV3, rpmAgeMs));
    TEST_ASSERT_EQUAL(22, offsetof(BLETelemetryPacketV3, speedAgeMs));
    TEST_ASSERT_EQUAL(24, offsetof(BLETelemetryPacketV3, coolantTempAgeMs));
    TEST_ASSERT_EQUAL(26, offsetof(BLETelemetryPacketV3, throttlePosAgeMs));
    TEST_ASSERT_EQUAL(28, offsetof(BLETelemetryPacketV3, batteryVoltAgeMs));
    TEST_ASSERT_EQUAL(30, offsetof(BLETelemetryPacketV3, canBusState));
    TEST_ASSERT_EQUAL(31, offsetof(BLETelemetryPacketV3, canTxErrorCount));
    TEST_ASSERT_EQUAL(32, offsetof(BLETelemetryPacketV3, canRxErrorCount));
    TEST_ASSERT_EQUAL(33, offsetof(BLETelemetryPacketV3, canBusOffCount));
    TEST_ASSERT_EQUAL(34, offsetof(BLETelemetryPacketV3, unansweredDidCount));
    TEST_ASSERT_EQUAL(36, offsetof(BLETelemetryPacketV3, canFlags));
    TEST_ASSERT_EQUAL_HEX8(0x80, BLE_FLAG_IMU_ACTIVE);
    TEST_ASSERT_EQUAL(0, (int)CanHealthState::NOT_INSTALLED);
    TEST_ASSERT_EQUAL(1, (int)CanHealthState::RUNNING);
    TEST_ASSERT_EQUAL(2, (int)CanHealthState::ERROR_WARNING);
    TEST_ASSERT_EQUAL(3, (int)CanHealthState::BUS_OFF);
    TEST_ASSERT_EQUAL(4, (int)CanHealthState::STOPPED);
    TEST_ASSERT_EQUAL_HEX8(0x01, CAN_HEALTH_FLAG_POLLER_ENABLED);
    TEST_ASSERT_EQUAL_HEX8(0x02, CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER);
    TEST_ASSERT_EQUAL_HEX8(0x04, CAN_HEALTH_FLAG_LATCHED_BUS_OFF);
    TEST_ASSERT_EQUAL_HEX8(0x08, CAN_HEALTH_FLAG_SYNTHETIC_DATA);
}

void test_build_v3_packet_values_ages_and_health(void) {
    SystemState state = freshState(10000);
    state.engine.speedUpdatedMs = 9950;       // 50 ms old at build time
    state.engine.coolantTempUpdatedMs = 0;    // never received
    state.imu.active = true;
    state.can.busState = CanHealthState::RUNNING;
    state.can.txErrorCount = 300;             // saturates to 255
    state.can.rxErrorCount = 7;
    state.can.busOffCount = 2;
    state.can.unansweredDidCount = 70000;     // saturates to 65535
    state.can.flags = CAN_HEALTH_FLAG_POLLER_ENABLED;
    test_setMillis(10000);
    BLETelemetryPacketV3 p = buildTelemetryPacketV3(state, 9, 10000);
    TEST_ASSERT_EQUAL_UINT8(3, p.version);
    TEST_ASSERT_EQUAL_UINT8(9, p.seq);
    TEST_ASSERT_EQUAL_UINT32(10000, p.deviceTimeMs);
    TEST_ASSERT_EQUAL_UINT16(4500, p.rpm);
    TEST_ASSERT_EQUAL_UINT8(65, p.speed);
    TEST_ASSERT_UINT16_WITHIN(1, 13800, p.batteryVolt);
    TEST_ASSERT_EQUAL_INT16(BLE_LEAN_NOT_AVAILABLE, p.leanAngle);
    TEST_ASSERT_EQUAL_INT16(BLE_LEAN_NOT_AVAILABLE, p.maxLeanRight);
    TEST_ASSERT_EQUAL_INT16(BLE_LEAN_NOT_AVAILABLE, p.maxLeanLeft);
    // rpm, speed, tps, battery valid + ECU present + IMU active; coolant never received.
    TEST_ASSERT_EQUAL_HEX8(0x01 | 0x02 | 0x08 | 0x10 | 0x40 | 0x80, p.flags);
    TEST_ASSERT_EQUAL_UINT16(0, p.rpmAgeMs);
    TEST_ASSERT_EQUAL_UINT16(50, p.speedAgeMs);
    TEST_ASSERT_EQUAL_UINT16(BLE_AGE_NEVER_RECEIVED, p.coolantTempAgeMs);
    TEST_ASSERT_EQUAL_UINT8(1, p.canBusState);
    TEST_ASSERT_EQUAL_UINT8(255, p.canTxErrorCount);
    TEST_ASSERT_EQUAL_UINT8(7, p.canRxErrorCount);
    TEST_ASSERT_EQUAL_UINT8(2, p.canBusOffCount);
    TEST_ASSERT_EQUAL_UINT16(65535, p.unansweredDidCount);
    TEST_ASSERT_EQUAL_HEX8(0x01, p.canFlags);
}

void test_v3_age_saturates_and_valid_bit_follows_generated_stale_limit(void) {
    SystemState state = freshState(1000);
    uint32_t rpmStale = vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].stale_after_ms;
    test_setMillis(1000 + rpmStale + 1);
    BLETelemetryPacketV3 p = buildTelemetryPacketV3(state, 0, 1000 + rpmStale + 1);
    TEST_ASSERT_EQUAL_UINT16(rpmStale + 1, p.rpmAgeMs);
    TEST_ASSERT_EQUAL_HEX8(0, p.flags & BLE_FLAG_RPM_VALID);
    // Very old value: age saturates below the "never received" marker.
    test_setMillis(1000 + 100000);
    p = buildTelemetryPacketV3(state, 0, 1000 + 100000);
    TEST_ASSERT_EQUAL_UINT16(BLE_AGE_MAX_MS, p.rpmAgeMs);
    // millis() wrap: age is still the unsigned difference.
    TEST_ASSERT_EQUAL_UINT16(20, signalAgeMs(0xFFFFFFF0u, 4));
}

void test_v3_raw_bytes_are_little_endian(void) {
    SystemState state = freshState(1000);
    test_setMillis(1000);
    BLETelemetryPacketV3 p = buildTelemetryPacketV3(state, 0, 0x11223344u);
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&p);
    TEST_ASSERT_EQUAL_HEX8(0x44, raw[2]);
    TEST_ASSERT_EQUAL_HEX8(0x33, raw[3]);
    TEST_ASSERT_EQUAL_HEX8(0x22, raw[4]);
    TEST_ASSERT_EQUAL_HEX8(0x11, raw[5]);
    TEST_ASSERT_EQUAL_HEX8(0x94, raw[6]); // 4500 = 0x1194
    TEST_ASSERT_EQUAL_HEX8(0x11, raw[7]);
}

void test_telemetry_version_follows_mtu(void) {
    TEST_ASSERT_EQUAL_UINT8(2, telemetryVersionForMtu(23));  // ATT default: 20-byte payload
    TEST_ASSERT_EQUAL_UINT8(2, telemetryVersionForMtu(39));  // 36 < 37
    TEST_ASSERT_EQUAL_UINT8(3, telemetryVersionForMtu(40));  // exactly fits
    TEST_ASSERT_EQUAL_UINT8(3, telemetryVersionForMtu(185));
    TEST_ASSERT_EQUAL_UINT8(2, telemetryVersionForMtu(0));   // nonsense MTU -> smallest layout
    TEST_ASSERT_TRUE(sizeof(BLETelemetryPacketV2) <= blePayloadLimit(BLE_DEFAULT_MTU));
}

// ---------------------------------------------------------------------------
// IMU ring buffer + block packing (D-032 -- schema `imuBlock`)
// ---------------------------------------------------------------------------

static ImuSample makeSample(uint32_t index) {
    ImuSample s;
    s.index = index;
    s.timeMs = 5000 + index * IMU_SAMPLE_PERIOD_MS;
    for (int a = 0; a < 3; a++) {
        s.accel[a] = (int16_t)(index * 10 + a);
        s.gyro[a] = (int16_t)(-(int32_t)(index * 10 + a));
    }
    return s;
}

static uint16_t le16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

void test_imu_samples_per_block_follow_mtu(void) {
    TEST_ASSERT_EQUAL_UINT8(0, imuSamplesPerBlock(23));   // default MTU: suspended
    TEST_ASSERT_EQUAL_UINT8(0, imuSamplesPerBlock(26));   // 23-byte payload < 24
    TEST_ASSERT_EQUAL_UINT8(1, imuSamplesPerBlock(27));
    TEST_ASSERT_EQUAL_UINT8(2, imuSamplesPerBlock(40));
    TEST_ASSERT_EQUAL_UINT8(9, imuSamplesPerBlock(134));
    TEST_ASSERT_EQUAL_UINT8(10, imuSamplesPerBlock(135)); // exactly 132 bytes
    TEST_ASSERT_EQUAL_UINT8(10, imuSamplesPerBlock(185));
    TEST_ASSERT_EQUAL_UINT8(10, imuSamplesPerBlock(512));
}

void test_imu_block_layout_timestamps_and_seq(void) {
    ImuRingBuffer<16> ring;
    for (uint32_t i = 100; i < 112; i++) {
        TEST_ASSERT_TRUE(ring.push(makeSample(i)));
    }
    uint8_t out[IMU_BLOCK_MAX_BYTES];
    size_t len = packImuBlock(ring, imuSamplesPerBlock(185), 7, IMU_EVENT_OVERFLOW, out, sizeof(out));
    TEST_ASSERT_EQUAL(132, len);
    TEST_ASSERT_EQUAL_UINT8(IMU_BLOCK_VERSION, out[0]);
    TEST_ASSERT_EQUAL_UINT8(7, out[1]);
    TEST_ASSERT_EQUAL_UINT32(5000 + 100 * 10, le32(&out[2]));
    TEST_ASSERT_EQUAL_UINT16(100, le16(&out[6]));
    TEST_ASSERT_EQUAL_UINT8(10, out[8]);
    TEST_ASSERT_EQUAL_UINT8(10, out[9]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[10]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[11]);
    // Sample 3 = index 103: ax=1030, gz=-1032, little-endian int16.
    const uint8_t* s3 = &out[IMU_BLOCK_HEADER_BYTES + 3 * IMU_BLOCK_SAMPLE_BYTES];
    TEST_ASSERT_EQUAL_INT16(1030, (int16_t)le16(&s3[0]));
    TEST_ASSERT_EQUAL_INT16(1032, (int16_t)le16(&s3[4]));
    TEST_ASSERT_EQUAL_INT16(-1030, (int16_t)le16(&s3[6]));
    TEST_ASSERT_EQUAL_INT16(-1032, (int16_t)le16(&s3[10]));
    // The remaining two samples form the next block with their own start time.
    TEST_ASSERT_EQUAL_UINT32(2, ring.size());
    len = packImuBlock(ring, 10, 8, 0, out, sizeof(out));
    TEST_ASSERT_EQUAL(12 + 2 * 12, len);
    TEST_ASSERT_EQUAL_UINT32(5000 + 110 * 10, le32(&out[2]));
    TEST_ASSERT_EQUAL_UINT16(110, le16(&out[6]));
    TEST_ASSERT_EQUAL_UINT8(2, out[8]);
    TEST_ASSERT_EQUAL(0, packImuBlock(ring, 10, 9, 0, out, sizeof(out))); // empty
}

void test_imu_block_never_spans_an_index_gap(void) {
    ImuRingBuffer<16> ring;
    ring.push(makeSample(1));
    ring.push(makeSample(2));
    ring.push(makeSample(5)); // ticks 3 and 4 missing (read error / missed tick)
    ring.push(makeSample(6));
    uint8_t out[IMU_BLOCK_MAX_BYTES];
    TEST_ASSERT_EQUAL(12 + 2 * 12, packImuBlock(ring, 10, 0, 0, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT16(1, le16(&out[6]));
    TEST_ASSERT_EQUAL(12 + 2 * 12, packImuBlock(ring, 10, 1, 0, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT16(5, le16(&out[6]));
    TEST_ASSERT_EQUAL_UINT32(5000 + 5 * 10, le32(&out[2]));
}

void test_imu_sample_index_wraps_to_uint16_on_the_wire(void) {
    ImuRingBuffer<4> ring;
    ring.push(makeSample(0x1FFFF));
    ring.push(makeSample(0x20000));
    uint8_t out[IMU_BLOCK_MAX_BYTES];
    TEST_ASSERT_EQUAL(12 + 2 * 12, packImuBlock(ring, 10, 0, 0, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, le16(&out[6]));
}

void test_imu_ring_overflow_drops_newest_and_flags_it(void) {
    ImuRingBuffer<4> ring;
    for (uint32_t i = 0; i < 4; i++) {
        TEST_ASSERT_TRUE(ring.push(makeSample(i)));
    }
    TEST_ASSERT_FALSE(ring.push(makeSample(4)));
    TEST_ASSERT_FALSE(ring.push(makeSample(5)));
    TEST_ASSERT_EQUAL_UINT32(2, ring.overflowCount());
    TEST_ASSERT_EQUAL_UINT32(4, ring.size());
    TEST_ASSERT_EQUAL_UINT32(0, ring.peek(0).index); // oldest kept
    TEST_ASSERT_EQUAL_HEX8(IMU_EVENT_OVERFLOW, ring.takeEvents());
    TEST_ASSERT_EQUAL_HEX8(0, ring.takeEvents()); // cleared on read
    ring.markReadError();
    TEST_ASSERT_EQUAL_HEX8(IMU_EVENT_READ_ERROR, ring.takeEvents());
    // Consumer drains, producer continues; the lost ticks 4 and 5 are an index gap.
    ring.drop(4);
    TEST_ASSERT_TRUE(ring.push(makeSample(6)));
    TEST_ASSERT_EQUAL_UINT32(6, ring.peek(0).index);
    ring.drop(10); // over-drop is clamped
    TEST_ASSERT_EQUAL_UINT32(0, ring.size());
}

void test_imu_low_mtu_blocks_fit_the_payload_and_nothing_is_consumed_on_small_buffer(void) {
    ImuRingBuffer<16> ring;
    for (uint32_t i = 0; i < 10; i++) {
        ring.push(makeSample(i));
    }
    uint8_t out[IMU_BLOCK_MAX_BYTES];
    uint8_t n = imuSamplesPerBlock(40); // 2 samples
    size_t len = packImuBlock(ring, n, 0, 0, out, sizeof(out));
    TEST_ASSERT_TRUE(len <= blePayloadLimit(40));
    TEST_ASSERT_EQUAL_UINT32(8, ring.size());
    // Suspended (MTU 23): nothing packed, nothing consumed.
    TEST_ASSERT_EQUAL(0, packImuBlock(ring, imuSamplesPerBlock(23), 0, 0, out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(8, ring.size());
    // Output buffer too small for the block: refused, nothing consumed.
    TEST_ASSERT_EQUAL(0, packImuBlock(ring, 10, 0, 0, out, 20));
    TEST_ASSERT_EQUAL_UINT32(8, ring.size());
}

void test_imu_blocks_per_notify_cap_bounds_work_at_low_mtu(void) {
    // Worst case per 100 ms notify: IMU_MAX_BLOCKS_PER_NOTIFY blocks of the MTU-limited
    // size. At MTU 27 (1 sample per block) the node delivers 30 Hz instead of looping more.
    TEST_ASSERT_EQUAL_UINT8(3, IMU_MAX_BLOCKS_PER_NOTIFY);
    ImuRingBuffer<64> ring;
    for (uint32_t i = 0; i < 10; i++) {
        ring.push(makeSample(i));
    }
    uint8_t out[IMU_BLOCK_MAX_BYTES];
    int blocks = 0;
    for (uint8_t i = 0; i < IMU_MAX_BLOCKS_PER_NOTIFY; i++) {
        if (packImuBlock(ring, imuSamplesPerBlock(27), i, 0, out, sizeof(out)) == 0) break;
        blocks++;
    }
    TEST_ASSERT_EQUAL(3, blocks);
    TEST_ASSERT_EQUAL_UINT32(7, ring.size());
}

void test_imu_ring_concurrent_producer_consumer_keeps_samples_whole_and_ordered(void) {
    // One producer thread (the sampler task) and this thread as the consumer (the BLE
    // pass), as on target. Every field of a sample derives from its index, so a torn
    // sample or a reordering is detected; pushed = delivered + dropped must hold.
    static ImuRingBuffer<64> ring;
    const uint32_t kSamples = 200000;
    uint32_t dropped = 0;
    std::thread producer([&]() {
        for (uint32_t i = 1; i <= kSamples; i++) {
            if (!ring.push(makeSample(i))) {
                dropped++;
            }
        }
    });
    uint32_t delivered = 0, lastIndex = 0;
    bool whole = true, ordered = true;
    uint8_t out[IMU_BLOCK_MAX_BYTES];
    auto drain = [&]() {
        while (ring.size() > 0) {
            const ImuSample& s = ring.peek(0);
            ImuSample expect = makeSample(s.index);
            whole = whole && s.timeMs == expect.timeMs && s.accel[2] == expect.accel[2] &&
                    s.gyro[2] == expect.gyro[2];
            ordered = ordered && s.index > lastIndex;
            lastIndex = s.index;
            size_t len = packImuBlock(ring, 10, 0, 0, out, sizeof(out));
            delivered += (uint32_t)((len - IMU_BLOCK_HEADER_BYTES) / IMU_BLOCK_SAMPLE_BYTES);
        }
    };
    while (delivered + ring.overflowCount() < kSamples) {
        drain();
    }
    producer.join();
    drain();
    TEST_ASSERT_TRUE(whole);
    TEST_ASSERT_TRUE(ordered);
    TEST_ASSERT_EQUAL_UINT32(dropped, ring.overflowCount());
    TEST_ASSERT_EQUAL_UINT32(kSamples, delivered + dropped);
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
    RUN_TEST(test_ble_v3_size_and_offsets);
    RUN_TEST(test_build_v3_packet_values_ages_and_health);
    RUN_TEST(test_v3_age_saturates_and_valid_bit_follows_generated_stale_limit);
    RUN_TEST(test_v3_raw_bytes_are_little_endian);
    RUN_TEST(test_telemetry_version_follows_mtu);
    RUN_TEST(test_imu_samples_per_block_follow_mtu);
    RUN_TEST(test_imu_block_layout_timestamps_and_seq);
    RUN_TEST(test_imu_block_never_spans_an_index_gap);
    RUN_TEST(test_imu_sample_index_wraps_to_uint16_on_the_wire);
    RUN_TEST(test_imu_ring_overflow_drops_newest_and_flags_it);
    RUN_TEST(test_imu_low_mtu_blocks_fit_the_payload_and_nothing_is_consumed_on_small_buffer);
    RUN_TEST(test_imu_blocks_per_notify_cap_bounds_work_at_low_mtu);
    RUN_TEST(test_imu_ring_concurrent_producer_consumer_keeps_samples_whole_and_ordered);
    RUN_TEST(test_json_fresh_values);
    RUN_TEST(test_json_stale_values_are_null);
    RUN_TEST(test_json_escapes_phone_strings_and_fits_worst_case);
    RUN_TEST(test_json_reports_overflow_instead_of_truncating);
    return UNITY_END();
}
