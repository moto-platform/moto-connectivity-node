#ifndef BLE_TELEMETRY_PACKET_H
#define BLE_TELEMETRY_PACKET_H

#include <Arduino.h>
#include <stddef.h>

#include "SystemState.h"
#include "ble_schema.h" // generated: external/moto-vehicle-defs/gen/c/conn/ (D-061)

// The telemetry layouts are defined in moto-vehicle-defs ble/ble_schema.json (D-061) and
// generated into ble_schema.h as offset/size macros. The packed structs below static_assert
// every field against those macros, so a layout change is a defs change (version bump) that
// fails the build here until the structs follow. moto-mobile and moto-server use the Python
// and Dart outputs generated from the same file.
//
// Version 4 (D-058) is the normal packet: version 3 (node clock, per-signal ages, CAN/tester
// health) plus the tester step gap and one rotating per-DID round-trip record. Version 2
// (D-023 port) stays as the low-MTU fallback: it fits the default ATT MTU of 23, so a phone
// that never negotiated a larger MTU still gets values. Version 3 is no longer produced.
#define BLE_PACKET_VERSION BLE_TELEMETRY_CURRENT_VERSION
#define BLE_PACKET_VERSION_LEGACY BLE_TELEMETRY_LEGACY_VERSION

// Lean fields hold this when no lean estimate is available. Deprecated (D-023): the
// legacy complementary filter was dropped, lean comes from the rt-core EKF later.
constexpr int16_t BLE_LEAN_NOT_AVAILABLE = (int16_t)BLE_TELEMETRY_NOT_AVAILABLE_INT16;

// Signal ages (ms) saturate at BLE_AGE_MAX_MS; BLE_AGE_NEVER_RECEIVED = no value since boot.
constexpr uint16_t BLE_AGE_MAX_MS = BLE_TELEMETRY_AGE_MAX;
constexpr uint16_t BLE_AGE_NEVER_RECEIVED = BLE_TELEMETRY_AGE_NEVER_RECEIVED;

// Bits of the flags byte (all versions). A cleared valid bit means the value is stale
// or was never received; receivers show it as unavailable.
constexpr uint8_t BLE_FLAG_RPM_VALID         = BLE_TELEMETRY_FLAG_RPM_VALID;
constexpr uint8_t BLE_FLAG_SPEED_VALID       = BLE_TELEMETRY_FLAG_SPEED_VALID;
constexpr uint8_t BLE_FLAG_COOLANT_VALID     = BLE_TELEMETRY_FLAG_COOLANT_TEMP_VALID;
constexpr uint8_t BLE_FLAG_THROTTLE_VALID    = BLE_TELEMETRY_FLAG_THROTTLE_POS_VALID;
constexpr uint8_t BLE_FLAG_BATTERY_VALID     = BLE_TELEMETRY_FLAG_BATTERY_VOLT_VALID;
constexpr uint8_t BLE_FLAG_LEAN_VALID        = BLE_TELEMETRY_FLAG_LEAN_VALID;
constexpr uint8_t BLE_FLAG_ECU_PRESENT       = BLE_TELEMETRY_FLAG_ECU_PRESENT;
constexpr uint8_t BLE_FLAG_IMU_ACTIVE        = BLE_TELEMETRY_FLAG_IMU_ACTIVE; // version 3 and 4 only

// SystemState.h keeps its own copies of the CAN health values; they are wire values.
static_assert((uint8_t)CanHealthState::NOT_INSTALLED == BLE_CAN_BUS_STATE_NOT_INSTALLED, "canBusState must match the schema");
static_assert((uint8_t)CanHealthState::RUNNING == BLE_CAN_BUS_STATE_RUNNING, "canBusState must match the schema");
static_assert((uint8_t)CanHealthState::ERROR_WARNING == BLE_CAN_BUS_STATE_ERROR_WARNING, "canBusState must match the schema");
static_assert((uint8_t)CanHealthState::BUS_OFF == BLE_CAN_BUS_STATE_BUS_OFF, "canBusState must match the schema");
static_assert((uint8_t)CanHealthState::STOPPED == BLE_CAN_BUS_STATE_STOPPED, "canBusState must match the schema");
static_assert(CAN_HEALTH_FLAG_POLLER_ENABLED == BLE_CAN_FLAG_POLLER_ENABLED, "canFlags must match the schema");
static_assert(CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER == BLE_CAN_FLAG_LATCHED_FOREIGN_TESTER, "canFlags must match the schema");
static_assert(CAN_HEALTH_FLAG_LATCHED_BUS_OFF == BLE_CAN_FLAG_LATCHED_BUS_OFF, "canFlags must match the schema");
static_assert(CAN_HEALTH_FLAG_SYNTHETIC_DATA == BLE_CAN_FLAG_SYNTHETIC_DATA, "canFlags must match the schema");

// ATT notification payload limit for a negotiated MTU (3 bytes of ATT header).
constexpr uint16_t BLE_ATT_HEADER_BYTES = 3;
constexpr uint16_t BLE_DEFAULT_MTU = 23;
inline uint16_t blePayloadLimit(uint16_t mtu) {
    return mtu > BLE_ATT_HEADER_BYTES ? (uint16_t)(mtu - BLE_ATT_HEADER_BYTES) : 0;
}

// static_assert one packed field against the generated <PREFIX>_<FIELD>_OFFSET / _SIZE.
#define BLE_ASSERT_FIELD(T, field, MACRO) \
    static_assert(offsetof(T, field) == MACRO##_OFFSET, #T "::" #field " offset must match the schema"); \
    static_assert(sizeof(((T*)nullptr)->field) == MACRO##_SIZE, #T "::" #field " size must match the schema")

/**
 * @brief Version 2 telemetry payload (16 bytes, little-endian, packed). Low-MTU fallback.
 * Field table: ble_schema.json `lowMtuFallback`.
 */
struct __attribute__((packed)) BLETelemetryPacketV2 {
    uint8_t  version;       // BLE_PACKET_VERSION_LEGACY
    uint8_t  seq;           // shared with version 4
    uint16_t rpm;
    uint8_t  speed;
    int8_t   coolantTemp;   // clamped to -40..127
    uint8_t  throttlePos;
    uint16_t batteryVolt;   // millivolts
    int16_t  leanAngle;     // deprecated, BLE_LEAN_NOT_AVAILABLE
    int16_t  maxLeanRight;  // deprecated, BLE_LEAN_NOT_AVAILABLE
    int16_t  maxLeanLeft;   // deprecated, BLE_LEAN_NOT_AVAILABLE
    uint8_t  flags;         // BLE_FLAG_* bits, bit 7 always 0
};
static_assert(sizeof(BLETelemetryPacketV2) == BLE_TELEMETRY_V2_TOTAL_BYTES, "BLE v2 size must match the schema");
BLE_ASSERT_FIELD(BLETelemetryPacketV2, version, BLE_TELEMETRY_V2_VERSION);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, seq, BLE_TELEMETRY_V2_SEQ);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, rpm, BLE_TELEMETRY_V2_RPM);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, speed, BLE_TELEMETRY_V2_SPEED);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, coolantTemp, BLE_TELEMETRY_V2_COOLANT_TEMP);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, throttlePos, BLE_TELEMETRY_V2_THROTTLE_POS);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, batteryVolt, BLE_TELEMETRY_V2_BATTERY_VOLT);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, leanAngle, BLE_TELEMETRY_V2_LEAN_ANGLE);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, maxLeanRight, BLE_TELEMETRY_V2_MAX_LEAN_RIGHT);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, maxLeanLeft, BLE_TELEMETRY_V2_MAX_LEAN_LEFT);
BLE_ASSERT_FIELD(BLETelemetryPacketV2, flags, BLE_TELEMETRY_V2_FLAGS);

/**
 * @brief Version 4 telemetry payload (57 bytes, little-endian, packed): the version 3
 * fields (node clock, ages, CAN health) plus the D-058 tester statistics. The tester
 * fields are TEMPORARY (rt-core's health DID 0xFD02 replaces them, D-055).
 */
struct __attribute__((packed)) BLETelemetryPacketV4 {
    uint8_t  version;            // 0  BLE_PACKET_VERSION
    uint8_t  seq;                // 1
    uint32_t deviceTimeMs;       // 2  millis() when built; same clock as the IMU blocks
    uint16_t rpm;                // 6
    uint8_t  speed;              // 8
    int8_t   coolantTemp;        // 9
    uint8_t  throttlePos;        // 10
    uint16_t batteryVolt;        // 11 millivolts
    int16_t  leanAngle;          // 13 deprecated
    int16_t  maxLeanRight;       // 15 deprecated
    int16_t  maxLeanLeft;        // 17 deprecated
    uint8_t  flags;              // 19
    uint16_t rpmAgeMs;           // 20
    uint16_t speedAgeMs;         // 22
    uint16_t coolantTempAgeMs;   // 24
    uint16_t throttlePosAgeMs;   // 26
    uint16_t batteryVoltAgeMs;   // 28
    uint8_t  canBusState;        // 30 CanHealthState
    uint8_t  canTxErrorCount;    // 31 saturating
    uint8_t  canRxErrorCount;    // 32 saturating
    uint8_t  canBusOffCount;     // 33 saturating
    uint16_t unansweredDidCount; // 34 saturating
    uint8_t  canFlags;           // 36 CAN_HEALTH_FLAG_* bits
    uint16_t stepGapMaxMs;       // 37 poller step gap, max since boot (ms, rounded up)
    uint16_t stepGapOverCount;   // 39 gaps above client_step_max_ms
    uint16_t rttDid;             // 41 DID of the round-trip record, 0 = no record
    uint16_t rttMinMs;           // 43 65535 = no sample yet
    uint16_t rttMaxMs;           // 45 0 = no sample yet
    uint32_t rttSumMs;           // 47 saturating
    uint32_t rttCount;           // 51 saturating
    uint16_t rttNrc78Count;      // 55 saturating
};
static_assert(sizeof(BLETelemetryPacketV4) == BLE_TELEMETRY_V4_TOTAL_BYTES, "BLE v4 size must match the schema");
BLE_ASSERT_FIELD(BLETelemetryPacketV4, version, BLE_TELEMETRY_V4_VERSION);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, seq, BLE_TELEMETRY_V4_SEQ);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, deviceTimeMs, BLE_TELEMETRY_V4_DEVICE_TIME_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rpm, BLE_TELEMETRY_V4_RPM);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, speed, BLE_TELEMETRY_V4_SPEED);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, coolantTemp, BLE_TELEMETRY_V4_COOLANT_TEMP);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, throttlePos, BLE_TELEMETRY_V4_THROTTLE_POS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, batteryVolt, BLE_TELEMETRY_V4_BATTERY_VOLT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, leanAngle, BLE_TELEMETRY_V4_LEAN_ANGLE);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, maxLeanRight, BLE_TELEMETRY_V4_MAX_LEAN_RIGHT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, maxLeanLeft, BLE_TELEMETRY_V4_MAX_LEAN_LEFT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, flags, BLE_TELEMETRY_V4_FLAGS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rpmAgeMs, BLE_TELEMETRY_V4_RPM_AGE_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, speedAgeMs, BLE_TELEMETRY_V4_SPEED_AGE_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, coolantTempAgeMs, BLE_TELEMETRY_V4_COOLANT_TEMP_AGE_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, throttlePosAgeMs, BLE_TELEMETRY_V4_THROTTLE_POS_AGE_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, batteryVoltAgeMs, BLE_TELEMETRY_V4_BATTERY_VOLT_AGE_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, canBusState, BLE_TELEMETRY_V4_CAN_BUS_STATE);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, canTxErrorCount, BLE_TELEMETRY_V4_CAN_TX_ERROR_COUNT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, canRxErrorCount, BLE_TELEMETRY_V4_CAN_RX_ERROR_COUNT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, canBusOffCount, BLE_TELEMETRY_V4_CAN_BUS_OFF_COUNT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, unansweredDidCount, BLE_TELEMETRY_V4_UNANSWERED_DID_COUNT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, canFlags, BLE_TELEMETRY_V4_CAN_FLAGS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, stepGapMaxMs, BLE_TELEMETRY_V4_STEP_GAP_MAX_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, stepGapOverCount, BLE_TELEMETRY_V4_STEP_GAP_OVER_COUNT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rttDid, BLE_TELEMETRY_V4_RTT_DID);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rttMinMs, BLE_TELEMETRY_V4_RTT_MIN_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rttMaxMs, BLE_TELEMETRY_V4_RTT_MAX_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rttSumMs, BLE_TELEMETRY_V4_RTT_SUM_MS);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rttCount, BLE_TELEMETRY_V4_RTT_COUNT);
BLE_ASSERT_FIELD(BLETelemetryPacketV4, rttNrc78Count, BLE_TELEMETRY_V4_RTT_NRC78_COUNT);

// Which layout fits a negotiated MTU: the current version when it fits, else the version 2
// fallback.
inline uint8_t telemetryVersionForMtu(uint16_t mtu) {
    return blePayloadLimit(mtu) >= BLE_TELEMETRY_V4_TOTAL_BYTES ? BLE_PACKET_VERSION : BLE_PACKET_VERSION_LEGACY;
}

inline uint16_t signalAgeMs(uint32_t updatedMs, uint32_t nowMs) {
    if (updatedMs == 0) {
        return BLE_AGE_NEVER_RECEIVED;
    }
    uint32_t age = nowMs - updatedMs;
    return age > BLE_AGE_MAX_MS ? BLE_AGE_MAX_MS : (uint16_t)age;
}

inline uint8_t saturateU8(uint32_t v) { return v > 0xFFu ? 0xFFu : (uint8_t)v; }
inline uint16_t saturateU16(uint32_t v) { return v > 0xFFFFu ? 0xFFFFu : (uint16_t)v; }

// Valid bits from the generated per-DID stale_after_ms (D-029), shared by all versions.
inline uint8_t telemetryValidFlags(const EngineData& e) {
    uint8_t flags = 0;
    if (!isStale(e.rpmUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_IDX_ENGINE_SPEED))) {
        flags |= BLE_FLAG_RPM_VALID;
    }
    if (!isStale(e.speedUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_IDX_VEHICLE_SPEED))) {
        flags |= BLE_FLAG_SPEED_VALID;
    }
    if (!isStale(e.coolantTempUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_IDX_COOLANT_TEMP))) {
        flags |= BLE_FLAG_COOLANT_VALID;
    }
    if (!isStale(e.throttlePosUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_IDX_THROTTLE_POS))) {
        flags |= BLE_FLAG_THROTTLE_VALID;
    }
    if (!isStale(e.batteryVoltageUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_IDX_BATTERY_VOLTAGE))) {
        flags |= BLE_FLAG_BATTERY_VALID;
    }
    if (e.ecuPresent) {
        flags |= BLE_FLAG_ECU_PRESENT;
    }
    // BLE_FLAG_LEAN_VALID stays 0: lean is deprecated until an EKF source exists.
    return flags;
}

/**
 * @brief Builds a version 2 packet (low-MTU fallback). Pure (no BLE calls) so the native
 * tests can check it against the schema.
 */
inline BLETelemetryPacketV2 buildTelemetryPacketV2(const SystemState& state, uint8_t seq) {
    const EngineData& e = state.engine;
    BLETelemetryPacketV2 p;
    p.version = BLE_PACKET_VERSION_LEGACY;
    p.seq = seq;
    p.rpm = (uint16_t)constrain(e.rpm, 0.0f, 65535.0f);
    p.speed = e.speed;
    p.coolantTemp = (int8_t)constrain(e.coolantTemp, (int16_t)-40, (int16_t)127);
    p.throttlePos = (uint8_t)constrain(e.throttlePos, 0.0f, 100.0f);
    p.batteryVolt = (uint16_t)constrain(e.batteryVoltage * 1000.0f, 0.0f, 65535.0f);
    p.leanAngle = BLE_LEAN_NOT_AVAILABLE;
    p.maxLeanRight = BLE_LEAN_NOT_AVAILABLE;
    p.maxLeanLeft = BLE_LEAN_NOT_AVAILABLE;
    p.flags = telemetryValidFlags(e);
    return p;
}

/**
 * @brief Builds a version 4 packet. Pure (no BLE calls); nowMs is the node clock
 * (millis()) at build time. The round-trip record is for the DID table index
 * rttIndex % VEHICLE_CL250_DID_COUNT (the caller advances rttIndex once per packet). A node
 * without tester statistics (mock, poller off, listen-only) sends the "no data" values:
 * rttDid 0, rttMinMs 65535, rttMaxMs 0, everything else 0.
 */
inline BLETelemetryPacketV4 buildTelemetryPacketV4(const SystemState& state, uint8_t seq, uint32_t nowMs,
                                                   uint32_t rttIndex) {
    const EngineData& e = state.engine;
    const CanHealth& c = state.can;
    const TesterStats& t = state.tester;
    BLETelemetryPacketV4 p;
    p.version = BLE_PACKET_VERSION;
    p.seq = seq;
    p.deviceTimeMs = nowMs;
    p.rpm = (uint16_t)constrain(e.rpm, 0.0f, 65535.0f);
    p.speed = e.speed;
    p.coolantTemp = (int8_t)constrain(e.coolantTemp, (int16_t)-40, (int16_t)127);
    p.throttlePos = (uint8_t)constrain(e.throttlePos, 0.0f, 100.0f);
    p.batteryVolt = (uint16_t)constrain(e.batteryVoltage * 1000.0f, 0.0f, 65535.0f);
    p.leanAngle = BLE_LEAN_NOT_AVAILABLE;
    p.maxLeanRight = BLE_LEAN_NOT_AVAILABLE;
    p.maxLeanLeft = BLE_LEAN_NOT_AVAILABLE;
    uint8_t flags = telemetryValidFlags(e);
    if (state.imu.active) {
        flags |= BLE_FLAG_IMU_ACTIVE;
    }
    p.flags = flags;
    p.rpmAgeMs = signalAgeMs(e.rpmUpdatedMs, nowMs);
    p.speedAgeMs = signalAgeMs(e.speedUpdatedMs, nowMs);
    p.coolantTempAgeMs = signalAgeMs(e.coolantTempUpdatedMs, nowMs);
    p.throttlePosAgeMs = signalAgeMs(e.throttlePosUpdatedMs, nowMs);
    p.batteryVoltAgeMs = signalAgeMs(e.batteryVoltageUpdatedMs, nowMs);
    p.canBusState = (uint8_t)c.busState;
    p.canTxErrorCount = saturateU8(c.txErrorCount);
    p.canRxErrorCount = saturateU8(c.rxErrorCount);
    p.canBusOffCount = saturateU8(c.busOffCount);
    p.unansweredDidCount = saturateU16(c.unansweredDidCount);
    p.canFlags = c.flags;
    if (t.available) {
        const uint8_t idx = (uint8_t)(rttIndex % VEHICLE_CL250_DID_COUNT);
        const DidRoundTripStats& r = t.rtt[idx];
        p.stepGapMaxMs = t.stepGapMaxMs;
        p.stepGapOverCount = t.stepGapOverCount;
        p.rttDid = vehicle_cl250_dids[idx].did;
        p.rttMinMs = r.minMs;
        p.rttMaxMs = r.maxMs;
        p.rttSumMs = r.sumMs;
        p.rttCount = r.count;
        p.rttNrc78Count = r.nrc78Count;
    } else {
        p.stepGapMaxMs = 0;
        p.stepGapOverCount = 0;
        p.rttDid = 0;
        p.rttMinMs = TESTER_RTT_NONE_MS;
        p.rttMaxMs = 0;
        p.rttSumMs = 0;
        p.rttCount = 0;
        p.rttNrc78Count = 0;
    }
    return p;
}

#endif // BLE_TELEMETRY_PACKET_H
