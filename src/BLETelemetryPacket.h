#ifndef BLE_TELEMETRY_PACKET_H
#define BLE_TELEMETRY_PACKET_H

#include <Arduino.h>

#include "SystemState.h"

// G3.3 -- Single source of truth for these layouts: docs/ble_telemetry_packet_schema.json.
// moto-mobile decodes them (lib/models/telemetry_data.dart) and checks a copy of that JSON
// in its tests; moto-server re-decodes recorded raw_hex from the same JSON. Bump the
// version here and in the schema together whenever a layout or meaning changes.
//
// Version 3 (D-032) is the normal packet: node clock, per-signal ages and CAN/tester
// health. Version 2 (D-023 port) stays as the low-MTU fallback: it fits the default ATT
// MTU of 23, so a phone that never negotiated a larger MTU still gets values.
#define BLE_PACKET_VERSION 3
#define BLE_PACKET_VERSION_LEGACY 2

// Lean fields hold this when no lean estimate is available. Deprecated (D-023): the
// legacy complementary filter was dropped, lean comes from the rt-core EKF later.
constexpr int16_t BLE_LEAN_NOT_AVAILABLE = INT16_MIN;

// Signal ages (ms) saturate at BLE_AGE_MAX_MS; BLE_AGE_NEVER_RECEIVED = no value since boot.
constexpr uint16_t BLE_AGE_MAX_MS = 65534;
constexpr uint16_t BLE_AGE_NEVER_RECEIVED = 65535;

// Bits of the flags byte (both versions). A cleared valid bit means the value is stale
// or was never received; receivers show it as unavailable.
constexpr uint8_t BLE_FLAG_RPM_VALID         = 1u << 0;
constexpr uint8_t BLE_FLAG_SPEED_VALID       = 1u << 1;
constexpr uint8_t BLE_FLAG_COOLANT_VALID     = 1u << 2;
constexpr uint8_t BLE_FLAG_THROTTLE_VALID    = 1u << 3;
constexpr uint8_t BLE_FLAG_BATTERY_VALID     = 1u << 4;
constexpr uint8_t BLE_FLAG_LEAN_VALID        = 1u << 5;
constexpr uint8_t BLE_FLAG_ECU_PRESENT       = 1u << 6;
constexpr uint8_t BLE_FLAG_IMU_ACTIVE        = 1u << 7; // version 3 only

// ATT notification payload limit for a negotiated MTU (3 bytes of ATT header).
constexpr uint16_t BLE_ATT_HEADER_BYTES = 3;
constexpr uint16_t BLE_DEFAULT_MTU = 23;
inline uint16_t blePayloadLimit(uint16_t mtu) {
    return mtu > BLE_ATT_HEADER_BYTES ? (uint16_t)(mtu - BLE_ATT_HEADER_BYTES) : 0;
}

/**
 * @brief Version 2 telemetry payload (16 bytes, little-endian, packed). Low-MTU fallback.
 * See the schema's `lowMtuFallback` for the field table.
 */
struct __attribute__((packed)) BLETelemetryPacketV2 {
    uint8_t  version;       // BLE_PACKET_VERSION_LEGACY
    uint8_t  seq;           // shared with version 3
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
static_assert(sizeof(BLETelemetryPacketV2) == 16, "BLE v2 layout must match the schema");

/**
 * @brief Version 3 telemetry payload (37 bytes, little-endian, packed). Field order
 * matches the schema's `fields` offsets exactly.
 */
struct __attribute__((packed)) BLETelemetryPacketV3 {
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
};
static_assert(sizeof(BLETelemetryPacketV3) == 37, "BLE v3 layout must match the schema");

// Which layout fits a negotiated MTU: version 3 when it fits, else the version 2 fallback.
inline uint8_t telemetryVersionForMtu(uint16_t mtu) {
    return blePayloadLimit(mtu) >= sizeof(BLETelemetryPacketV3) ? BLE_PACKET_VERSION : BLE_PACKET_VERSION_LEGACY;
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

// Valid bits from the generated per-DID stale_after_ms (D-029), shared by both versions.
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
 * @brief Builds a version 3 packet. Pure (no BLE calls); nowMs is the node clock
 * (millis()) at build time.
 */
inline BLETelemetryPacketV3 buildTelemetryPacketV3(const SystemState& state, uint8_t seq, uint32_t nowMs) {
    const EngineData& e = state.engine;
    const CanHealth& c = state.can;
    BLETelemetryPacketV3 p;
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
    return p;
}

#endif // BLE_TELEMETRY_PACKET_H
