#ifndef BLE_TELEMETRY_PACKET_H
#define BLE_TELEMETRY_PACKET_H

#include <Arduino.h>

#include "SystemState.h"

// G3.3 -- Single source of truth for this layout: docs/ble_telemetry_packet_schema.json.
// moto-mobile decodes it (lib/models/telemetry_data.dart) and checks a copy of that JSON
// in its tests. Bump both this and the schema's "version" together whenever the layout
// or meaning changes -- see BLE_PACKET_VERSION below.
//
// Version 2 (D-023 port): lean fields keep their offsets but carry
// BLE_LEAN_NOT_AVAILABLE until the rt-core EKF lean estimate reaches this node (the
// legacy complementary filter was dropped), and byte 15 carries validity flags.
#define BLE_PACKET_VERSION 2

// Lean fields hold this when no lean estimate is available.
constexpr int16_t BLE_LEAN_NOT_AVAILABLE = INT16_MIN;

// Bits of BLETelemetryPacket::flags. A cleared bit means the value is stale or was
// never received; receivers show it as unavailable.
constexpr uint8_t BLE_FLAG_RPM_VALID         = 1u << 0;
constexpr uint8_t BLE_FLAG_SPEED_VALID       = 1u << 1;
constexpr uint8_t BLE_FLAG_COOLANT_VALID     = 1u << 2;
constexpr uint8_t BLE_FLAG_THROTTLE_VALID    = 1u << 3;
constexpr uint8_t BLE_FLAG_BATTERY_VALID     = 1u << 4;
constexpr uint8_t BLE_FLAG_LEAN_VALID        = 1u << 5;
constexpr uint8_t BLE_FLAG_ECU_PRESENT       = 1u << 6;

/**
 * @brief Binary telemetry payload sent over BLE notifications.
 *
 * G1.1 -- Wire format is explicitly LITTLE-ENDIAN (native to the ESP32-S3/Xtensa CPU;
 * Dart's ByteData defaults to big-endian, so every consumer MUST decode with that flag set).
 *
 * 16 bytes total, field order below matches byte offset order exactly (no padding, due
 * to __attribute__((packed))):
 *   offset 0    : version      uint8   -- must equal BLE_PACKET_VERSION
 *   offset 1    : seq          uint8   -- rolls over 0-255, gaps = lost notifications
 *   offset 2-3  : rpm          uint16 LE
 *   offset 4    : speed        uint8
 *   offset 5    : coolantTemp  int8    -- clamped to -40..127
 *   offset 6    : throttlePos  uint8
 *   offset 7-8  : batteryVolt  uint16 LE (millivolts)
 *   offset 9-10 : leanAngle    int16 LE (0.1 deg, BLE_LEAN_NOT_AVAILABLE if none)
 *   offset 11-12: maxLeanRight int16 LE (0.1 deg, BLE_LEAN_NOT_AVAILABLE if none)
 *   offset 13-14: maxLeanLeft  int16 LE (0.1 deg, BLE_LEAN_NOT_AVAILABLE if none)
 *   offset 15   : flags        uint8   -- BLE_FLAG_* bits
 */
struct __attribute__((packed)) BLETelemetryPacket {
    uint8_t  version;
    uint8_t  seq;
    uint16_t rpm;
    uint8_t  speed;
    int8_t   coolantTemp;
    uint8_t  throttlePos;
    uint16_t batteryVolt;
    int16_t  leanAngle;
    int16_t  maxLeanRight;
    int16_t  maxLeanLeft;
    uint8_t  flags;
};

static_assert(sizeof(BLETelemetryPacket) == 16, "BLE packet layout must match the schema");

/**
 * @brief Builds one packet from the current state. Pure (no BLE calls) so the native
 * tests can check it against the schema.
 */
inline BLETelemetryPacket buildTelemetryPacket(const SystemState& state, uint8_t seq) {
    const EngineData& e = state.engine;
    BLETelemetryPacket p;
    p.version = BLE_PACKET_VERSION;
    p.seq = seq;
    p.rpm = (uint16_t)constrain(e.rpm, 0.0f, 65535.0f);
    p.speed = e.speed;
    p.coolantTemp = (int8_t)constrain(e.coolantTemp, (int16_t)-40, (int16_t)127);
    p.throttlePos = (uint8_t)constrain(e.throttlePos, 0.0f, 100.0f);
    p.batteryVolt = (uint16_t)constrain(e.batteryVoltage * 1000.0f, 0.0f, 65535.0f);
    p.leanAngle = BLE_LEAN_NOT_AVAILABLE;
    p.maxLeanRight = BLE_LEAN_NOT_AVAILABLE;
    p.maxLeanLeft = BLE_LEAN_NOT_AVAILABLE;

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
    // BLE_FLAG_LEAN_VALID stays 0 until a lean source exists.
    p.flags = flags;
    return p;
}

#endif // BLE_TELEMETRY_PACKET_H
