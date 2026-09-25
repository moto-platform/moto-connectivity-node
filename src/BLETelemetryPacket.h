#ifndef BLE_TELEMETRY_PACKET_H
#define BLE_TELEMETRY_PACKET_H

#include <Arduino.h>

// G3.3 -- Single source of truth for this layout: docs/ble_telemetry_packet_schema.json.
// That file is also machine-readable (JSON) for future tooling, e.g. a TinyML
// training-data pipeline that needs to decode logged BLE packets without a second
// hand-maintained copy of this layout. Bump both this and the schema's "version"
// together whenever the layout changes -- see BLE_PACKET_VERSION below.
#define BLE_PACKET_VERSION 1

/**
 * @brief Binary telemetry payload sent over BLE notifications.
 * Packed attribute guarantees byte alignment across different microcontrollers/mobile OS architectures.
 *
 * G1.1 -- Wire format is explicitly LITTLE-ENDIAN (native to the ESP32-S3/Xtensa CPU;
 * Web Bluetooth's DataView and Dart's ByteData both default to big-endian, so every
 * consumer MUST decode with that flag set).
 *
 * G3.3 -- 15 bytes total, field order below matches byte offset order exactly (no
 * padding, due to __attribute__((packed))). `version`/`seq` were added at the front
 * specifically so a receiver can validate/track them before touching anything else:
 *   offset 0    : version      uint8   -- must equal BLE_PACKET_VERSION; receivers
 *                                          reject the packet otherwise (schema.json).
 *   offset 1    : seq          uint8   -- rolls over 0-255; receivers use gaps in this
 *                                          to count lost BLE notifications.
 *   offset 2-3  : rpm          uint16 LE
 *   offset 4    : speed        uint8
 *   offset 5    : coolantTemp  int8
 *   offset 6    : throttlePos  uint8
 *   offset 7-8  : batteryVolt  uint16 LE
 *   offset 9-10 : leanAngle    int16 LE
 *   offset 11-12: maxLeanRight int16 LE
 *   offset 13-14: maxLeanLeft  int16 LE
 * Mirrored in mobile_app/app.js (parseTelemetryPacket, DataView getX(offset, true))
 * and mobile_app/flutter_app/lib/models/telemetry_data.dart (fromBinaryBuffer, Endian.little).
 * Changing this layout requires updating all three places AND docs/ble_telemetry_packet_schema.json.
 */
struct __attribute__((packed)) BLETelemetryPacket {
    uint8_t  version;      // Packet format version -- see BLE_PACKET_VERSION
    uint8_t  seq;          // Rolling 0-255 sequence counter, for packet-loss measurement
    uint16_t rpm;          // Engine RPM (0 - 15,000)
    uint8_t  speed;        // Vehicle Speed in km/h (0 - 255)
    int8_t   coolantTemp;  // Coolant Temperature in °C (-40 to 150)
    uint8_t  throttlePos;  // Throttle position percentage (0 - 100%)
    uint16_t batteryVolt;  // Battery Voltage in millivolts (e.g., 12400 = 12.4V)
    int16_t  leanAngle;    // Current roll/lean angle in tenths of a degree (e.g. 254 = 25.4°)
    int16_t  maxLeanRight; // Peak right lean angle in tenths of a degree
    int16_t  maxLeanLeft;  // Peak left lean angle in tenths of a degree
};

#endif // BLE_TELEMETRY_PACKET_H
