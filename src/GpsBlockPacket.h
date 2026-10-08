#ifndef GPS_BLOCK_PACKET_H
#define GPS_BLOCK_PACKET_H

#include <stddef.h>
#include <stdint.h>

#include "GpsFix.h"
#include "ble_schema.h" // generated: external/moto-vehicle-defs/gen/c/conn/ (D-061)

// GPS block notification (moto-vehicle-defs ble/ble_schema.json `gpsBlock`, D-060, D-061);
// sizes, offsets, scales and flag bits come from the generated ble_schema.h. Little-endian,
// written byte by byte (no struct punning), 26 bytes:
//   version u8, seq u8, deviceTimeMs u32, groundSpeed i32, headingOfMotion i32,
//   speedAccuracy u32, headingAccuracy u32, fixType u8, numSv u8, flags u8, reserved u8
// There is no latitude, longitude or height field to fill (invariant 7): GpsFix holds none.
constexpr uint8_t GPS_BLOCK_VERSION = BLE_GPS_BLOCK_VERSION;
constexpr size_t GPS_BLOCK_BYTES = BLE_GPS_TOTAL_BYTES;

// GpsFix keeps the raw UBX NAV-PVT units, and the block uses the same LSBs, so packing is a
// plain copy with no scaling. A schema scale change must fail here, not on the phone.
static_assert(BLE_GPS_SPEED_LSB_PER_MPS == 1000u, "GpsFix speeds are mm/s: the block LSB must be 1 mm/s");
static_assert(BLE_GPS_HEADING_LSB_PER_DEG == 100000u, "GpsFix headings are 1e-5 deg: the block LSB must match");

// packGpsBlock() writes the fields in this order and with these sizes.
static_assert(BLE_GPS_VERSION_OFFSET == 0 && BLE_GPS_SEQ_OFFSET == 1 && BLE_GPS_DEVICE_TIME_MS_OFFSET == 2 &&
              BLE_GPS_GROUND_SPEED_OFFSET == 6 && BLE_GPS_HEADING_OF_MOTION_OFFSET == 10 &&
              BLE_GPS_SPEED_ACCURACY_OFFSET == 14 && BLE_GPS_HEADING_ACCURACY_OFFSET == 18 &&
              BLE_GPS_FIX_TYPE_OFFSET == 22 && BLE_GPS_NUM_SV_OFFSET == 23 && BLE_GPS_FLAGS_OFFSET == 24 &&
              BLE_GPS_RESERVED_OFFSET == 25,
              "GPS block field order must match the schema");
static_assert(BLE_GPS_VERSION_SIZE == 1 && BLE_GPS_SEQ_SIZE == 1 && BLE_GPS_DEVICE_TIME_MS_SIZE == 4 &&
              BLE_GPS_GROUND_SPEED_SIZE == 4 && BLE_GPS_HEADING_OF_MOTION_SIZE == 4 &&
              BLE_GPS_SPEED_ACCURACY_SIZE == 4 && BLE_GPS_HEADING_ACCURACY_SIZE == 4 &&
              BLE_GPS_FIX_TYPE_SIZE == 1 && BLE_GPS_NUM_SV_SIZE == 1 && BLE_GPS_FLAGS_SIZE == 1 &&
              BLE_GPS_RESERVED_SIZE == 1,
              "GPS block field sizes must match the schema");
static_assert(GPS_BLOCK_BYTES == BLE_GPS_RESERVED_OFFSET + BLE_GPS_RESERVED_SIZE, "GPS block total must match the schema");
static_assert(sizeof(GpsFix::groundSpeedMmps) == BLE_GPS_GROUND_SPEED_SIZE &&
              sizeof(GpsFix::headingMotionE5) == BLE_GPS_HEADING_OF_MOTION_SIZE &&
              sizeof(GpsFix::speedAccMmps) == BLE_GPS_SPEED_ACCURACY_SIZE &&
              sizeof(GpsFix::headingAccE5) == BLE_GPS_HEADING_ACCURACY_SIZE &&
              sizeof(GpsFix::rxTimeMs) == BLE_GPS_DEVICE_TIME_MS_SIZE,
              "GpsFix fields must copy into the block without truncation");

// GpsCore's error counters at one block. They only grow (mod 2^32), so any change since
// the previous block means at least one new error.
struct GpsErrorCounters {
    uint32_t parseErrors;   // GpsCore checksumErrors() + lengthErrors()
    uint32_t uartOverflows; // GpsCore uartOverflows()
};

/**
 * @brief The block's flags byte (BLE_GPS_FLAG_*): gnssFixOK from the fix, the error bits
 * from the counter change between the previous block (prev) and this one (now).
 */
inline uint8_t gpsBlockFlags(const GpsFix& fix, const GpsErrorCounters& now, const GpsErrorCounters& prev) {
    uint8_t flags = 0u;
    if (fix.gnssFixOk != 0u) {
        flags |= (uint8_t)BLE_GPS_FLAG_GNSS_FIX_OK;
    }
    if (now.parseErrors != prev.parseErrors) {
        flags |= (uint8_t)BLE_GPS_FLAG_PARSE_ERROR;
    }
    if (now.uartOverflows != prev.uartOverflows) {
        flags |= (uint8_t)BLE_GPS_FLAG_UART_OVERFLOW;
    }
    return flags;
}

inline void gpsPutU32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)(v >> 24);
}

/**
 * @brief Writes one GPS block for fix. deviceTimeMs is the node time the NAV-PVT was
 * complete (fix.rxTimeMs); iTOW stays on the node. `flags` comes from gpsBlockFlags().
 * @return GPS_BLOCK_BYTES, or 0 if outCap is too small (then out is left untouched).
 */
inline size_t packGpsBlock(const GpsFix& fix, uint8_t seq, uint8_t flags, uint8_t* out, size_t outCap) {
    if (out == nullptr || outCap < GPS_BLOCK_BYTES) {
        return 0;
    }
    out[BLE_GPS_VERSION_OFFSET] = GPS_BLOCK_VERSION;
    out[BLE_GPS_SEQ_OFFSET] = seq;
    gpsPutU32(&out[BLE_GPS_DEVICE_TIME_MS_OFFSET], fix.rxTimeMs);
    gpsPutU32(&out[BLE_GPS_GROUND_SPEED_OFFSET], (uint32_t)fix.groundSpeedMmps);
    gpsPutU32(&out[BLE_GPS_HEADING_OF_MOTION_OFFSET], (uint32_t)fix.headingMotionE5);
    gpsPutU32(&out[BLE_GPS_SPEED_ACCURACY_OFFSET], fix.speedAccMmps);
    gpsPutU32(&out[BLE_GPS_HEADING_ACCURACY_OFFSET], fix.headingAccE5);
    out[BLE_GPS_FIX_TYPE_OFFSET] = fix.fixType;
    out[BLE_GPS_NUM_SV_OFFSET] = fix.numSv;
    out[BLE_GPS_FLAGS_OFFSET] = flags;
    out[BLE_GPS_RESERVED_OFFSET] = 0u;
    return GPS_BLOCK_BYTES;
}

#endif // GPS_BLOCK_PACKET_H
