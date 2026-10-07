#ifndef GPS_FIX_H
#define GPS_FIX_H

#include <stdint.h>
#include <type_traits>

/**
 * @brief The NAV-PVT values that may leave the node (D-060 item 3), raw UBX units, plus the
 * node time of reception. There is deliberately no latitude, longitude or height here
 * (invariant 7): UbxParser never copies them out, and nothing downstream can send what
 * this struct cannot hold. Raw units keep the later BLE encoding a plain copy into the
 * defs GPS block (ble_schema.json `gpsBlock`, D-061).
 */
struct GpsFix {
    uint32_t rxTimeMs = 0;        // node clock (millis) when the NAV-PVT was complete
    uint32_t iTowMs = 0;          // GPS time of week; internal only (epoch de-duplication)
    int32_t groundSpeedMmps = 0;  // gSpeed, mm/s
    int32_t headingMotionE5 = 0;  // headMot, 1e-5 deg (unreliable near standstill: see headAcc)
    uint32_t speedAccMmps = 0;    // sAcc, mm/s
    uint32_t headingAccE5 = 0;    // headAcc, 1e-5 deg
    uint8_t fixType = 0;          // NAV-PVT fixType: 0 no fix .. 5 time only
    uint8_t numSv = 0;            // satellites used
    uint8_t gnssFixOk = 0;        // NAV-PVT flags bit 0
    uint8_t reserved = 0;
};

static_assert(std::is_trivially_copyable<GpsFix>::value, "GpsFix crosses cores via a seqlock");
static_assert(sizeof(GpsFix) == 28u, "GpsFix layout: 6 x 4 bytes + 4 x 1 byte, no position");

#endif // GPS_FIX_H
