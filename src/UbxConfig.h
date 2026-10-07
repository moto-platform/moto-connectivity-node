#ifndef UBX_CONFIG_H
#define UBX_CONFIG_H

#include <stddef.h>
#include <stdint.h>

#include "UbxParser.h"

/**
 * @brief UBX-CFG frames that put the NEO-M8N into the D-060 mode: UBX only on its UART1
 * (NMEA off, invariant 5), NAV-PVT every solution, 10 Hz. Built into caller buffers with
 * the Fletcher checksum; no heap. Settings go to the receiver's RAM only (no CFG-CFG save),
 * so GpsModule sends them on every start.
 */
namespace ubx {

constexpr uint8_t kClassCfg = 0x06u;
constexpr uint8_t kIdCfgPrt = 0x00u;
constexpr uint8_t kIdCfgMsg = 0x01u;
constexpr uint8_t kIdCfgRate = 0x08u;

constexpr uint32_t kModuleDefaultBaud = 9600u; // NEO-M8N factory default on UART1
constexpr uint32_t kGpsBaud = 38400u;          // D-060: >= 38400 for NAV-PVT at 10 Hz
static_assert(kGpsBaud >= 38400u, "D-060: NAV-PVT at 10 Hz needs at least 38400 baud");
constexpr uint16_t kMeasRateMs = 100u;          // 10 Hz
constexpr size_t kFrameOverhead = 8u;           // sync(2) + class + id + len(2) + ck(2)
constexpr size_t kMaxCfgFrame = kFrameOverhead + 20u;

// Writes a complete UBX frame; returns its length, or 0 if `cap` is too small.
inline size_t buildFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len,
                         uint8_t* out, size_t cap) {
    size_t total = kFrameOverhead + len;
    if ((out == nullptr) || (cap < total) || ((len != 0u) && (payload == nullptr))) {
        return 0u;
    }
    Fletcher8 ck;
    out[0] = kSync1;
    out[1] = kSync2;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)(len & 0xFFu);
    out[5] = (uint8_t)(len >> 8u);
    for (size_t i = 2u; i < 6u; i++) {
        ck.add(out[i]);
    }
    for (uint16_t i = 0u; i < len; i++) {
        out[6u + i] = payload[i];
        ck.add(payload[i]);
    }
    out[6u + len] = ck.a;
    out[7u + len] = ck.b;
    return total;
}

// CFG-PRT for the receiver's UART1: 8N1 at `baud`, UBX in and out only (NMEA off).
inline size_t buildCfgPrtUart1(uint32_t baud, uint8_t* out, size_t cap) {
    const uint8_t p[20] = {
        0x01u, 0x00u,                         // portID UART1, reserved
        0x00u, 0x00u,                         // txReady off
        0xC0u, 0x08u, 0x00u, 0x00u,           // mode: 8 bit, no parity, 1 stop bit
        (uint8_t)baud, (uint8_t)(baud >> 8u), (uint8_t)(baud >> 16u), (uint8_t)(baud >> 24u),
        0x01u, 0x00u,                         // inProtoMask: UBX only
        0x01u, 0x00u,                         // outProtoMask: UBX only (NMEA off)
        0x00u, 0x00u,                         // flags
        0x00u, 0x00u,                         // reserved
    };
    return buildFrame(kClassCfg, kIdCfgPrt, p, sizeof(p), out, cap);
}

// CFG-RATE: one navigation solution every kMeasRateMs, GPS time reference.
inline size_t buildCfgRate(uint8_t* out, size_t cap) {
    const uint8_t p[6] = {
        (uint8_t)kMeasRateMs, (uint8_t)(kMeasRateMs >> 8u),
        0x01u, 0x00u, // navRate: every measurement
        0x01u, 0x00u, // timeRef: GPS
    };
    return buildFrame(kClassCfg, kIdCfgRate, p, sizeof(p), out, cap);
}

// CFG-MSG: NAV-PVT on every solution on the port the command arrives on.
inline size_t buildCfgMsgNavPvt(uint8_t* out, size_t cap) {
    const uint8_t p[3] = {kClassNav, kIdNavPvt, 0x01u};
    return buildFrame(kClassCfg, kIdCfgMsg, p, sizeof(p), out, cap);
}

} // namespace ubx

#endif // UBX_CONFIG_H
