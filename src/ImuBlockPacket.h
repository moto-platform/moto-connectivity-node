#ifndef IMU_BLOCK_PACKET_H
#define IMU_BLOCK_PACKET_H

#include <stdint.h>
#include <stddef.h>

#include "BLETelemetryPacket.h" // blePayloadLimit()
#include "ImuRingBuffer.h"

// IMU block notification (docs/ble_telemetry_packet_schema.json `imuBlock`, D-032).
// Little-endian, written byte by byte (no struct punning):
//   header (12): version u8, seq u8, deviceTimeMs u32, firstSampleIndex u16,
//                sampleCount u8, samplePeriodMs u8, flags u8, reserved u8
//   sample (12): ax, ay, az, gx, gy, gz as int16
constexpr uint8_t IMU_BLOCK_VERSION = 1;
constexpr uint8_t IMU_BLOCK_HEADER_BYTES = 12;
constexpr uint8_t IMU_BLOCK_SAMPLE_BYTES = 12;
constexpr uint8_t IMU_BLOCK_MAX_SAMPLES = 10;
constexpr uint8_t IMU_SAMPLE_PERIOD_MS = 10;
constexpr size_t IMU_BLOCK_MAX_BYTES = IMU_BLOCK_HEADER_BYTES + IMU_BLOCK_MAX_SAMPLES * IMU_BLOCK_SAMPLE_BYTES;
// Upper bound of blocks per 100 ms notify period, so a small MTU lowers the delivered IMU
// rate instead of making the main loop (and the CAN poller in it) do more work.
constexpr uint8_t IMU_MAX_BLOCKS_PER_NOTIFY = 3;

static_assert(IMU_BLOCK_MAX_BYTES == 132, "IMU block layout must match the schema");

// Samples per block for a negotiated MTU; 0 = IMU notifications suspended (MTU too small).
inline uint8_t imuSamplesPerBlock(uint16_t mtu) {
    uint16_t limit = blePayloadLimit(mtu);
    if (limit < IMU_BLOCK_HEADER_BYTES + IMU_BLOCK_SAMPLE_BYTES) {
        return 0;
    }
    uint16_t n = (uint16_t)((limit - IMU_BLOCK_HEADER_BYTES) / IMU_BLOCK_SAMPLE_BYTES);
    return n > IMU_BLOCK_MAX_SAMPLES ? IMU_BLOCK_MAX_SAMPLES : (uint8_t)n;
}

inline void imuPutU16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

inline void imuPutU32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)(v >> 24);
}

/**
 * @brief Moves up to maxSamples consecutive samples (tick indices without a gap) from the
 * ring into one block. A block never spans an index gap, so the receiver can place every
 * sample at deviceTimeMs + i * IMU_SAMPLE_PERIOD_MS. `flags` are IMU_EVENT_* bits.
 * @return bytes written to out, 0 when there is nothing to send, maxSamples is 0, or
 *         outCap is too small (then nothing is consumed).
 */
template <uint32_t N>
size_t packImuBlock(ImuRingBuffer<N>& ring, uint8_t maxSamples, uint8_t seq, uint8_t flags,
                    uint8_t* out, size_t outCap) {
    uint32_t available = ring.size();
    if (available == 0 || maxSamples == 0) {
        return 0;
    }
    if (maxSamples > IMU_BLOCK_MAX_SAMPLES) {
        maxSamples = IMU_BLOCK_MAX_SAMPLES;
    }
    uint32_t count = available < maxSamples ? available : maxSamples;
    const ImuSample& first = ring.peek(0);
    for (uint32_t i = 1; i < count; i++) {
        if (ring.peek(i).index != first.index + i) {
            count = i; // stop at the first gap
            break;
        }
    }
    size_t len = IMU_BLOCK_HEADER_BYTES + count * IMU_BLOCK_SAMPLE_BYTES;
    if (outCap < len) {
        return 0;
    }

    out[0] = IMU_BLOCK_VERSION;
    out[1] = seq;
    imuPutU32(&out[2], first.timeMs);
    imuPutU16(&out[6], (uint16_t)(first.index & 0xFFFF));
    out[8] = (uint8_t)count;
    out[9] = IMU_SAMPLE_PERIOD_MS;
    out[10] = flags;
    out[11] = 0;

    uint8_t* p = &out[IMU_BLOCK_HEADER_BYTES];
    for (uint32_t i = 0; i < count; i++) {
        const ImuSample& s = ring.peek(i);
        for (int axis = 0; axis < 3; axis++) {
            imuPutU16(p, (uint16_t)s.accel[axis]);
            p += 2;
        }
        for (int axis = 0; axis < 3; axis++) {
            imuPutU16(p, (uint16_t)s.gyro[axis]);
            p += 2;
        }
    }
    ring.drop(count);
    return len;
}

#endif // IMU_BLOCK_PACKET_H
