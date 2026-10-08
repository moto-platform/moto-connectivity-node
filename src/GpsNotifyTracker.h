#ifndef GPS_NOTIFY_TRACKER_H
#define GPS_NOTIFY_TRACKER_H

#include <stdint.h>

#include "GpsBlockPacket.h"
#include "UbxConfig.h"

// One GPS block per NAV-PVT (ble_schema.json gpsBlock.source): BLE_GPS_NOTIFY_PERIOD_MS is
// the PVT rate the receiver is configured for, not a send timer.
static_assert(ubx::kMeasRateMs == BLE_GPS_NOTIFY_PERIOD_MS, "the GPS block rate is the NAV-PVT rate (D-060)");

// Minimum spacing between two blocks. A strict 100 ms spacing against a 10 Hz source would
// turn the PVT jitter into a random walk of the delay and drop PVTs; half the period only
// spreads PVTs that arrive bunched. A held PVT goes out on a later pass, it is never dropped.
constexpr uint32_t GPS_BLOCK_MIN_SPACING_MS = BLE_GPS_NOTIFY_PERIOD_MS / 2u;

enum class GpsNotifyAction : uint8_t {
    NONE,     // no new NAV-PVT, held for the spacing, not connected, or no fix ever
    SEND,     // pack the block with this seq and flags and notify it
    SKIP_MTU, // MTU - 3 < GPS_BLOCK_BYTES: not sent, but seq and the error base advanced
};

struct GpsNotifyDecision {
    GpsNotifyAction action;
    uint8_t seq;
    uint8_t flags;
};

/**
 * @brief Decides, once per BLE pass, whether the latest NAV-PVT becomes a GPS block
 * (ble_schema.json gpsBlock, D-060). Pure: the caller passes the mirrored GPS state.
 *
 * - A new NAV-PVT is detected by navPvtCount, not by the fix time. The 50 ms mirror can merge
 *   PVTs (delta > 1); seq then advances by the delta (mod 256) so the loss is a seq gap.
 * - Below the MTU the block is skipped, yet seq and the error base advance as for a sent one
 *   (gpsBlock.sequenceRule / mtuRule): a suspension shows up as a seq gap.
 * - The error flags cover the counter change since the previous block, sent or skipped; arm()
 *   sets the base to the counters at connection time, so nothing older than the connection
 *   is reported.
 * - No block before the first NAV-PVT (hasFix false). Blocks without a usable fix (fixType
 *   0-2, gnssFixOk 0) are sent; the receiver filters them.
 */
class GpsNotifyTracker {
public:
    // A connection came up: blocks start with the next NAV-PVT.
    void arm(uint32_t navPvtCount, const GpsErrorCounters& errors) {
        _armed = true;
        _produced = false;
        _lastPvtCount = navPvtCount;
        _prevErrors = errors;
    }

    // The connection dropped: forget the last-block state. seq keeps rolling, like the
    // telemetry and IMU counters.
    void disarm() {
        _armed = false;
        _produced = false;
    }

    // payloadLimit = blePayloadLimit(mtu) of the current peer.
    GpsNotifyDecision step(bool hasFix, const GpsFix& fix, uint32_t navPvtCount,
                           const GpsErrorCounters& errors, uint32_t nowMs, uint16_t payloadLimit) {
        GpsNotifyDecision d = {GpsNotifyAction::NONE, 0u, 0u};
        if (!_armed || !hasFix) {
            return d;
        }
        const uint32_t newPvts = navPvtCount - _lastPvtCount; // wraps like the counter
        if (newPvts == 0u) {
            return d;
        }
        if (_produced && ((nowMs - _lastBlockMs) < GPS_BLOCK_MIN_SPACING_MS)) {
            return d; // held: the same PVT is still new on the next pass
        }
        _nextSeq = (uint8_t)(_nextSeq + newPvts);
        d.seq = (uint8_t)(_nextSeq - 1u);
        d.flags = gpsBlockFlags(fix, errors, _prevErrors);
        d.action = (payloadLimit >= GPS_BLOCK_BYTES) ? GpsNotifyAction::SEND : GpsNotifyAction::SKIP_MTU;
        _lastPvtCount = navPvtCount;
        _prevErrors = errors;
        _lastBlockMs = nowMs;
        _produced = true;
        return d;
    }

private:
    bool _armed = false;
    bool _produced = false; // a block was produced (sent or skipped) since arm()
    uint8_t _nextSeq = 0u;
    uint32_t _lastPvtCount = 0u;
    uint32_t _lastBlockMs = 0u;
    GpsErrorCounters _prevErrors = {0u, 0u};
};

#endif // GPS_NOTIFY_TRACKER_H
