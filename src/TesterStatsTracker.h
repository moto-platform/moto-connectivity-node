#ifndef TESTER_STATS_TRACKER_H
#define TESTER_STATS_TRACKER_H

#include <stdint.h>

#include "SystemState.h"

// D-058 items 1-2: counters about the tester's own step and its ECU round trips. Pure
// bookkeeping on fixed arrays (no heap, no I/O) so the native tests can drive it with a fake
// clock. HondaCANModule feeds it and copies stats() into SystemState; nothing here can
// transmit or change the poller's behaviour.
class TesterStatsTracker {
public:
    /**
     * @brief Call at every entry of the poller step with a microsecond timestamp (wraps at
     * 2^32 us, the subtraction is wrap-safe). The first entry only sets the reference. Later
     * entries record the gap since the previous one in ms, rounded up (D-053's definition):
     * the maximum since boot and the count of gaps above client_step_max_ms, both saturating
     * at 65535.
     */
    void onStepEntry(uint32_t nowUs) {
        if (_haveLastEntry) {
            const uint32_t gapUs = nowUs - _lastEntryUs;
            const uint32_t gapMs = gapUs / 1000u + ((gapUs % 1000u) != 0u ? 1u : 0u);
            const uint16_t gap16 = gapMs > 0xFFFFu ? (uint16_t)0xFFFFu : (uint16_t)gapMs;
            if (gap16 > _stats.stepGapMaxMs) {
                _stats.stepGapMaxMs = gap16;
            }
            if (gapMs > VEHICLE_CL250_CLIENT_STEP_MAX_MS && _stats.stepGapOverCount != 0xFFFFu) {
                _stats.stepGapOverCount++;
            }
        }
        _lastEntryUs = nowUs;
        _haveLastEntry = true;
    }

    /**
     * @brief One round-trip sample (ms) for the DID at generated table index `didIndex`:
     * a positive response or an NRC other than 0x78 resolved the request. min/max/sum/count
     * saturate. Out-of-range indices are ignored.
     */
    void recordRoundTrip(uint8_t didIndex, uint32_t rttMs) {
        if (didIndex >= VEHICLE_CL250_DID_COUNT) {
            return;
        }
        DidRoundTripStats& r = _stats.rtt[didIndex];
        const uint16_t ms16 = rttMs > 0xFFFFu ? (uint16_t)0xFFFFu : (uint16_t)rttMs;
        // 65535 is the "no sample" marker of the minimum; a saturated sample keeps it
        // unambiguous by staying below it.
        const uint16_t minCandidate = ms16 == TESTER_RTT_NONE_MS ? (uint16_t)(TESTER_RTT_NONE_MS - 1) : ms16;
        if (r.count == 0 || minCandidate < r.minMs) {
            r.minMs = minCandidate;
        }
        if (ms16 > r.maxMs) {
            r.maxMs = ms16;
        }
        r.sumMs = (rttMs > 0xFFFFFFFFu - r.sumMs) ? 0xFFFFFFFFu : r.sumMs + rttMs;
        if (r.count != 0xFFFFFFFFu) {
            r.count++;
        }
    }

    /** @brief A request of this DID saw NRC 0x78 (call once per request). */
    void recordNrc78(uint8_t didIndex) {
        if (didIndex >= VEHICLE_CL250_DID_COUNT) {
            return;
        }
        DidRoundTripStats& r = _stats.rtt[didIndex];
        if (r.nrc78Count != 0xFFFFu) {
            r.nrc78Count++;
        }
    }

    const TesterStats& stats() const { return _stats; }

#ifdef CONN_NATIVE_TEST
    // Test hook: preloads a DID's statistics to reach the 32-bit saturation (never built on target).
    void testSetRoundTrip(uint8_t didIndex, const DidRoundTripStats& r) { _stats.rtt[didIndex] = r; }
#endif

private:
    TesterStats _stats;
    uint32_t _lastEntryUs = 0;
    bool _haveLastEntry = false;
};

#endif // TESTER_STATS_TRACKER_H
