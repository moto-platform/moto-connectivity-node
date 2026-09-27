#ifndef TESTER_LATCH_H
#define TESTER_LATCH_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Q-018 / D-030 -- the vehicle-bus poller's latch and bus-off count, kept across
 * resets so a watchdog, panic or brownout reset cannot silently put this node back on
 * the bus as a second tester (D-021) or reset the bus-off budget.
 *
 * Header-only and hardware-independent: the record codec and the restore rule below are
 * unit-tested on the host. On the ESP32 the record lives in RTC no-init memory
 * (RtcTesterLatchStore): it keeps its content across software/watchdog/panic resets and
 * is undefined after a power-on reset.
 *
 * Fail-safe restore rule: a power-on reset writes a valid ARMED record. After any other
 * reset the record must decode as ARMED or as a latch; anything else (corrupted, lost,
 * never written by this firmware) is treated as latched with reason UNKNOWN.
 */
enum class TesterLatchReason : uint8_t {
    NONE = 0,           // not latched: the poller may run
    FOREIGN_TESTER = 1,
    BUS_OFF = 2,
    UNKNOWN = 3,        // record invalid after a non-power-on reset: latched, cause unknown
};

struct TesterLatchStatus {
    TesterLatchReason reason;
    uint32_t busOffCount; // bus-off events since the last power-on (D-030 budget)
};

struct TesterLatchRecord {
    uint32_t magic;
    uint32_t state;         // kStateArmed or a latched TesterLatchReason
    uint32_t stateInverted; // ~state: a stuck-at or zeroed word cannot decode as valid
    uint32_t busOffCount;
    uint32_t crc;           // CRC-32 over the four words above
};

namespace tester_latch {

const uint32_t kMagic = 0x4C415443u;     // "LATC"
const uint32_t kStateArmed = 0x41524Du;  // "ARM": deliberately not 0 or a small number

// CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320). Bitwise: 16 bytes, rarely called.
inline uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
        }
    }
    return ~crc;
}

inline uint32_t recordCrc(const TesterLatchRecord& r) {
    const uint32_t words[4] = {r.magic, r.state, r.stateInverted, r.busOffCount};
    return crc32(reinterpret_cast<const uint8_t*>(words), sizeof(words));
}

// reason NONE writes an ARMED record. UNKNOWN is never written: it only comes from decode.
inline void encode(TesterLatchReason reason, uint32_t busOffCount, TesterLatchRecord& r) {
    r.magic = kMagic;
    r.state = (reason == TesterLatchReason::NONE) ? kStateArmed : (uint32_t)reason;
    r.stateInverted = ~r.state;
    r.busOffCount = busOffCount;
    r.crc = recordCrc(r);
}

inline void clear(TesterLatchRecord& r) {
    r.magic = 0;
    r.state = 0;
    r.stateInverted = 0;
    r.busOffCount = 0;
    r.crc = 0;
}

// True only if every check passes and the state is ARMED or a known latch reason.
inline bool decode(const TesterLatchRecord& r, TesterLatchStatus& out) {
    if (r.magic != kMagic || r.stateInverted != ~r.state || r.crc != recordCrc(r)) {
        return false;
    }
    out.busOffCount = r.busOffCount;
    if (r.state == kStateArmed) {
        out.reason = TesterLatchReason::NONE;
    } else if (r.state == (uint32_t)TesterLatchReason::FOREIGN_TESTER) {
        out.reason = TesterLatchReason::FOREIGN_TESTER;
    } else if (r.state == (uint32_t)TesterLatchReason::BUS_OFF) {
        out.reason = TesterLatchReason::BUS_OFF;
    } else {
        return false;
    }
    return true;
}

// The boot-time rule shared by the RTC store and the test store (see the file comment).
inline TesterLatchStatus restore(TesterLatchRecord& r, bool powerOnReset) {
    TesterLatchStatus status = {TesterLatchReason::NONE, 0};
    if (powerOnReset) {
        encode(TesterLatchReason::NONE, 0, r);
        return status;
    }
    if (!decode(r, status)) {
        status.reason = TesterLatchReason::UNKNOWN;
        status.busOffCount = 0;
    }
    return status;
}

} // namespace tester_latch

/**
 * @brief Where HondaCANModule keeps its latch across resets. main.cpp injects the RTC
 * no-init store on the ESP32; the native tests inject an in-memory record.
 */
class ITesterLatchStore {
public:
    virtual ~ITesterLatchStore() {}
    // Called once per boot, before anything else (applies tester_latch::restore()).
    virtual TesterLatchStatus load() = 0;
    // Writes the full state: reason NONE = armed (not latched).
    virtual void save(TesterLatchReason reason, uint32_t busOffCount) = 0;
};

#endif // TESTER_LATCH_H
