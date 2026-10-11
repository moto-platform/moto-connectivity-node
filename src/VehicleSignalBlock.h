#ifndef VEHICLE_SIGNAL_BLOCK_H
#define VEHICLE_SIGNAL_BLOCK_H

#include <stddef.h>
#include <stdint.h>

#include "ble_schema.h"    // generated: external/moto-vehicle-defs/gen/c/conn/ (D-061)
#include "vehicle_cl250.h" // generated: the tester's DID table

// Vehicle-signal block notification (moto-vehicle-defs ble/ble_schema.json
// `vehicleSignalBlock`, D-066 item 4): one record per DID of the generated DID table, keyed by
// DID, so a DID promoted from the candidate catalog flows without a change here. Little-endian,
// written byte by byte (no struct punning):
//   header (10): version u8, seq u8, deviceTimeMs u32, totalRecords u8, firstRecord u8,
//                recordCount u8, reserved u8
//   record (9):  did u16, state u8, ageMs u16, raw u32
// raw = the DID's data bytes as an unsigned big-endian number; moto-mobile and moto-server
// decode it with the generated vehicle catalog. Nothing here touches the bus: the tester only
// writes VehicleSignalRecord, BLEServerModule only reads it (both on the loop task).

static_assert(BLE_VSIG_HEADER_VERSION_OFFSET == 0 && BLE_VSIG_HEADER_SEQ_OFFSET == 1 &&
              BLE_VSIG_HEADER_DEVICE_TIME_MS_OFFSET == 2 && BLE_VSIG_HEADER_TOTAL_RECORDS_OFFSET == 6 &&
              BLE_VSIG_HEADER_FIRST_RECORD_OFFSET == 7 && BLE_VSIG_HEADER_RECORD_COUNT_OFFSET == 8 &&
              BLE_VSIG_HEADER_RESERVED_OFFSET == 9 &&
              BLE_VSIG_HEADER_BYTES == BLE_VSIG_HEADER_RESERVED_OFFSET + BLE_VSIG_HEADER_RESERVED_SIZE,
              "vehicle-signal header order must match the schema");
static_assert(BLE_VSIG_HEADER_DEVICE_TIME_MS_SIZE == 4 && BLE_VSIG_HEADER_TOTAL_RECORDS_SIZE == 1 &&
              BLE_VSIG_HEADER_FIRST_RECORD_SIZE == 1 && BLE_VSIG_HEADER_RECORD_COUNT_SIZE == 1,
              "vehicle-signal header sizes must match the schema");
static_assert(BLE_VSIG_RECORD_DID_OFFSET == 0 && BLE_VSIG_RECORD_DID_SIZE == 2 &&
              BLE_VSIG_RECORD_STATE_OFFSET == 2 && BLE_VSIG_RECORD_STATE_SIZE == 1 &&
              BLE_VSIG_RECORD_AGE_MS_OFFSET == 3 && BLE_VSIG_RECORD_AGE_MS_SIZE == 2 &&
              BLE_VSIG_RECORD_RAW_OFFSET == 5 && BLE_VSIG_RECORD_RAW_SIZE == 4 &&
              BLE_VSIG_RECORD_BYTES == BLE_VSIG_RECORD_RAW_OFFSET + BLE_VSIG_RECORD_RAW_SIZE,
              "vehicle-signal record layout must match the schema");
static_assert(BLE_VSIG_TOTAL_BYTES_MAX == BLE_VSIG_HEADER_BYTES + BLE_VSIG_MAX_RECORDS_PER_BLOCK * BLE_VSIG_RECORD_BYTES,
              "vehicle-signal block maximum must match the schema");
static_assert(VEHICLE_CL250_DID_COUNT >= 1u && VEHICLE_CL250_DID_COUNT <= 255u,
              "the DID table must fit totalRecords/firstRecord (uint8)");

constexpr size_t VSIG_BLOCK_MAX_BYTES = BLE_VSIG_TOTAL_BYTES_MAX;

/**
 * @brief What the tester last saw for one DID of its table (index = generated table index).
 * Written by HondaCANModule only, read by BLEServerModule; both run on the loop task.
 */
struct VehicleSignalRecord {
    uint32_t raw = 0;         // data bytes of the last positive response, unsigned big-endian
    uint32_t updatedMs = 0;   // millis() of that response (valid only if hasValue)
    bool hasValue = false;    // a positive response since boot
    bool lastWasNrc = false;  // the DID's last answer was a negative response (not 0x78)
    uint8_t nrc = 0;          // that NRC
    bool skipped = false;     // the tester skips the DID after repeated timeouts (cool-down)
};

struct VehicleSignalStore {
    VehicleSignalRecord dids[VEHICLE_CL250_DID_COUNT];
};

/**
 * @brief Records a positive response. `data` = the bytes after the DID echo, `length` = the
 * DID's generated length. A length the record cannot hold (> 4) is ignored: defs codegen
 * refuses such a DID, so it cannot occur with a generated table.
 */
inline void vsigRecordPositive(VehicleSignalRecord& r, const uint8_t* data, uint8_t length, uint32_t now) {
    if (data == nullptr || length == 0u || length > BLE_VSIG_RECORD_RAW_SIZE) {
        return;
    }
    uint32_t raw = 0u;
    for (uint8_t i = 0; i < length; i++) {
        raw = (raw << 8) | data[i];
    }
    r.raw = raw;
    r.updatedMs = now;
    r.hasValue = true;
    r.lastWasNrc = false;
}

/** @brief Records a negative response (not 0x78) the tester attributed to this DID's read. */
inline void vsigRecordNrc(VehicleSignalRecord& r, uint8_t nrc) {
    r.lastWasNrc = true;
    r.nrc = nrc;
}

/** @brief ageMs of a record (ble_schema.json `ageRule`). */
inline uint16_t vsigAgeMs(const VehicleSignalRecord& r, uint32_t now) {
    if (!r.hasValue) {
        return (uint16_t)BLE_TELEMETRY_AGE_NEVER_RECEIVED;
    }
    uint32_t age = now - r.updatedMs;
    return age > BLE_TELEMETRY_AGE_MAX ? (uint16_t)BLE_TELEMETRY_AGE_MAX : (uint16_t)age;
}

/** @brief state of a record (ble_schema.json `stateRule`: the first that applies). */
inline uint8_t vsigState(const VehicleSignalRecord& r, uint16_t staleAfterMs, uint32_t now) {
    if (r.skipped) {
        return (uint8_t)BLE_VSIG_STATE_SKIPPED;
    }
    if (r.lastWasNrc) {
        return (uint8_t)BLE_VSIG_STATE_NEGATIVE_RESPONSE;
    }
    if (r.hasValue) {
        // Same boundary as isStale(): valid while the age is within stale_after_ms.
        return (now - r.updatedMs) <= staleAfterMs ? (uint8_t)BLE_VSIG_STATE_VALID : (uint8_t)BLE_VSIG_STATE_STALE;
    }
    return (uint8_t)BLE_VSIG_STATE_NEVER_RECEIVED;
}

/** @brief raw field of a record for its state (ble_schema.json `rawRule`). */
inline uint32_t vsigRawField(const VehicleSignalRecord& r, uint8_t state) {
    if (state == BLE_VSIG_STATE_VALID || state == BLE_VSIG_STATE_STALE) {
        return r.raw;
    }
    if (state == BLE_VSIG_STATE_NEGATIVE_RESPONSE) {
        return r.nrc;
    }
    return 0u;
}

/** @brief Records per block for a payload limit (MTU - 3); 0 if not even one fits. */
inline uint8_t vsigRecordsPerBlock(uint16_t payloadLimit) {
    if (payloadLimit < BLE_VSIG_HEADER_BYTES + BLE_VSIG_RECORD_BYTES) {
        return 0u;
    }
    uint32_t n = (payloadLimit - BLE_VSIG_HEADER_BYTES) / BLE_VSIG_RECORD_BYTES;
    return n > BLE_VSIG_MAX_RECORDS_PER_BLOCK ? (uint8_t)BLE_VSIG_MAX_RECORDS_PER_BLOCK : (uint8_t)n;
}

inline void vsigPutU16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

inline void vsigPutU32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)(v >> 24);
}

/**
 * @brief Writes one block with records first..first+count-1 of the DID table.
 * @return the block length, or 0 (out untouched) for an empty or out-of-table range or a
 * buffer that is too small.
 */
inline size_t packVehicleSignalBlock(const VehicleSignalStore& store, uint8_t first, uint8_t count, uint8_t seq,
                                     uint32_t now, uint8_t* out, size_t outCap) {
    const size_t len = BLE_VSIG_HEADER_BYTES + (size_t)count * BLE_VSIG_RECORD_BYTES;
    if (out == nullptr || count == 0u || count > BLE_VSIG_MAX_RECORDS_PER_BLOCK ||
        (uint16_t)first + count > VEHICLE_CL250_DID_COUNT || outCap < len) {
        return 0;
    }
    out[BLE_VSIG_HEADER_VERSION_OFFSET] = (uint8_t)BLE_VSIG_BLOCK_VERSION;
    out[BLE_VSIG_HEADER_SEQ_OFFSET] = seq;
    vsigPutU32(&out[BLE_VSIG_HEADER_DEVICE_TIME_MS_OFFSET], now);
    out[BLE_VSIG_HEADER_TOTAL_RECORDS_OFFSET] = (uint8_t)VEHICLE_CL250_DID_COUNT;
    out[BLE_VSIG_HEADER_FIRST_RECORD_OFFSET] = first;
    out[BLE_VSIG_HEADER_RECORD_COUNT_OFFSET] = count;
    out[BLE_VSIG_HEADER_RESERVED_OFFSET] = 0u;
    for (uint8_t k = 0; k < count; k++) {
        const uint8_t i = (uint8_t)(first + k);
        const VehicleSignalRecord& r = store.dids[i];
        const uint8_t state = vsigState(r, vehicle_cl250_dids[i].stale_after_ms, now);
        uint8_t* rec = &out[BLE_VSIG_HEADER_BYTES + (size_t)k * BLE_VSIG_RECORD_BYTES];
        vsigPutU16(&rec[BLE_VSIG_RECORD_DID_OFFSET], vehicle_cl250_dids[i].did);
        rec[BLE_VSIG_RECORD_STATE_OFFSET] = state;
        vsigPutU16(&rec[BLE_VSIG_RECORD_AGE_MS_OFFSET], vsigAgeMs(r, now));
        vsigPutU32(&rec[BLE_VSIG_RECORD_RAW_OFFSET], vsigRawField(r, state));
    }
    return len;
}

/** @brief One notify period of the paging rule (ble_schema.json `pagingRule`, `mtuRule`). */
struct VsigPeriodPlan {
    uint8_t blocks = 0;                                     // blocks to send this period
    uint8_t first[BLE_VSIG_MAX_BLOCKS_PER_PERIOD] = {};     // table index of each block's first record
    uint8_t count[BLE_VSIG_MAX_BLOCKS_PER_PERIOD] = {};     // records in each block
    uint8_t next = 0;                                       // where the next period starts
};

/**
 * @brief Plans one period from table index `next`: blocks of at most `perBlock` records that
 * end at the table end (no wrap inside a block), until the table is covered once or
 * BLE_VSIG_MAX_BLOCKS_PER_PERIOD blocks are planned. perBlock 0 plans nothing.
 */
inline VsigPeriodPlan planVehicleSignalPeriod(uint8_t next, uint8_t perBlock) {
    VsigPeriodPlan plan;
    const uint8_t total = (uint8_t)VEHICLE_CL250_DID_COUNT;
    plan.next = next < total ? next : 0u;
    if (perBlock == 0u) {
        return plan;
    }
    uint16_t covered = 0;
    while (plan.blocks < BLE_VSIG_MAX_BLOCKS_PER_PERIOD && covered < total) {
        const uint8_t first = plan.next;
        const uint8_t toEnd = (uint8_t)(total - first);
        const uint8_t uncovered = (uint8_t)(total - covered);
        uint8_t n = perBlock < toEnd ? perBlock : toEnd;
        n = n < uncovered ? n : uncovered; // each record at most once per period
        plan.first[plan.blocks] = first;
        plan.count[plan.blocks] = n;
        plan.blocks++;
        covered += n;
        plan.next = (uint8_t)((first + n) % total);
    }
    return plan;
}

#endif // VEHICLE_SIGNAL_BLOCK_H
