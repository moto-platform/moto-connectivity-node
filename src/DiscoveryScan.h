#ifndef DISCOVERY_SCAN_H
#define DISCOVERY_SCAN_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "uds_iso14229.h" // generated: D-040 client subset (gen/c/conn/)
#include "vehicle_cl250.h" // generated: discovery_scan table (D-059 item 2)

// MISRA C:2012 17.1 / 21.6 deviation (conn reports MISRA only, D-046 item 3): <stdarg.h> and
// vsnprintf/snprintf format the probe's serial report into one static buffer. Only integer,
// %c and %s conversions with literal formats are used; no heap, no input parsing.
static_assert(VEHICLE_CL250_SCAN_REQUEST_MAX <= 3, "report ids assume requests of at most 3 bytes");

/**
 * @brief D-059 item 3: the one-shot discovery scan of conn's probe env. Walks the generated
 * vehicle_cl250_discovery_scan[] once, in table order, and reports every answer as text
 * lines (USB serial in the firmware). Pure logic: it never transmits. HondaCANModule sends
 * each request through the D-020 frame gate, one request in flight, and feeds the answers
 * back. TEMPORARY, like conn's tester (D-023).
 *
 * The requests and their order, the support bitmaps and the `after` gates all come from
 * defs. An entry whose `after` gate did not answer positively with a bitmap marking
 * `after_id` (NRC, no answer, aborted, short bitmap) is SKIPPED, and so are its dependents.
 *
 * VIN (D-059 item 3, D-033): only the WMI (its first 3 characters) is ever printed. The VIN
 * entry is found at run time by its SAE J1979 request (service 0x09, infotype 0x02; gen/c
 * names the table entries only in comments). Fail-closed: if not exactly one entry matches,
 * no answer of service 0x09 other than the support bitmaps is printed as data, only its
 * length. A sensitive entry's answer is never hex-dumped, also when aborted.
 * ISSUES: a `sensitive` flag on the defs entry would replace the local constants.
 */
class DiscoveryScan {
public:
    enum class Status : uint8_t { PENDING, POSITIVE, NRC, NO_ANSWER, ABORTED, FF_REFUSED, SKIPPED };
    typedef void (*LineSink)(const char* line);

    // SAE J1979 (ISO 15031-5) standard values, not vehicle signals (architecture-guard).
    static const uint8_t kJ1979ServiceVehicleInfo = 0x09;
    static const uint8_t kJ1979InfotypeVin = 0x02;
    static const uint8_t kVinLength = 17;
    static const uint8_t kWmiLength = 3;
    static const uint8_t kBitmapBytes = 4;
    static const uint8_t kBitmapIds = 32;
    static const uint8_t kNone = 0xFF;
    static const uint8_t kHexBytesPerLine = 32;

    explicit DiscoveryScan(LineSink sink)
        : _sink(sink), _vinIndex(findVinIndex(vehicle_cl250_discovery_scan, VEHICLE_CL250_SCAN_COUNT)) {
        for (uint8_t i = 0; i < VEHICLE_CL250_SCAN_COUNT; i++) {
            _status[i] = Status::PENDING;
            _hasBitmap[i] = false;
        }
    }

    // The one entry requesting the VIN, or kNone if not exactly one does (fail-closed: then
    // every non-bitmap answer of service 0x09 is sensitive).
    static uint8_t findVinIndex(const vehicle_cl250_scan_t* table, uint8_t count) {
        uint8_t found = kNone;
        uint8_t matches = 0;
        for (uint8_t i = 0; i < count; i++) {
            const vehicle_cl250_scan_t& e = table[i];
            if (e.size == 2 && e.request[0] == kJ1979ServiceVehicleInfo && e.request[1] == kJ1979InfotypeVin) {
                found = i;
                matches++;
            }
        }
        return (matches == 1) ? found : kNone;
    }

    void setSink(LineSink sink) { _sink = sink; }
#ifdef CONN_NATIVE_TEST
    void testSetVinIndex(uint8_t i) { _vinIndex = i; }
#endif

    // Banner; `sessionConfirmed` = the extended session was confirmed before the scan.
    void begin(bool sessionConfirmed) {
        _started = true;
        emitf("[SCAN] D-059 discovery scan: %u requests, session %s, one request in flight",
              (unsigned)VEHICLE_CL250_SCAN_COUNT, sessionConfirmed ? "confirmed" : "NOT confirmed");
    }

    bool started() const { return _started; }
    bool done() const { return _done; }

    /**
     * @brief The next entry to send, in table order, or -1 once the list is done (the
     * summary is then printed, once). Entries whose `after` gate says no are SKIPPED here.
     */
    int next() {
        while (_cursor < VEHICLE_CL250_SCAN_COUNT) {
            uint8_t i = _cursor;
            if (_status[i] != Status::PENDING) {
                _cursor++;
                continue;
            }
            const vehicle_cl250_scan_t& e = vehicle_cl250_discovery_scan[i];
            if (e.after != VEHICLE_CL250_SCAN_NONE && !gateSupports(e.after, e.after_id)) {
                _status[i] = Status::SKIPPED;
                emitPrefix(i, "skipped (%02u/%02u does not mark 0x%02X supported)", (unsigned)(e.after + 1u),
                           (unsigned)VEHICLE_CL250_SCAN_COUNT, e.after_id);
                _cursor++;
                continue;
            }
            return i;
        }
        if (!_done) {
            _done = true;
            emitSummary();
        }
        return -1;
    }

    static const vehicle_cl250_scan_t& entry(uint8_t i) { return vehicle_cl250_discovery_scan[i]; }
    static uint8_t positiveSid(uint8_t i) { return (uint8_t)(entry(i).request[0] + UDS_POSITIVE_RESPONSE_OFFSET); }

    // Bytes after the SID a positive answer echoes: the DID (2) for 0x22, otherwise the
    // PID / infotype / report type (1). ISO 14229-1 and SAE J1979.
    static uint8_t echoLength(uint8_t i) {
        const vehicle_cl250_scan_t& e = entry(i);
        uint8_t n = (e.request[0] == UDS_SID_READ_DATA_BY_IDENTIFIER) ? 2u : 1u;
        return (n < e.size) ? n : (uint8_t)(e.size - 1u);
    }

    // `msg` starts at the response SID: positive SID and echo of entry i's request.
    static bool matchesPositive(uint8_t i, const uint8_t* msg, uint16_t len) {
        uint8_t echo = echoLength(i);
        if (len < (uint16_t)(1u + echo) || msg[0] != positiveSid(i)) {
            return false;
        }
        for (uint8_t k = 0; k < echo; k++) {
            if (msg[1 + k] != entry(i).request[1 + k]) {
                return false;
            }
        }
        return true;
    }

    // `msg` = [0x7F][echoed SID][NRC]: a negative answer to entry i's service.
    static bool matchesNegative(uint8_t i, const uint8_t* msg, uint16_t len) {
        return len >= UDS_NEGATIVE_RESPONSE_LEN && msg[0] == UDS_SID_NEGATIVE_RESPONSE && msg[1] == entry(i).request[0];
    }

    bool sensitive(uint8_t i) const {
        if (_vinIndex != kNone) {
            return i == _vinIndex;
        }
        return entry(i).request[0] == kJ1979ServiceVehicleInfo && entry(i).bitmap_offset == 0;
    }

    uint8_t vinIndex() const { return _vinIndex; }
    Status status(uint8_t i) const { return _status[i]; }

    // A positive answer (`msg` from the response SID, matchesPositive() true).
    void recordPositive(uint8_t i, const uint8_t* msg, uint16_t len) {
        _status[i] = Status::POSITIVE;
        const vehicle_cl250_scan_t& e = entry(i);
        if (e.bitmap_offset != 0 && len >= (uint16_t)(e.bitmap_offset + kBitmapBytes)) {
            for (uint8_t k = 0; k < kBitmapBytes; k++) {
                _bitmap[i][k] = msg[e.bitmap_offset + k];
            }
            _hasBitmap[i] = true;
        }
        if (sensitive(i)) {
            reportSensitive(i, msg, len);
            return;
        }
        emitPrefix(i, "positive, %u bytes", (unsigned)len);
        emitHex(msg, len);
        if (_hasBitmap[i]) {
            emitSupported(i);
        } else if (e.bitmap_offset != 0) {
            emitPrefix(i, "bitmap missing (answer shorter than %u bytes)", (unsigned)(e.bitmap_offset + kBitmapBytes));
        }
    }

    void recordNrc(uint8_t i, uint8_t nrc) {
        _status[i] = Status::NRC;
        emitPrefix(i, "negative response NRC 0x%02X", nrc);
    }

    void recordNoAnswer(uint8_t i) {
        _status[i] = Status::NO_ANSWER;
        emitPrefix(i, "no answer (timeout)");
    }

    // A segmented answer aborted after `received` of `length` bytes; nothing of it is printed.
    void recordAborted(uint8_t i, const char* reason, uint16_t received, uint16_t length) {
        _status[i] = Status::ABORTED;
        emitPrefix(i, "reception aborted (%s) after %u of %u bytes", reason, (unsigned)received, (unsigned)length);
    }

    // A First Frame that got no FC (FF_DL outside the generated bounds, or not acceptable).
    void recordFfRefused(uint8_t i, uint16_t ffDl) {
        _status[i] = Status::FF_REFUSED;
        emitPrefix(i, "segmented answer of %u bytes refused, no FC (max %u)", (unsigned)ffDl,
                   (unsigned)VEHICLE_CL250_MAX_FF_DL);
    }

    // True if entry `gate` answered with a bitmap that marks `id` supported.
    bool gateSupports(uint8_t gate, uint8_t id) const {
        if (gate >= VEHICLE_CL250_SCAN_COUNT || _status[gate] != Status::POSITIVE || !_hasBitmap[gate]) {
            return false;
        }
        const vehicle_cl250_scan_t& g = entry(gate);
        uint8_t base = g.request[g.size - 1];
        if (id <= base || (unsigned)(id - base) > kBitmapIds) {
            return false; // outside this bitmap: fail-closed
        }
        uint8_t k = (uint8_t)(id - base - 1);
        return (_bitmap[gate][k / 8] & (uint8_t)(0x80u >> (k % 8))) != 0;
    }

private:
    LineSink _sink;
    uint8_t _vinIndex;
    uint8_t _cursor = 0;
    bool _started = false;
    bool _done = false;
    Status _status[VEHICLE_CL250_SCAN_COUNT];
    bool _hasBitmap[VEHICLE_CL250_SCAN_COUNT];
    uint8_t _bitmap[VEHICLE_CL250_SCAN_COUNT][kBitmapBytes] = {{0}};
    char _line[160] = {0};

    void emitLine() {
        if (_sink != nullptr) {
            _sink(_line);
        }
    }

    void emitf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(_line, sizeof(_line), fmt, args);
        va_end(args);
        emitLine();
    }

    // "[SCAN] 03/24 01 40: <text>"
    void emitPrefix(uint8_t i, const char* fmt, ...) {
        const vehicle_cl250_scan_t& e = entry(i);
        int n = snprintf(_line, sizeof(_line), "[SCAN] %02u/%02u", (unsigned)(i + 1), (unsigned)VEHICLE_CL250_SCAN_COUNT);
        for (uint8_t k = 0; k < e.size && n > 0 && (size_t)n < sizeof(_line); k++) {
            n += snprintf(&_line[n], sizeof(_line) - (size_t)n, " %02X", e.request[k]);
        }
        if (n > 0 && (size_t)n < sizeof(_line)) {
            n += snprintf(&_line[n], sizeof(_line) - (size_t)n, ": ");
        }
        if (n > 0 && (size_t)n < sizeof(_line)) {
            va_list args;
            va_start(args, fmt);
            vsnprintf(&_line[n], sizeof(_line) - (size_t)n, fmt, args);
            va_end(args);
        }
        emitLine();
    }

    void emitHex(const uint8_t* msg, uint16_t len) {
        for (uint16_t off = 0; off < len; off = (uint16_t)(off + kHexBytesPerLine)) {
            int n = snprintf(_line, sizeof(_line), "[SCAN]   +%03u:", (unsigned)off);
            for (uint16_t k = off; k < len && k < off + kHexBytesPerLine && n > 0 && (size_t)n < sizeof(_line); k++) {
                n += snprintf(&_line[n], sizeof(_line) - (size_t)n, " %02X", msg[k]);
            }
            emitLine();
        }
    }

    // Supported ids of a bitmap answer: the request bytes after the SID read as one number
    // (the 0xF4xx DID, or the OBD PID / infotype base) plus bit + 1, without wrapping (the
    // last bit of 0xF4E0's bitmap is 0xF500, of 0x01 0xE0's is 0x100).
    void emitSupported(uint8_t i) {
        const vehicle_cl250_scan_t& e = entry(i);
        uint32_t base = 0;
        for (uint8_t b = 1; b < e.size; b++) {
            base = (base << 8) | e.request[b];
        }
        const int width = 2 * (e.size - 1);
        int n = snprintf(_line, sizeof(_line), "[SCAN]   supported:");
        bool any = false;
        for (uint8_t k = 0; k < kBitmapIds; k++) {
            if ((_bitmap[i][k / 8] & (uint8_t)(0x80u >> (k % 8))) == 0) {
                continue;
            }
            any = true;
            char id[12];
            int m = snprintf(id, sizeof(id), " %0*lX", width, (unsigned long)(base + k + 1u));
            if (n < 0 || m < 0 || (size_t)(n + m) >= sizeof(_line)) {
                break; // cannot happen for 32 ids of at most 5 digits; never overflows
            }
            memcpy(&_line[n], id, (size_t)m + 1u);
            n += m;
        }
        if (!any && n >= 0 && (size_t)n + 6u <= sizeof(_line)) {
            memcpy(&_line[n], " none", 6u);
        }
        emitLine();
    }

    // VIN: WMI only, after the echo and the optional NODI byte (CAN: 1 + 17 bytes).
    void reportSensitive(uint8_t i, const uint8_t* msg, uint16_t len) {
        uint16_t after = (uint16_t)(1u + echoLength(i));
        uint16_t data = (len > after) ? (uint16_t)(len - after) : 0;
        const uint8_t* vin = nullptr;
        if (i == _vinIndex && data == kVinLength + 1u) {
            vin = &msg[after + 1];
        } else if (i == _vinIndex && data == kVinLength) {
            vin = &msg[after];
        }
        bool printable = (vin != nullptr);
        for (uint8_t k = 0; printable && k < kVinLength; k++) {
            printable = vin[k] >= 0x20 && vin[k] <= 0x7E;
        }
        if (printable) {
            emitPrefix(i, "VIN WMI=%c%c%c (other %u characters masked, D-033)", vin[0], vin[1], vin[2],
                       (unsigned)(kVinLength - kWmiLength));
        } else {
            emitPrefix(i, "positive, %u bytes (masked, D-033)", (unsigned)len);
        }
    }

    void emitSummary() {
        unsigned counts[7] = {0};
        for (uint8_t i = 0; i < VEHICLE_CL250_SCAN_COUNT; i++) {
            counts[(uint8_t)_status[i]]++;
        }
        emitf("[SCAN] done: %u positive, %u NRC, %u no answer, %u aborted, %u FF refused, %u skipped. Normal polling starts.",
              counts[(uint8_t)Status::POSITIVE], counts[(uint8_t)Status::NRC], counts[(uint8_t)Status::NO_ANSWER],
              counts[(uint8_t)Status::ABORTED], counts[(uint8_t)Status::FF_REFUSED], counts[(uint8_t)Status::SKIPPED]);
    }
};

#endif // DISCOVERY_SCAN_H
