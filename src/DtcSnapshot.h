#ifndef DTC_SNAPSHOT_H
#define DTC_SNAPSHOT_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "uds_iso14229.h" // generated: D-040 client subset (gen/c/conn/)
#include "vehicle_cl250.h" // generated: VEHICLE_CL250_DTC_* (D-080)

// MISRA C:2012 17.1 / 21.6 deviation (conn reports MISRA only, D-046 item 3), as in
// DiscoveryScan.h: vsnprintf/snprintf format the report into one static buffer, with literal
// formats and integer conversions only; no heap, no input parsing beyond fixed offsets.

/**
 * @brief D-080: reads the ECU's DTC freeze frames (0x19 0x04) and extended data (0x19 0x06),
 * only when a DTC is present. Pure logic: it never transmits. HondaCANModule sends each
 * request through the D-020 frame gate, one request in flight, after every DID that is due
 * (D-080 item 4), and feeds the answers back. TEMPORARY, like conn's tester (D-023); rt-core
 * implements the same rule later (D-080 item 6).
 *
 * A run (D-080 item 2): 0x19 0x01 <mask> (count); only if the count is > 0, 0x19 0x02 <mask>
 * (list); then per listed DTC whose status matches the mask, at most DTC_MAX_PER_RUN,
 * 0x19 0x04 <DTC> <record> and 0x19 0x06 <DTC> <record>. Mask, cap and record numbers come
 * from gen/. At most kMaxRequestsPerRun requests per run (also counting a re-send after a
 * session refusal) and kRunsPerEpoch runs per stop epoch: the key-on run at the epoch start
 * and one warm-idle run (D-080 item 1; "ignition" = the D-075 stop epoch).
 *
 * Outcomes per epoch:
 *  - NRC 0x11 / 0x12 / 0x31: "not served", no more 0x19 until the next epoch.
 *  - NRC 0x7E / 0x7F: the D-075 item 1 session rule (the caller requests the session again);
 *    the same request is sent again once the session is confirmed; a second refusal in the
 *    epoch stops 0x19 until the next epoch (as the D-075 (f) rule for answered DIDs).
 *  - A refused (above max_ff_dl), oversize or failed segmented reception: the run is aborted
 *    and 0x19 is skipped for the rest of the epoch (D-080 item 3: at most one N_Bs hold).
 *  - NRC 0x78 is not an extension (D-080 item 4): the request times out at the base timeout.
 *  - Every other single fault of the 0x19 path that can cost the verified DIDs more than one
 *    unverified-read slot also turns 0x19 off for the epoch, after the answer in flight is
 *    logged (safety-reviewer MAJOR-1/2 on conn D-080): a timeout (a late answer may still come
 *    as a First Frame and start the quiet time), a transaction longer than its slot (SLOW),
 *    and a First Frame from the other response ID during a 0x19 reception (BOTH_IDS). So
 *    the quiet time and the long slot happen at most once per epoch.
 *  - Any other NRC or a malformed answer ends the run; the warm-idle run may still come.
 *
 * Raw answers are always logged (D-069 item 6, D-080 item 5): a header line now, then the
 * hex dump one line per flushOne() call (Serial is slow, see TWDT); the next request waits
 * until the dump is out. DTC numbers and status bytes are standard and decoded here only for
 * the sequence; record layouts are manufacturer-specific and never decoded here.
 */
class DtcSnapshot {
public:
    typedef void (*LineSink)(const char* line);
    enum class Step : uint8_t { NONE, COUNT, LIST, SNAPSHOT, EXT_DATA };
    // Why 0x19 is off for the rest of the stop epoch (NONE: not halted).
    enum class Halt : uint8_t { NONE, NOT_SERVED, SEGMENTED, SESSION_REFUSED, NO_ANSWER, SLOW, BOTH_IDS };
    enum class Trigger : uint8_t { KEY_ON, WARM_IDLE };

    static const uint8_t kDtcBytes = 3;
    static const uint8_t kRequestMax = (uint8_t)(3u + kDtcBytes); // 19 04 <DTC> <record>
    static const uint8_t kRunsPerEpoch = 2;
    static const uint8_t kMaxRequestsPerRun = (uint8_t)(2u + 2u * VEHICLE_CL250_DTC_MAX_PER_RUN);
    static const uint8_t kSessionRefusalsMax = 2;
    static const uint8_t kHexBytesPerLine = 32;
    // ISO 14229-1 response layouts (clause 12.3.2), standard values, not vehicle signals.
    static const uint8_t kCountAnswerLen = 6;  // 59 01 <availability> <format> <count hi> <lo>
    static const uint8_t kListHeaderLen = 3;   // 59 02 <availability>
    static const uint8_t kListRecordLen = 4;   // <DTC 3 bytes> <status>
    static const uint8_t kRecordHeaderLen = 6; // 59 04|06 <DTC 3 bytes> <status>

    static_assert(VEHICLE_CL250_DTC_MAX_PER_RUN >= 1u && VEHICLE_CL250_DTC_MAX_PER_RUN <= 8u,
                  "D-080: the run state is sized for at most 8 DTCs");
    static_assert(VEHICLE_CL250_MAX_FF_DL <= 0xFFFFu, "answer lengths are uint16_t");

    explicit DtcSnapshot(LineSink sink) : _sink(sink) {}

    void setSink(LineSink sink) { _sink = sink; }

    // A stop epoch starts (the first session confirmation after the tester's start or after
    // the ECU was lost): everything of the last epoch is forgotten and the key-on run starts.
    void startEpoch() {
        _halt = Halt::NONE;
        _runsStarted = 0;
        _warmIdleUsed = false;
        _sessionRefusals = 0;
        _requestsInEpoch = 0;
        endRun(nullptr);
        startRun(Trigger::KEY_ON);
    }

    // D-080 item 1: the first warm-idle condition of the epoch starts the second run. Once
    // per epoch, also if refused (halted or the run budget spent). While the key-on run is
    // still active nothing is used up: the caller asks again on a later pass.
    bool armWarmIdle() {
        if (_warmIdleUsed || _runActive) {
            return false;
        }
        _warmIdleUsed = true;
        if (_halt != Halt::NONE || _runsStarted >= kRunsPerEpoch) {
            emitf("[DTC] D-080 warm-idle run not started (%s)",
                  _halt != Halt::NONE ? haltText(_halt) : "no run left in this epoch");
            return false;
        }
        startRun(Trigger::WARM_IDLE);
        return true;
    }

    // A request is waiting: a run is active, 0x19 is not off for the epoch, nothing is in
    // flight and the last dump is out.
    bool due() const { return _runActive && _halt == Halt::NONE && !_inFlight && _dumpLen == 0; }
    bool inFlight() const { return _inFlight; }
    bool runActive() const { return _runActive; }
    Halt halt() const { return _halt; }
    Step step() const { return _step; }
    uint8_t runsStarted() const { return _runsStarted; }
    uint8_t requestsInRun() const { return _requestsInRun; }
    uint8_t requestsInEpoch() const { return _requestsInEpoch; }
    uint8_t dtcCount() const { return _dtcCount; }
    const uint8_t* dtc(uint8_t i) const { return (i < _dtcCount) ? _dtcs[i] : nullptr; }

    /**
     * @brief Builds the next request of the run (SID first, without ISO-TP PCI) into `out`
     * (kRequestMax bytes) and marks it in flight. 0 if none is due; the run ends if its
     * request budget is spent.
     */
    uint8_t buildRequest(uint8_t* out) {
        if (!due()) {
            return 0;
        }
        if (_requestsInRun >= kMaxRequestsPerRun) {
            endRun("request budget spent");
            return 0;
        }
        uint8_t len = 0;
        out[len++] = UDS_SID_READ_DTC_INFORMATION;
        out[len++] = subFunction(_step);
        if (_step == Step::COUNT || _step == Step::LIST) {
            out[len++] = VEHICLE_CL250_DTC_STATUS_MASK;
        } else {
            for (uint8_t k = 0; k < kDtcBytes; k++) {
                out[len++] = _dtcs[_dtcIndex][k];
            }
            out[len++] = (_step == Step::SNAPSHOT) ? VEHICLE_CL250_DTC_SNAPSHOT_RECORD
                                                   : VEHICLE_CL250_DTC_EXT_DATA_RECORD;
        }
        _inFlight = true;
        _requestsInRun++;
        _requestsInEpoch++;
        return len;
    }

    /**
     * @brief `msg` starts at the response SID; `len` may be only the start of a segmented
     * answer (the First Frame's bytes). True for the positive SID, the sub-function echo and,
     * for 0x04/0x06, the DTC echo of the request in flight.
     */
    bool matchesPositive(const uint8_t* msg, uint16_t len) const {
        if (!_inFlight || len < 2u ||
            msg[0] != (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET) ||
            msg[1] != subFunction(_step)) {
            return false;
        }
        if (_step == Step::SNAPSHOT || _step == Step::EXT_DATA) {
            if (len < (uint16_t)(2u + kDtcBytes)) {
                return false;
            }
            for (uint8_t k = 0; k < kDtcBytes; k++) {
                if (msg[2u + k] != _dtcs[_dtcIndex][k]) {
                    return false;
                }
            }
        }
        return true;
    }

    // `msg` = [0x7F][0x19][NRC].
    bool matchesNegative(const uint8_t* msg, uint16_t len) const {
        return _inFlight && len >= UDS_NEGATIVE_RESPONSE_LEN && msg[0] == UDS_SID_NEGATIVE_RESPONSE &&
               msg[1] == UDS_SID_READ_DTC_INFORMATION;
    }

    /**
     * @brief Turns 0x19 off for the rest of the epoch. The request in flight (if any) still
     * resolves normally, so its answer is logged; the run ends then.
     */
    void haltAfterAnswer(Halt h) {
        if (_halt != Halt::NONE) {
            return;
        }
        _halt = h;
        emitf("[DTC] D-080 no more 0x19 until the next stop epoch (%s)", haltText(h));
        settle();
    }

    // The complete positive answer to the request in flight (matchesPositive() was true),
    // from response ID `fromId` (logged: the fallback addressing is unverified).
    void onPositive(const uint8_t* msg, uint16_t len, uint32_t fromId = 0) {
        if (!_inFlight) {
            return;
        }
        _inFlight = false;
        logAnswer(msg, len, fromId);
        advance(msg, len);
        settle();
    }

private:
    void advance(const uint8_t* msg, uint16_t len) {
        switch (_step) {
            case Step::COUNT: {
                if (len != kCountAnswerLen) {
                    endRun("malformed count answer");
                    return;
                }
                const uint16_t count = (uint16_t)(((uint16_t)msg[4] << 8) | msg[5]);
                if (count == 0u) {
                    endRun("no DTC");
                    return;
                }
                _step = Step::LIST;
                return;
            }
            case Step::LIST: {
                if (len < kListHeaderLen || (uint16_t)(len - kListHeaderLen) % kListRecordLen != 0u) {
                    endRun("malformed list answer");
                    return;
                }
                uint16_t matching = 0;
                for (uint16_t off = kListHeaderLen; off < len; off = (uint16_t)(off + kListRecordLen)) {
                    if ((msg[off + kDtcBytes] & VEHICLE_CL250_DTC_STATUS_MASK) == 0u) {
                        continue; // the ECU filters by the mask; checked again here
                    }
                    matching++;
                    if (_dtcCount < VEHICLE_CL250_DTC_MAX_PER_RUN) {
                        memcpy(_dtcs[_dtcCount], &msg[off], kDtcBytes);
                        _dtcCount++;
                    }
                }
                if (matching > _dtcCount) {
                    emitf("[DTC] D-080 %u matching DTCs, only the first %u are read (DTC_MAX_PER_RUN)",
                          (unsigned)matching, (unsigned)_dtcCount);
                }
                if (_dtcCount == 0u) {
                    endRun("no DTC matches the status mask");
                    return;
                }
                _dtcIndex = 0;
                _step = Step::SNAPSHOT;
                return;
            }
            case Step::SNAPSHOT:
            case Step::EXT_DATA: {
                if (len < kRecordHeaderLen) {
                    endRun("malformed record answer");
                    return;
                }
                if (_step == Step::SNAPSHOT) {
                    _step = Step::EXT_DATA;
                    return;
                }
                _dtcIndex++;
                if (_dtcIndex >= _dtcCount) {
                    endRun("done");
                    return;
                }
                _step = Step::SNAPSHOT;
                return;
            }
            default:
                endRun("no step");
                return;
        }
    }

    // A halt that came while a request was in flight ends the run once it resolved.
    void settle() {
        if (_halt != Halt::NONE && _runActive && !_inFlight) {
            endRun(haltText(_halt));
        }
    }

public:
    enum class NrcAction : uint8_t { WAIT, ENDED, SESSION_LOST };

    /**
     * @brief An NRC to the request in flight. WAIT: 0x78, the request stays in flight (no
     * extension, it times out at the base timeout). SESSION_LOST: 0x7E/0x7F, the caller
     * requests the session again (D-075 item 1). ENDED: the request is resolved.
     */
    NrcAction onNrc(uint8_t nrc) {
        if (!_inFlight) {
            return NrcAction::ENDED;
        }
        if (nrc == UDS_NRC_RESPONSE_PENDING) {
            return NrcAction::WAIT;
        }
        _inFlight = false;
        emitf("[DTC] D-080 run %u: 0x19 0x%02X refused with NRC 0x%02X", (unsigned)_runsStarted,
              subFunction(_step), nrc);
        if (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED || nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED ||
            nrc == UDS_NRC_REQUEST_OUT_OF_RANGE) {
            haltEpoch(Halt::NOT_SERVED);
            return NrcAction::ENDED;
        }
        if (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION ||
            nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION) {
            if (++_sessionRefusals >= kSessionRefusalsMax) {
                haltEpoch(Halt::SESSION_REFUSED);
            }
            // Otherwise the same step is due again; the caller sends it in a confirmed session.
            settle();
            return NrcAction::SESSION_LOST;
        }
        endRun("refused");
        settle();
        return NrcAction::ENDED;
    }

    // MAJOR-1 (b): a late answer of a timed-out 0x19 may still come as a First Frame and
    // start the quiet time, so a timeout turns 0x19 off for the epoch.
    void onTimeout() {
        if (!_inFlight) {
            return;
        }
        _inFlight = false;
        haltEpoch(Halt::NO_ANSWER);
    }

    // D-080 item 3: a First Frame above max_ff_dl, an FC that could not go out or an aborted
    // reception of the answer in flight. `announced` = FF_DL.
    void onSegmentedFailure(const char* why, uint16_t received, uint16_t announced) {
        if (!_inFlight) {
            return;
        }
        _inFlight = false;
        emitf("[DTC] D-080 run %u: 0x19 0x%02X segmented answer failed (%s, %u of %u bytes)",
              (unsigned)_runsStarted, subFunction(_step), why, (unsigned)received, (unsigned)announced);
        haltEpoch(Halt::SEGMENTED);
    }

    // Prints at most one line of the pending dump. True if a line was printed.
    bool flushOne() {
        if (_dumpLen == 0u) {
            return false;
        }
        int n = snprintf(_line, sizeof(_line), "[DTC]   +%03u:", (unsigned)_dumpPos);
        for (uint16_t k = _dumpPos; k < _dumpLen && k < _dumpPos + kHexBytesPerLine && n > 0 &&
                                    (size_t)n < sizeof(_line); k++) {
            n += snprintf(&_line[n], sizeof(_line) - (size_t)n, " %02X", _dump[k]);
        }
        emitLine();
        _dumpPos = (uint16_t)(_dumpPos + kHexBytesPerLine);
        if (_dumpPos >= _dumpLen) {
            _dumpLen = 0;
            _dumpPos = 0;
        }
        return true;
    }

    static uint8_t subFunction(Step s) {
        switch (s) {
            case Step::COUNT: return UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK;
            case Step::LIST: return UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK;
            case Step::SNAPSHOT: return UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER;
            case Step::EXT_DATA: return UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER;
            default: return 0;
        }
    }

    static const char* haltText(Halt h) {
        switch (h) {
            case Halt::NOT_SERVED: return "not served";
            case Halt::SEGMENTED: return "segmented answer failed";
            case Halt::SESSION_REFUSED: return "refused in the session twice";
            case Halt::NO_ANSWER: return "no answer";
            case Halt::SLOW: return "answer slower than its slot";
            case Halt::BOTH_IDS: return "an unexpected 0x19 answer (other addressing or late)";
            default: return "none";
        }
    }

private:
    LineSink _sink;
    char _line[160] = {0};
    uint8_t _dump[VEHICLE_CL250_MAX_FF_DL] = {0};
    uint16_t _dumpLen = 0;
    uint16_t _dumpPos = 0;

    // Stop epoch.
    Halt _halt = Halt::NONE;
    uint8_t _runsStarted = 0;
    bool _warmIdleUsed = false;
    uint8_t _sessionRefusals = 0;
    uint8_t _requestsInEpoch = 0;

    // Run.
    bool _runActive = false;
    bool _inFlight = false;
    Trigger _trigger = Trigger::KEY_ON;
    Step _step = Step::NONE;
    uint8_t _requestsInRun = 0;
    uint8_t _dtcs[VEHICLE_CL250_DTC_MAX_PER_RUN][kDtcBytes] = {{0}};
    uint8_t _dtcCount = 0;
    uint8_t _dtcIndex = 0;

    void startRun(Trigger t) {
        _runActive = true;
        _inFlight = false;
        _trigger = t;
        _step = Step::COUNT;
        _requestsInRun = 0;
        _dtcCount = 0;
        _dtcIndex = 0;
        _runsStarted++;
        emitf("[DTC] D-080 run %u (%s) started", (unsigned)_runsStarted,
              t == Trigger::KEY_ON ? "key-on" : "warm idle");
    }

    // `why` nullptr: silently (a new epoch). The pending dump is kept (raw is always logged).
    void endRun(const char* why) {
        if (_runActive && why != nullptr) {
            emitf("[DTC] D-080 run %u ended: %s (%u DTCs, %u requests)", (unsigned)_runsStarted, why,
                  (unsigned)_dtcCount, (unsigned)_requestsInRun);
        }
        _runActive = false;
        _inFlight = false;
        _step = Step::NONE;
    }

    void haltEpoch(Halt h) {
        if (_halt != Halt::NONE) {
            endRun(haltText(_halt)); // already off (haltAfterAnswer): keep the first reason
            return;
        }
        endRun(haltText(h));
        _halt = h;
        emitf("[DTC] D-080 no more 0x19 until the next stop epoch (%s)", haltText(h));
    }

    void logAnswer(const uint8_t* msg, uint16_t len, uint32_t fromId) {
        if (_step == Step::SNAPSHOT || _step == Step::EXT_DATA) {
            emitf("[DTC] D-080 run %u: 0x19 0x%02X DTC %02X%02X%02X answer, %u bytes from 0x%08lX", (unsigned)_runsStarted,
                  subFunction(_step), _dtcs[_dtcIndex][0], _dtcs[_dtcIndex][1], _dtcs[_dtcIndex][2], (unsigned)len,
                  (unsigned long)fromId);
        } else {
            emitf("[DTC] D-080 run %u: 0x19 0x%02X answer, %u bytes from 0x%08lX", (unsigned)_runsStarted,
                  subFunction(_step), (unsigned)len, (unsigned long)fromId);
        }
        if (len > sizeof(_dump)) {
            len = (uint16_t)sizeof(_dump); // never: answers are bounded by max_ff_dl
        }
        memcpy(_dump, msg, len);
        _dumpLen = len;
        _dumpPos = 0;
    }

    void emitf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vsnprintf(_line, sizeof(_line), fmt, args);
        va_end(args);
        emitLine();
    }

    void emitLine() {
        if (_sink != nullptr) {
            _sink(_line);
        }
    }
};

#endif // DTC_SNAPSHOT_H
