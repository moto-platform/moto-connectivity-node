// D-080 -- DTC freeze frames (0x19) in the tester build: the key-on run at the stop epoch
// start, the warm-idle run from the generated D-079 W3 conditions, the 0x19 sequence and its
// caps, the NRC rules, the segmented answers with the one generated FC.CTS, and the
// failures that turn 0x19 off for the epoch. Same single translation unit pattern as
// test_can_protocol: the exact HondaCANModule.cpp of the tester env, against a simulated ECU.
#include <unity.h>
#include "../../src/HondaCANModule.h"
#include "../mocks/MockCanBus.h"
#include "../mocks/MockTesterLatchStore.h"
#include "../../src/IsoTpCan.h"
#include "uds_iso14229.h" // generated (gen/c/conn/, D-040)
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "../../src/HondaCANModule.cpp"
#include "../../external/moto-vehicle-defs/gen/c/conn/vehicle_cl250.c"

typedef std::vector<uint8_t> Bytes;

static const unsigned long kT0 = 5000 + kListenOnlyWindowMs + 10;
static void setClock(unsigned long relativeMs) { test_setMillis(kT0 + relativeMs); }

static std::vector<std::string> g_lines;
// DTC_ECHO=1 also prints the report lines (to read the format by eye).
static void captureLine(const char* line) {
    g_lines.push_back(line);
    if (getenv("DTC_ECHO") != nullptr) std::puts(line);
}
static size_t countLines(const char* needle) {
    size_t n = 0;
    for (const std::string& l : g_lines) {
        if (l.find(needle) != std::string::npos) n++;
    }
    return n;
}

void setUp(void) { g_lines.clear(); }
void tearDown(void) {}

// A 15-byte 0x59 0x01 answer (segmented; only its First Frame matters here).
static Bytes listOfBytes() {
    Bytes b = {0x59, 0x01, 0xFF, UDS_DTC_FORMAT_ISO_14229_1, 0x00, 0x02};
    for (int i = 0; i < 9; i++) b.push_back(0);
    return b;
}

// ---------------------------------------------------------------------------
// A simulated CL250 ECU. It answers requests on the primary request ID only (the fallback
// copy is ignored, so every answer comes once), one pass later.
// ---------------------------------------------------------------------------
struct Ecu {
    // DID values (raw bytes A, B) for the five verified DIDs; unverified DIDs get NRC 0x31.
    uint8_t rpmA = 0, rpmB = 0;  // (A*256+B)/4
    uint8_t speed = 0;           // A
    uint8_t ectRaw = 40;         // A-40 -> 0 degC
    bool answerSession = true;
    bool silent = false;         // the ECU answers nothing (ECU lost)
    // 0x19: answer per sub-function. Empty = no answer; {0x7F, 0x19, nrc} = an NRC.
    Bytes count = {0x59, 0x01, 0xFF, UDS_DTC_FORMAT_ISO_14229_1, 0x00, 0x00};
    Bytes list;
    Bytes snapshot;  // template: bytes 2..4 are replaced by the requested DTC
    Bytes extData;   // same
    uint16_t forceFfDl = 0; // != 0: a First Frame announcing this length (no CFs)
    bool corruptCf = false; // a sequence gap in the first CF
    bool bothIds = false;     // a 0x19 sent to the fallback request ID is answered there too
    unsigned long fbDelayMs = 0;    // the fallback's answers come this much later
    Bytes fbCount;                  // non-empty: the fallback answers the count with this
    unsigned long cfGapMs = 0;      // CF spacing after the FC (0 = all at once)
    unsigned long dtcLatencyMs = 0; // 0x19 answers come this much later
    bool lateFfAfter78 = false;     // COUNT: NRC 0x78, then a First Frame after the timeout
    bool wrongDtcEcho = false;      // 0x04/0x06 answers echo another DTC
    unsigned long now = 0;
    // Frames due later: (time, frame).
    std::vector<std::pair<unsigned long, CanFrame>> timed;
    std::vector<Bytes> dtcRequests; // every 0x19 request seen on the primary ID (payload without PCI)
    size_t fallbackDtcRequests = 0; // 0x19 requests seen on the fallback ID
    void onFallbackRequest(MockCanBus& bus, const CanFrame& f);

    std::vector<CanFrame> pendingCfs[2]; // per addressing (0 primary, 1 fallback), sent after its FC

    void inject(MockCanBus& bus, const CanFrame& f, unsigned long delay) {
        if (delay == 0) {
            bus.injectRxFrame(f);
        } else {
            timed.push_back(std::make_pair(now + delay, f));
        }
    }
    void deliverDue(MockCanBus& bus) {
        for (size_t i = 0; i < timed.size();) {
            if ((long)(now - timed[i].first) >= 0) {
                bus.injectRxFrame(timed[i].second);
                timed.erase(timed.begin() + (long)i);
            } else {
                i++;
            }
        }
    }

    static CanFrame frame() {
        CanFrame f;
        f.id = VEHICLE_CL250_RESPONSE_ID;
        f.extended = true;
        f.dlc = 8;
        for (int i = 0; i < 8; i++) f.data[i] = VEHICLE_CL250_PADDING_BYTE;
        return f;
    }

    void answer(MockCanBus& bus, const Bytes& msg, unsigned long delay = 0,
                uint32_t id = VEHICLE_CL250_RESPONSE_ID) {
        if (msg.size() <= 7) {
            CanFrame f = frame();
            f.id = id;
            f.extended = (id == VEHICLE_CL250_RESPONSE_ID) || id > 0x7FF;
            f.data[0] = (uint8_t)msg.size();
            for (size_t i = 0; i < msg.size(); i++) f.data[1 + i] = msg[i];
            inject(bus, f, delay);
            return;
        }
        const uint16_t len = forceFfDl ? forceFfDl : (uint16_t)msg.size();
        CanFrame ff = frame();
        ff.id = id;
        ff.extended = (id == VEHICLE_CL250_RESPONSE_ID) || id > 0x7FF;
        ff.data[0] = (uint8_t)(0x10 | ((len >> 8) & 0x0F));
        ff.data[1] = (uint8_t)(len & 0xFF);
        for (int i = 0; i < 6; i++) ff.data[2 + i] = msg[i];
        inject(bus, ff, delay);
        if (forceFfDl) return;
        std::vector<CanFrame>& cfs = pendingCfs[id == VEHICLE_CL250_RESPONSE_ID ? 0 : 1];
        cfs.clear();
        uint8_t sn = 1;
        for (size_t off = 6; off < msg.size(); off += 7) {
            CanFrame cf = frame();
            cf.data[0] = (uint8_t)(0x20 | ((corruptCf && sn == 1) ? 3 : sn));
            for (size_t k = 0; k < 7 && off + k < msg.size(); k++) cf.data[1 + k] = msg[off + k];
            cfs.push_back(cf);
            sn = (uint8_t)((sn + 1) & 0x0F);
        }
    }

    void onRequest(MockCanBus& bus, const CanFrame& f) {
        const bool fallback = (f.id == VEHICLE_CL250_FALLBACK_REQUEST_ID);
        if (f.id != VEHICLE_CL250_REQUEST_ID && !fallback) return;
        if (memcmp(f.data, vehicle_cl250_fc_cts, 8) == 0) {
            std::vector<CanFrame>& cfs = pendingCfs[fallback ? 1 : 0];
            unsigned long d = 0;
            for (const CanFrame& cf : cfs) {
                d += cfGapMs;
                inject(bus, cf, d);
            }
            cfs.clear();
            return;
        }
        if (fallback) {
            onFallbackRequest(bus, f);
            return;
        }
        if (silent) return;
        const uint8_t len = f.data[0] & 0x0F;
        Bytes req(&f.data[1], &f.data[1] + len);
        if (req[0] == VEHICLE_CL250_SESSION_SID) {
            if (answerSession) answer(bus, {VEHICLE_CL250_SESSION_POSITIVE_SID, VEHICLE_CL250_SESSION_SUBFUNCTION, 0x00, 0x32, 0x01, 0xF4});
            return;
        }
        if (req[0] == UDS_SID_READ_DATA_BY_IDENTIFIER) {
            const uint16_t did = (uint16_t)((req[1] << 8) | req[2]);
            switch (did) {
                case VEHICLE_CL250_DID_ENGINE_SPEED: answer(bus, {0x62, req[1], req[2], rpmA, rpmB}); break;
                case VEHICLE_CL250_DID_VEHICLE_SPEED: answer(bus, {0x62, req[1], req[2], speed}); break;
                case VEHICLE_CL250_DID_COOLANT_TEMP: answer(bus, {0x62, req[1], req[2], ectRaw}); break;
                case VEHICLE_CL250_DID_THROTTLE_POS: answer(bus, {0x62, req[1], req[2], 0}); break;
                case VEHICLE_CL250_DID_BATTERY_VOLTAGE: answer(bus, {0x62, req[1], req[2], 0x30, 0x70}); break;
                default: answer(bus, {0x7F, 0x22, UDS_NRC_REQUEST_OUT_OF_RANGE}); break;
            }
            return;
        }
        if (req[0] == UDS_SID_READ_DTC_INFORMATION) {
            dtcRequests.push_back(req);
            Bytes a;
            switch (req[1]) {
                case UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK: a = count; break;
                case UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK: a = list; break;
                case UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER: a = snapshot; break;
                case UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER: a = extData; break;
                default: break;
            }
            if (a.size() >= 5 && a[0] == 0x59 && (req[1] == 0x04 || req[1] == 0x06)) {
                a[2] = req[2]; a[3] = req[3]; a[4] = req[4]; // echo the requested DTC
                if (wrongDtcEcho) a[4] ^= 0xFF;
            }
            if (lateFfAfter78 && req[1] == UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK) {
                answer(bus, {0x7F, UDS_SID_READ_DTC_INFORMATION, UDS_NRC_RESPONSE_PENDING});
                answer(bus, listOfBytes(), VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 50); // a late segmented answer
                return;
            }
            if (a.empty()) return;
            answer(bus, a, dtcLatencyMs);
        }
    }
};

// The fallback addressing answers 0x19 like the primary (same stored DTCs), fbDelayMs later.
void Ecu::onFallbackRequest(MockCanBus& bus, const CanFrame& f) {
    if (!bothIds || silent) return;
    const uint8_t len = f.data[0] & 0x0F;
    Bytes req(&f.data[1], &f.data[1] + len);
    if (req[0] != UDS_SID_READ_DTC_INFORMATION) return;
    fallbackDtcRequests++;
    Bytes a;
    switch (req[1]) {
        case UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK: a = fbCount.empty() ? count : fbCount; break;
        case UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK: a = list; break;
        case UDS_READ_DTC_REPORT_DTC_SNAPSHOT_RECORD_BY_DTC_NUMBER: a = snapshot; break;
        case UDS_READ_DTC_REPORT_DTC_EXT_DATA_RECORD_BY_DTC_NUMBER: a = extData; break;
        default: break;
    }
    if (a.size() >= 5 && a[0] == 0x59 && (req[1] == 0x04 || req[1] == 0x06)) {
        a[2] = req[2]; a[3] = req[3]; a[4] = req[4];
    }
    if (!a.empty()) answer(bus, a, fbDelayMs + 1, VEHICLE_CL250_FALLBACK_RESPONSE_ID);
}

struct Rig {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module{bus, latchStore};
    SystemState state;
    Ecu ecu;
    unsigned long t = 0;
    size_t seen = 0;
    // Passes in which more than one request (not counting tester present / session) went out.
    int multiRequestPasses = 0;
    // Requests on the primary ID with the pass time they went out in.
    std::vector<std::pair<unsigned long, CanFrame>> sent;

    bool start() {
        module.testDtc().setSink(captureLine);
        test_setMillis(kT0 - kListenOnlyWindowMs - 10);
        if (!module.begin()) return false;
        module.update(state);
        test_setMillis(kT0 - 10);
        module.update(state);
        setClock(0);
        step();
        return !module.listening() && !module.latchedOff();
    }

    void step() {
        setClock(t);
        ecu.now = t;
        ecu.deliverDue(bus);
        module.update(state);
        int requests = 0;
        for (; seen < bus.txLog.size(); seen++) {
            const CanFrame& f = bus.txLog[seen];
            if (f.id == VEHICLE_CL250_REQUEST_ID && (f.data[0] & 0xF0) == 0 &&
                (f.data[1] == UDS_SID_READ_DATA_BY_IDENTIFIER || f.data[1] == UDS_SID_READ_DTC_INFORMATION)) {
                requests++;
            }
            if (f.id == VEHICLE_CL250_REQUEST_ID) sent.push_back(std::make_pair(t, f));
            ecu.onRequest(bus, f);
        }
        if (requests > 1) multiRequestPasses++;
    }

    void runFor(unsigned long ms, unsigned long stepMs = 5) {
        const unsigned long end = t + ms;
        while (t < end) {
            t += stepMs;
            step();
        }
    }

    size_t dtcRequestsWith(uint8_t sub) const {
        size_t n = 0;
        for (const Bytes& r : ecu.dtcRequests) {
            if (r[1] == sub) n++;
        }
        return n;
    }
    size_t fcCount() const {
        size_t n = 0;
        for (const CanFrame& f : bus.txLog) {
            if (memcmp(f.data, vehicle_cl250_fc_cts, 8) == 0) n++;
        }
        return n;
    }
};

static Bytes listOf(uint8_t n, uint8_t status = UDS_DTC_STATUS_CONFIRMED_DTC) {
    Bytes b = {0x59, 0x02, 0xFF};
    for (uint8_t i = 0; i < n; i++) {
        b.push_back(0x01);
        b.push_back((uint8_t)(0x10 + i));
        b.push_back(0x00);
        b.push_back(status);
    }
    return b;
}
static Bytes countOf(uint8_t n) { return {0x59, 0x01, 0xFF, UDS_DTC_FORMAT_ISO_14229_1, 0x00, n}; }
static Bytes recordOf(uint8_t sub, size_t extra) {
    Bytes b = {0x59, sub, 0, 0, 0, UDS_DTC_STATUS_CONFIRMED_DTC};
    for (size_t i = 0; i < extra; i++) b.push_back((uint8_t)i);
    return b;
}

// ---------------------------------------------------------------------------

void test_key_on_run_asks_the_count_once_and_stops_without_a_dtc(void) {
    Rig r;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(5000);
    TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size());
    const Bytes expected = {UDS_SID_READ_DTC_INFORMATION, UDS_READ_DTC_REPORT_NUMBER_OF_DTC_BY_STATUS_MASK,
                            VEHICLE_CL250_DTC_STATUS_MASK};
    TEST_ASSERT_TRUE(r.ecu.dtcRequests[0] == expected);
    TEST_ASSERT_FALSE(r.module.dtcSnapshot().runActive());
    TEST_ASSERT_EQUAL(1, countLines("ended: no DTC"));
    TEST_ASSERT_EQUAL_UINT32(0, r.module.blockedFrameCount());
    TEST_ASSERT_EQUAL(0, r.multiRequestPasses);
}

void test_full_run_reads_list_snapshot_and_ext_data_with_one_fc_per_segmented_answer(void) {
    Rig r;
    r.ecu.count = countOf(2);
    r.ecu.list = listOf(2);                       // 11 bytes: segmented
    r.ecu.snapshot = recordOf(0x04, 30);          // 36 bytes: segmented
    r.ecu.extData = recordOf(0x06, 0);            // 6 bytes: Single Frame
    TEST_ASSERT_TRUE(r.start());
    r.runFor(8000);
    TEST_ASSERT_EQUAL_size_t(6, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL_size_t(1, r.dtcRequestsWith(0x01));
    TEST_ASSERT_EQUAL_size_t(1, r.dtcRequestsWith(0x02));
    TEST_ASSERT_EQUAL_size_t(2, r.dtcRequestsWith(0x04));
    TEST_ASSERT_EQUAL_size_t(2, r.dtcRequestsWith(0x06));
    // The DTCs of the list, in order, with the generated record numbers.
    const Bytes& s0 = r.ecu.dtcRequests[2];
    TEST_ASSERT_EQUAL_HEX8(0x04, s0[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, s0[2]);
    TEST_ASSERT_EQUAL_HEX8(0x10, s0[3]);
    TEST_ASSERT_EQUAL_HEX8(VEHICLE_CL250_DTC_SNAPSHOT_RECORD, s0[5]);
    TEST_ASSERT_EQUAL_HEX8(0x11, r.ecu.dtcRequests[4][3]);
    TEST_ASSERT_EQUAL_HEX8(VEHICLE_CL250_DTC_EXT_DATA_RECORD, r.ecu.dtcRequests[5][5]);
    // List + 2 snapshots were segmented: one FC each, on the primary request ID.
    TEST_ASSERT_EQUAL_size_t(3, r.fcCount());
    for (const CanFrame& f : r.bus.txLog) {
        if (memcmp(f.data, vehicle_cl250_fc_cts, 8) == 0) {
            TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, f.id);
        }
    }
    TEST_ASSERT_EQUAL(1, countLines("ended: done (2 DTCs, 6 requests)"));
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NONE, r.module.dtcSnapshot().halt());
    // Raw answers are logged: 6 headers, and the 36-byte snapshot as two dump lines.
    TEST_ASSERT_EQUAL(6, countLines(" answer, "));
    TEST_ASSERT_EQUAL(2, countLines("[DTC]   +032:"));
    TEST_ASSERT_EQUAL_UINT32(0, r.module.blockedFrameCount());
    TEST_ASSERT_EQUAL(0, r.multiRequestPasses);
}

void test_run_reads_at_most_dtc_max_per_run_dtcs(void) {
    Rig r;
    r.ecu.count = countOf(12);
    r.ecu.list = listOf(12);
    r.ecu.snapshot = recordOf(0x04, 0);
    r.ecu.extData = recordOf(0x06, 0);
    TEST_ASSERT_TRUE(r.start());
    r.runFor(20000);
    TEST_ASSERT_EQUAL_size_t(VEHICLE_CL250_DTC_MAX_PER_RUN, r.dtcRequestsWith(0x04));
    TEST_ASSERT_EQUAL_size_t(VEHICLE_CL250_DTC_MAX_PER_RUN, r.dtcRequestsWith(0x06));
    TEST_ASSERT_EQUAL_size_t(DtcSnapshot::kMaxRequestsPerRun, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL(1, countLines("only the first"));
}

void test_list_entries_outside_the_status_mask_are_not_read(void) {
    Rig r;
    r.ecu.count = countOf(1);
    r.ecu.list = listOf(1, UDS_DTC_STATUS_TEST_FAILED); // not pending/confirmed
    TEST_ASSERT_TRUE(r.start());
    r.runFor(5000);
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL(1, countLines("no DTC matches the status mask"));
}

void test_nrc_0x11_0x12_0x31_mean_not_served_until_the_next_epoch(void) {
    const uint8_t nrcs[] = {UDS_NRC_SERVICE_NOT_SUPPORTED, UDS_NRC_SUBFUNCTION_NOT_SUPPORTED, UDS_NRC_REQUEST_OUT_OF_RANGE};
    for (uint8_t nrc : nrcs) {
        Rig r;
        r.ecu.count = {0x7F, UDS_SID_READ_DTC_INFORMATION, nrc};
        TEST_ASSERT_TRUE(r.start());
        r.runFor(3000);
        TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size());
        TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NOT_SERVED, r.module.dtcSnapshot().halt());
        // The warm-idle trigger is refused too in this epoch.
        TEST_ASSERT_FALSE(r.module.testDtc().armWarmIdle());
        r.runFor(3000);
        TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size());
        // ECU lost (> ECU absent timeout), then back: a new stop epoch asks again.
        r.ecu.silent = true;
        r.runFor(VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS + 500);
        r.ecu.silent = false;
        r.runFor(5000);
        TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
    }
}

void test_nrc_0x7f_requests_the_session_and_resends_then_halts_on_the_second(void) {
    Rig r;
    r.ecu.count = {0x7F, UDS_SID_READ_DTC_INFORMATION, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION};
    TEST_ASSERT_TRUE(r.start());
    r.runFor(6000);
    // Count asked twice (re-sent after the session was confirmed again), then halted.
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::SESSION_REFUSED, r.module.dtcSnapshot().halt());
    // A session request followed each refusal at once (no kSessionRetryMs wait).
    size_t sessionRequests = 0;
    for (const CanFrame& f : r.bus.txLog) {
        if (f.id == VEHICLE_CL250_REQUEST_ID && f.data[1] == VEHICLE_CL250_SESSION_SID) sessionRequests++;
    }
    TEST_ASSERT_EQUAL_size_t(3, sessionRequests); // start + one per refusal
}

void test_nrc_0x78_is_not_an_extension(void) {
    Rig r;
    r.ecu.count = {0x7F, UDS_SID_READ_DTC_INFORMATION, UDS_NRC_RESPONSE_PENDING};
    TEST_ASSERT_TRUE(r.start());
    r.runFor(5000);
    TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL(1, countLines("ended: no answer"));
    // MAJOR-1 (b): a late answer may still come as a First Frame, so 0x19 is off for the epoch.
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NO_ANSWER, r.module.dtcSnapshot().halt());
}

// D-080 item 4: an unanswered 0x19 request holds the slot for the base timeout only (+ one
// step), like an unverified DID read, also after NRC 0x78.
void test_dtc_request_holds_the_slot_for_the_base_timeout_only(void) {
    const Bytes answers[] = {Bytes(), Bytes{0x7F, UDS_SID_READ_DTC_INFORMATION, UDS_NRC_RESPONSE_PENDING}};
    for (const Bytes& a : answers) {
        g_lines.clear();
        Rig r;
        r.ecu.count = a;
        TEST_ASSERT_TRUE(r.start());
        r.runFor(3000, 1);
        size_t i = 0;
        while (i < r.sent.size() && r.sent[i].second.data[1] != UDS_SID_READ_DTC_INFORMATION) i++;
        TEST_ASSERT_TRUE(i < r.sent.size());
        size_t j = i + 1;
        while (j < r.sent.size() && r.sent[j].second.data[1] != UDS_SID_READ_DATA_BY_IDENTIFIER) j++;
        TEST_ASSERT_TRUE(j < r.sent.size());
        TEST_ASSERT_TRUE(r.sent[j].first - r.sent[i].first <= VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 2);
        TEST_ASSERT_EQUAL(1, countLines("ended: no answer"));
    }
}

void test_first_frame_above_max_ff_dl_is_refused_and_halts_the_epoch(void) {
    Rig r;
    r.ecu.count = countOf(1);
    r.ecu.list = listOf(2);
    r.ecu.forceFfDl = VEHICLE_CL250_MAX_FF_DL + 1;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(6000);
    TEST_ASSERT_EQUAL_size_t(0, r.fcCount());
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::SEGMENTED, r.module.dtcSnapshot().halt());
    TEST_ASSERT_EQUAL(1, countLines("First Frame refused"));
}

void test_aborted_reception_halts_the_epoch_and_starts_the_quiet_time(void) {
    Rig r;
    r.ecu.count = countOf(2);
    r.ecu.list = listOf(2);
    r.ecu.corruptCf = true;
    TEST_ASSERT_TRUE(r.start());
    // Run until the list's FC went out, then check no request follows within the quiet time.
    while (r.fcCount() == 0 && r.t < 5000) {
        r.t += 1;
        r.step();
    }
    TEST_ASSERT_EQUAL_size_t(1, r.fcCount());
    r.t += 1;
    r.step(); // drains the bad CF: reception aborted
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::SEGMENTED, r.module.dtcSnapshot().halt());
    const size_t before = r.bus.txLog.size();
    const unsigned long abortedAt = r.t;
    while (r.bus.txLog.size() == before) {
        r.t += 1;
        r.step();
    }
    TEST_ASSERT_TRUE(r.t - abortedAt >= VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS);
    TEST_ASSERT_EQUAL(1, countLines("sequence gap"));
    r.runFor(5000);
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size()); // nothing more this epoch
}

void test_no_dtc_request_without_a_confirmed_session(void) {
    Rig r;
    r.ecu.answerSession = false;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(8000);
    TEST_ASSERT_EQUAL_size_t(0, r.ecu.dtcRequests.size());
}

// D-080 item 1: the generated W3 conditions (ECT, speed, RPM band) for their min duration.
static void warmIdle(Ecu& e) {
    e.ectRaw = 40 + 85;   // 85 degC
    e.speed = 0;
    e.rpmA = 0x12;        // 0x12C0 / 4 = 1200 rpm
    e.rpmB = 0xC0;
}

void test_warm_idle_starts_the_second_run_once_after_its_min_duration(void) {
    Rig r;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(3000);
    TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size()); // key-on run
    warmIdle(r.ecu);
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS - 1000);
    TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size()); // not yet
    r.runFor(3000);
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL(1, countLines("(warm idle) started"));
    TEST_ASSERT_EQUAL_UINT8(2, r.module.dtcSnapshot().runsStarted());
    // Never a third run in the epoch, however long the idle lasts.
    r.runFor(2 * VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS);
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
}

void test_warm_idle_hold_restarts_when_a_condition_breaks(void) {
    Rig r;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(3000);
    warmIdle(r.ecu);
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS - 5000);
    r.ecu.speed = 5; // moving: not met
    r.runFor(1000);
    r.ecu.speed = 0;
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS - 5000);
    TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size());
    r.runFor(7000);
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
}

void test_cold_or_off_band_engine_never_starts_the_warm_idle_run(void) {
    Rig r;
    TEST_ASSERT_TRUE(r.start());
    warmIdle(r.ecu);
    r.ecu.ectRaw = 40 + 80; // 80 degC: the condition is gt 80
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS + 5000);
    warmIdle(r.ecu);
    r.ecu.rpmA = 0x1D; r.ecu.rpmB = 0x4C; // 0x1D4C / 4 = 1875 rpm: above the band
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS + 5000);
    TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL_UINT8(1, r.module.dtcSnapshot().runsStarted());
}

void test_stale_values_do_not_hold_the_warm_idle_condition(void) {
    Rig r;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(3000);
    warmIdle(r.ecu);
    r.runFor(2000);
    r.ecu.silent = true; // values age past stale_after_ms, ECU lost meanwhile
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS);
    TEST_ASSERT_EQUAL_UINT8(1, r.module.dtcSnapshot().runsStarted());
}

// The 0x19 reads never change what a verified DID gets: across a full run the engine speed
// is still asked within its period plus the one read in flight.
void test_dtc_run_keeps_the_engine_speed_cadence(void) {
    Rig r;
    r.ecu.count = countOf(8);
    r.ecu.list = listOf(8);
    r.ecu.snapshot = recordOf(0x04, 100);
    r.ecu.extData = recordOf(0x06, 20);
    TEST_ASSERT_TRUE(r.start());
    r.runFor(15000, 1);
    TEST_ASSERT_EQUAL_size_t(DtcSnapshot::kMaxRequestsPerRun, r.ecu.dtcRequests.size());
    // The largest gap between engine speed requests: its period plus at most the one read
    // in flight (base timeout) and one step, as without 0x19 (D-068 item 1, D-080 item 4).
    const unsigned long period = vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].poll_period_ms;
    unsigned long last = 0, maxGap = 0;
    for (const auto& p : r.sent) {
        const CanFrame& f = p.second;
        if (f.data[1] == UDS_SID_READ_DATA_BY_IDENTIFIER &&
            (uint16_t)((f.data[2] << 8) | f.data[3]) == VEHICLE_CL250_DID_ENGINE_SPEED) {
            if (last != 0 && p.first - last > maxGap) maxGap = p.first - last;
            last = p.first;
        }
    }
    TEST_ASSERT_TRUE(last > 0);
    TEST_ASSERT_TRUE(maxGap <= period + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 2);
    TEST_ASSERT_EQUAL(0, r.multiRequestPasses);
}


static void dtcAnswers8(Ecu& e, size_t snapshotExtra) {
    e.count = countOf(8);
    e.list = listOf(8);
    e.snapshot = recordOf(0x04, snapshotExtra);
    e.extData = recordOf(0x06, 20);
}

// Gaps between consecutive transmissions on the primary request ID longer than `ms`, from the
// session request on: a quiet time (no request at all, tester present included) shows as a
// gap >= 1500 ms, a long 0x19 slot as a gap > 300 ms (round-2 MINOR-1).
static size_t gapsOver(const Rig& r, unsigned long ms) {
    size_t n = 0;
    for (size_t i = 1; i < r.sent.size(); i++) {
        if (r.sent[i].first - r.sent[i - 1].first > ms) n++;
    }
    return n;
}

// Round-2 MAJOR-A/B: both addressings answer the count; the run is locked to the first
// answering ID, the rest goes there only, so no quiet time and no halt at any delay.
void test_both_ids_the_run_is_locked_to_the_first_answering_id(void) {
    const unsigned long delays[] = {0, 3, 10, 30, 50};
    for (unsigned long d : delays) {
        g_lines.clear();
        Rig r;
        dtcAnswers8(r.ecu, 30);
        r.ecu.bothIds = true;
        r.ecu.fbDelayMs = d;
        TEST_ASSERT_TRUE(r.start());
        r.runFor(15000, 1);
        TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NONE, r.module.dtcSnapshot().halt());
        TEST_ASSERT_EQUAL_size_t(DtcSnapshot::kMaxRequestsPerRun, r.ecu.dtcRequests.size());
        TEST_ASSERT_EQUAL_size_t(1, r.ecu.fallbackDtcRequests); // the count only
        TEST_ASSERT_EQUAL_size_t(0, gapsOver(r, 1500));
        TEST_ASSERT_EQUAL(1, countLines("ended: done (8 DTCs, 18 requests)"));
        TEST_ASSERT_EQUAL(0, countLines("from 0x000007E8"));
    }
}

// A node on the fallback addressing that refuses 0x19 first stops it for the epoch
// (conservative, round-1 MINOR-3); one that answers after the ECU is ignored.
void test_fallback_refusal_first_stops_later_is_ignored(void) {
    {
        Rig r;
        r.ecu.bothIds = true;
        r.ecu.count = countOf(1);
        r.ecu.fbCount = {0x7F, UDS_SID_READ_DTC_INFORMATION, UDS_NRC_SERVICE_NOT_SUPPORTED};
        r.ecu.dtcLatencyMs = 20; // the primary answers after the fallback
        TEST_ASSERT_TRUE(r.start());
        r.runFor(5000, 1);
        TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NOT_SERVED, r.module.dtcSnapshot().halt());
        TEST_ASSERT_EQUAL_size_t(0, gapsOver(r, 1500));
    }
    {
        Rig r;
        r.ecu.bothIds = true;
        r.ecu.count = countOf(1);
        r.ecu.list = listOf(1);
        r.ecu.snapshot = recordOf(0x04, 0);
        r.ecu.extData = recordOf(0x06, 0);
        r.ecu.fbCount = {0x7F, UDS_SID_READ_DTC_INFORMATION, UDS_NRC_SERVICE_NOT_SUPPORTED};
        r.ecu.fbDelayMs = 30;
        TEST_ASSERT_TRUE(r.start());
        r.runFor(5000, 1);
        TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NONE, r.module.dtcSnapshot().halt());
        TEST_ASSERT_EQUAL(1, countLines("ended: done (1 DTCs, 4 requests)"));
    }
}

// Round-2 MAJOR-A: a late segmented 0x19 answer from the other addressing, while nothing is
// in flight: one quiet time, then 0x19 is off for the epoch.
void test_late_segmented_answer_from_the_other_id_costs_one_hold(void) {
    Rig r;
    dtcAnswers8(r.ecu, 30);
    r.ecu.bothIds = true;
    r.ecu.fbCount = {0x59, 0x01, 0xFF, UDS_DTC_FORMAT_ISO_14229_1, 0x00, 0x08, 0, 0, 0, 0, 0, 0};
    r.ecu.fbDelayMs = 30;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(15000, 1);
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::BOTH_IDS, r.module.dtcSnapshot().halt());
    TEST_ASSERT_EQUAL_size_t(1, gapsOver(r, 1500));
    TEST_ASSERT_EQUAL(1, countLines("ended: an unexpected 0x19 answer"));
    TEST_ASSERT_TRUE(r.ecu.dtcRequests.size() < DtcSnapshot::kMaxRequestsPerRun); // cut short
    warmIdle(r.ecu);
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS + 5000, 1);
    TEST_ASSERT_EQUAL_size_t(1, gapsOver(r, 1500)); // the warm-idle run is refused
}

// MAJOR-2: CFs at N_Cr - 5 ms: the slow transaction completes once, then 0x19 is off.
void test_slow_segmented_answer_costs_one_long_slot_per_epoch(void) {
    Rig r;
    dtcAnswers8(r.ecu, 100);
    r.ecu.cfGapMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS - 5;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(30000, 1);
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::SLOW, r.module.dtcSnapshot().halt());
    TEST_ASSERT_EQUAL_size_t(2, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL(1, countLines("0x19 0x02 answer, 35 bytes")); // logged before the halt
    TEST_ASSERT_EQUAL_size_t(1, gapsOver(r, 300));                  // exactly one long slot
    TEST_ASSERT_EQUAL_size_t(0, gapsOver(r, 1500));
}

// MAJOR-1 (b): NRC 0x78, then the late answer as a First Frame after the timeout: one quiet
// time, and 0x19 is off for the epoch (also the warm-idle run is refused).
void test_0x78_then_a_late_first_frame_costs_one_hold_per_epoch(void) {
    Rig r;
    r.ecu.lateFfAfter78 = true;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(5000, 1);
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NO_ANSWER, r.module.dtcSnapshot().halt());
    TEST_ASSERT_EQUAL_size_t(0, r.fcCount());
    warmIdle(r.ecu);
    r.runFor(VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS + 5000, 1);
    TEST_ASSERT_EQUAL_size_t(1, r.ecu.dtcRequests.size());
    TEST_ASSERT_EQUAL_size_t(1, gapsOver(r, 1500)); // exactly one quiet time
}

// A First Frame whose DTC echo is not the request's gets no FC.
void test_first_frame_with_another_dtc_echo_gets_no_fc(void) {
    Rig r;
    r.ecu.count = countOf(1);
    r.ecu.list = listOf(1);            // 7 bytes: a Single Frame
    r.ecu.snapshot = recordOf(0x04, 30);
    r.ecu.wrongDtcEcho = true;
    TEST_ASSERT_TRUE(r.start());
    r.runFor(6000, 1);
    TEST_ASSERT_EQUAL_size_t(0, r.fcCount());
    TEST_ASSERT_EQUAL_size_t(3, r.ecu.dtcRequests.size());
    // Round-2 MAJOR-A: the FF that got no FC costs the epoch's one quiet time and stops 0x19.
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::BOTH_IDS, r.module.dtcSnapshot().halt());
    TEST_ASSERT_EQUAL_size_t(1, gapsOver(r, 1500));
}

// MINOR-2: no tester present or session request while a 0x19 waits for its answer.
void test_no_aux_request_while_a_dtc_request_is_in_flight(void) {
    Rig r;
    dtcAnswers8(r.ecu, 0);
    r.ecu.dtcLatencyMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS - 15; // inside the slot
    TEST_ASSERT_TRUE(r.start());
    r.runFor(10000, 1);
    TEST_ASSERT_EQUAL_size_t(DtcSnapshot::kMaxRequestsPerRun, r.ecu.dtcRequests.size());
    // 18 requests x ~85 ms span more than one tester present period, so one fell due inside.
    TEST_ASSERT_TRUE(DtcSnapshot::kMaxRequestsPerRun * (VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS - 15) >
                     VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS);
    for (size_t i = 0; i < r.sent.size(); i++) {
        if (r.sent[i].second.data[1] != UDS_SID_READ_DTC_INFORMATION) continue;
        const unsigned long t0 = r.sent[i].first;
        for (size_t j = i + 1; j < r.sent.size() && r.sent[j].first < t0 + r.ecu.dtcLatencyMs; j++) {
            const uint8_t sid = r.sent[j].second.data[1];
            TEST_ASSERT_TRUE(sid != VEHICLE_CL250_TESTER_PRESENT_SID && sid != VEHICLE_CL250_SESSION_SID);
        }
    }
    TEST_ASSERT_EQUAL(DtcSnapshot::Halt::NONE, r.module.dtcSnapshot().halt());
}

// Round-2 MINOR-3: a 0x19 never goes out in the pass that sent tester present or the session
// retry (its answer could land in the 0x19 reception). Swept over answer delays so tester
// present falls due right where a 0x19 is due.
void test_no_dtc_request_in_a_pass_with_an_aux_request(void) {
    for (unsigned long lat = 1; lat < VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS; lat += 2) {
        Rig r;
        dtcAnswers8(r.ecu, 0);
        r.ecu.dtcLatencyMs = lat;
        TEST_ASSERT_TRUE(r.start());
        r.runFor(4000, 1);
        for (size_t i = 0; i < r.sent.size(); i++) {
            if (r.sent[i].second.data[1] != UDS_SID_READ_DTC_INFORMATION) continue;
            for (size_t j = 0; j < r.sent.size(); j++) {
                const uint8_t sid = r.sent[j].second.data[1];
                if (r.sent[j].first == r.sent[i].first &&
                    (sid == VEHICLE_CL250_TESTER_PRESENT_SID || sid == VEHICLE_CL250_SESSION_SID)) {
                    TEST_FAIL_MESSAGE("tester present / session request in the pass of a 0x19");
                }
            }
        }
    }
}

// The pure state machine: request budget and the dump flush gate.
void test_snapshot_state_machine_budget_and_dump_gate(void) {
    DtcSnapshot d(captureLine);
    TEST_ASSERT_FALSE(d.due());
    d.startEpoch();
    TEST_ASSERT_TRUE(d.due());
    uint8_t req[DtcSnapshot::kRequestMax];
    TEST_ASSERT_EQUAL_UINT8(3, d.buildRequest(req));
    TEST_ASSERT_FALSE(d.due()); // in flight
    TEST_ASSERT_EQUAL_UINT8(0, d.buildRequest(req));
    const uint8_t count[] = {0x59, 0x01, 0xFF, 0x01, 0x00, 0x03};
    TEST_ASSERT_TRUE(d.matchesPositive(count, sizeof(count)));
    d.onPositive(count, sizeof(count));
    TEST_ASSERT_FALSE(d.due()); // the dump line is still pending
    TEST_ASSERT_TRUE(d.flushOne());
    TEST_ASSERT_FALSE(d.flushOne());
    TEST_ASSERT_TRUE(d.due());
    TEST_ASSERT_EQUAL_UINT8(3, d.buildRequest(req));
    TEST_ASSERT_EQUAL_HEX8(UDS_READ_DTC_REPORT_DTC_BY_STATUS_MASK, req[1]);
    // A list answer echoing the wrong sub-function is not ours.
    const uint8_t wrong[] = {0x59, 0x01, 0xFF};
    TEST_ASSERT_FALSE(d.matchesPositive(wrong, sizeof(wrong)));
    // Malformed list (not 3 + 4n bytes) ends the run.
    const uint8_t bad[] = {0x59, 0x02, 0xFF, 0x01, 0x02};
    d.onPositive(bad, sizeof(bad));
    TEST_ASSERT_FALSE(d.runActive());
    TEST_ASSERT_EQUAL(1, countLines("malformed list answer"));
    // The warm-idle run, then no third.
    TEST_ASSERT_TRUE(d.armWarmIdle());
    TEST_ASSERT_FALSE(d.armWarmIdle());
    TEST_ASSERT_EQUAL_UINT8(2, d.runsStarted());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_key_on_run_asks_the_count_once_and_stops_without_a_dtc);
    RUN_TEST(test_full_run_reads_list_snapshot_and_ext_data_with_one_fc_per_segmented_answer);
    RUN_TEST(test_run_reads_at_most_dtc_max_per_run_dtcs);
    RUN_TEST(test_list_entries_outside_the_status_mask_are_not_read);
    RUN_TEST(test_nrc_0x11_0x12_0x31_mean_not_served_until_the_next_epoch);
    RUN_TEST(test_nrc_0x7f_requests_the_session_and_resends_then_halts_on_the_second);
    RUN_TEST(test_nrc_0x78_is_not_an_extension);
    RUN_TEST(test_dtc_request_holds_the_slot_for_the_base_timeout_only);
    RUN_TEST(test_first_frame_above_max_ff_dl_is_refused_and_halts_the_epoch);
    RUN_TEST(test_aborted_reception_halts_the_epoch_and_starts_the_quiet_time);
    RUN_TEST(test_no_dtc_request_without_a_confirmed_session);
    RUN_TEST(test_warm_idle_starts_the_second_run_once_after_its_min_duration);
    RUN_TEST(test_warm_idle_hold_restarts_when_a_condition_breaks);
    RUN_TEST(test_cold_or_off_band_engine_never_starts_the_warm_idle_run);
    RUN_TEST(test_stale_values_do_not_hold_the_warm_idle_condition);
    RUN_TEST(test_dtc_run_keeps_the_engine_speed_cadence);
    RUN_TEST(test_both_ids_the_run_is_locked_to_the_first_answering_id);
    RUN_TEST(test_fallback_refusal_first_stops_later_is_ignored);
    RUN_TEST(test_late_segmented_answer_from_the_other_id_costs_one_hold);
    RUN_TEST(test_slow_segmented_answer_costs_one_long_slot_per_epoch);
    RUN_TEST(test_0x78_then_a_late_first_frame_costs_one_hold_per_epoch);
    RUN_TEST(test_first_frame_with_another_dtc_echo_gets_no_fc);
    RUN_TEST(test_no_aux_request_while_a_dtc_request_is_in_flight);
    RUN_TEST(test_no_dtc_request_in_a_pass_with_an_aux_request);
    RUN_TEST(test_snapshot_state_machine_budget_and_dump_gate);
    return UNITY_END();
}
