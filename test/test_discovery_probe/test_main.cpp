// D-059 item 3 -- the discovery probe env (CONN_DISCOVERY_PROBE=1): the one-shot scan over the
// generated discovery_scan table, segmented reception with the one generated FC.CTS, the
// aborts (N_Cr, sequence gap, lost frame, unexpected frame) and the VIN mask. Same single
// translation unit pattern as test_can_protocol, with the probe flag set before any include,
// so this suite runs the exact HondaCANModule.cpp the probe env compiles.
#define CONN_DISCOVERY_PROBE 1

#include <unity.h>
#include "../../src/HondaCANModule.h"
#include "../mocks/MockCanBus.h"
#include "../mocks/MockTesterLatchStore.h"
#include "../../src/IsoTpCan.h"
#include "uds_iso14229.h" // generated (gen/c/conn/, D-040)
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "../../src/HondaCANModule.cpp"
#include "../../external/moto-vehicle-defs/gen/c/conn/vehicle_cl250.c"

typedef std::vector<uint8_t> Bytes;

// Scan table indexes used below, checked against the generated requests in
// test_table_indexes_used_by_these_tests.
static const uint8_t kObdPids0 = 0;      // 01 00
static const uint8_t kObdPids1 = 1;      // 01 20
static const uint8_t kObdInfotypes = 8;  // 09 00
static const uint8_t kVin = 9;           // 09 02
static const uint8_t kDtcList = 15;      // 19 02 FF
static const uint8_t kDid0 = 16;         // 22 F4 00

static std::vector<std::string> g_lines;
// SCAN_ECHO=1 also prints the report lines (to read the format by eye).
static void captureLine(const char* line) {
    g_lines.push_back(line);
    if (getenv("SCAN_ECHO") != nullptr) std::puts(line);
}

void setUp(void) { g_lines.clear(); }
void tearDown(void) {}

static const unsigned long kT0 = 5000 + kListenOnlyWindowMs + 10;
static void setClock(unsigned long relativeMs) { test_setMillis(kT0 + relativeMs); }

static bool anyLine(const char* needle) {
    for (const std::string& l : g_lines) {
        if (l.find(needle) != std::string::npos) return true;
    }
    return false;
}

// Listen window, normal mode, session request at setClock(0) (as in test_can_protocol).
static bool startProbe(HondaCANModule& module, SystemState& state) {
    module.testScan().setSink(captureLine);
    test_setMillis(kT0 - kListenOnlyWindowMs - 10);
    if (!module.begin()) return false;
    module.update(state);
    test_setMillis(kT0 - 10);
    module.update(state);
    setClock(0);
    module.update(state);
    return !module.listening() && !module.latchedOff();
}

static CanFrame frameOn(uint32_t id, bool extended) {
    CanFrame f;
    f.id = id;
    f.extended = extended;
    f.dlc = 8;
    for (int i = 0; i < 8; i++) f.data[i] = VEHICLE_CL250_PADDING_BYTE;
    return f;
}

static CanFrame singleFrame(const Bytes& msg, uint32_t id = VEHICLE_CL250_RESPONSE_ID, bool ext = true) {
    CanFrame f = frameOn(id, ext);
    f.data[0] = (uint8_t)msg.size();
    for (size_t i = 0; i < msg.size(); i++) f.data[1 + i] = msg[i];
    return f;
}

// FF announcing `announced` bytes (defaults to msg.size()), carrying the first 6 of msg.
static CanFrame firstFrame(const Bytes& msg, uint32_t id = VEHICLE_CL250_RESPONSE_ID, bool ext = true,
                           int announced = -1) {
    uint16_t len = (announced < 0) ? (uint16_t)msg.size() : (uint16_t)announced;
    CanFrame f = frameOn(id, ext);
    f.data[0] = (uint8_t)(0x10 | ((len >> 8) & 0x0F));
    f.data[1] = (uint8_t)(len & 0xFF);
    for (size_t i = 0; i < 6 && i < msg.size(); i++) f.data[2 + i] = msg[i];
    return f;
}

static std::vector<CanFrame> consecutiveFrames(const Bytes& msg, uint32_t id = VEHICLE_CL250_RESPONSE_ID, bool ext = true) {
    std::vector<CanFrame> out;
    uint8_t sn = 1;
    for (size_t off = 6; off < msg.size(); off += 7) {
        CanFrame f = frameOn(id, ext);
        f.data[0] = (uint8_t)(0x20 | sn);
        for (size_t k = 0; k < 7 && off + k < msg.size(); k++) f.data[1 + k] = msg[off + k];
        out.push_back(f);
        sn = (uint8_t)((sn + 1) & 0x0F);
    }
    return out;
}

static bool isFcCts(const CanFrame& f) {
    if (f.dlc != VEHICLE_CL250_FRAME_DLC) return false;
    for (int i = 0; i < 8; i++) {
        if (f.data[i] != vehicle_cl250_fc_cts[i]) return false;
    }
    return true;
}

static int countFc(const MockCanBus& bus) {
    int n = 0;
    for (const CanFrame& f : bus.txLog) n += isFcCts(f) ? 1 : 0;
    return n;
}

static Bytes requestOf(uint8_t entry) {
    const vehicle_cl250_scan_t& e = vehicle_cl250_discovery_scan[entry];
    return Bytes(e.request, e.request + e.size);
}

// The SF payload of a primary-ID request frame, or empty for anything else (FC, fallback).
static Bytes primaryRequest(const CanFrame& f) {
    if (f.id != VEHICLE_CL250_REQUEST_ID || (f.data[0] & 0xF0) != 0) return Bytes();
    return Bytes(&f.data[1], &f.data[1] + (f.data[0] & 0x0F));
}

static bool lastPrimaryRequestIs(const MockCanBus& bus, const Bytes& req) {
    for (size_t i = bus.txLog.size(); i > 0; i--) {
        Bytes p = primaryRequest(bus.txLog[i - 1]);
        if (!p.empty()) return p == req;
    }
    return false;
}

static int countPrimaryRequests(const MockCanBus& bus, const Bytes& req) {
    int n = 0;
    for (const CanFrame& f : bus.txLog) n += (primaryRequest(f) == req) ? 1 : 0;
    return n;
}

static const char kVinText[] = "TSTSECRETVIN12345"; // a made-up VIN: WMI "TST"

static Bytes withText(Bytes head, const char* text, size_t n) {
    for (size_t i = 0; i < n; i++) head.push_back((uint8_t)text[i]);
    return head;
}

// A plausible ECU: SF, segmented and negative answers, one entry silent, gates as noted.
static Bytes defaultEcu(const Bytes& req) {
    if (req == Bytes({0x01, 0x00})) return {0x41, 0x00, 0xBE, 0x1F, 0xA8, 0x13}; // last bit: 0x20 supported
    if (req == Bytes({0x01, 0x20})) return {0x41, 0x20, 0x80, 0x00, 0x00, 0x00}; // chain stops
    if (req == Bytes({0x09, 0x00})) return {0x49, 0x00, 0x55, 0x40, 0x00, 0x00}; // 02 04 06 08 0A
    if (req == Bytes({0x09, 0x02})) return withText({0x49, 0x02, 0x01}, kVinText, 17);
    if (req == Bytes({0x09, 0x04})) return withText({0x49, 0x04, 0x01}, "CALIDTEST0000001", 16);
    if (req == Bytes({0x09, 0x06})) return {0x49, 0x06, 0x01, 0x12, 0x34, 0x56, 0x78};
    if (req == Bytes({0x09, 0x08})) return {0x7F, 0x09, 0x12};
    if (req == Bytes({0x09, 0x0A})) return withText({0x49, 0x0A, 0x01}, "ECM-EngineControl\0\0\0", 20);
    if (req == Bytes({0x19, 0x01, 0xFF})) return {0x59, 0x01, 0xFF, 0x01, 0x00, 0x02};
    if (req == Bytes({0x19, 0x02, 0xFF}))
        return {0x59, 0x02, 0xFF, 0x01, 0x23, 0x45, 0x2F, 0x06, 0x78, 0x9A, 0x08, 0x0B, 0xCD, 0xEF, 0x2F};
    if (req == Bytes({0x22, 0xF4, 0x00})) return {0x62, 0xF4, 0x00, 0x18, 0x00, 0x00, 0x01}; // F420 supported
    return Bytes(); // 22 F4 20 and anything else: no answer
}

typedef Bytes (*Responder)(const Bytes& req);

struct ScanLog {
    std::vector<unsigned long> sentAt; // scan request sends (primary ID)
    std::vector<bool> answered;
};

// Runs the bus with a simulated ECU until the scan is done: new primary-ID requests are
// answered on the next pass (session 0x10 0x03 is confirmed); a segmented answer's CFs
// are queued only after the FC.CTS went out.
static void runScan(HondaCANModule& module, MockCanBus& bus, SystemState& state, unsigned long& t,
                    Responder ecu, ScanLog* log = nullptr, unsigned long maxMs = 120000) {
    size_t seen = 0;
    std::vector<CanFrame> waitingCfs;
    while (!module.discoveryScanDone() && t < maxMs) {
        t += 5;
        setClock(t);
        module.update(state);
        for (; seen < bus.txLog.size(); seen++) {
            const CanFrame& f = bus.txLog[seen];
            if (isFcCts(f)) {
                for (const CanFrame& cf : waitingCfs) bus.injectRxFrame(cf);
                waitingCfs.clear();
                continue;
            }
            Bytes req = primaryRequest(f);
            if (req.empty() || req[0] == VEHICLE_CL250_TESTER_PRESENT_SID) continue;
            if (req[0] == VEHICLE_CL250_SESSION_SID) {
                bus.injectRxFrame(singleFrame({VEHICLE_CL250_SESSION_POSITIVE_SID, VEHICLE_CL250_SESSION_SUBFUNCTION,
                                               0x00, 0x32, 0x01, 0xF4}));
                continue;
            }
            Bytes answer = ecu(req);
            if (log != nullptr) {
                log->sentAt.push_back(t);
                log->answered.push_back(!answer.empty());
            }
            if (answer.empty()) continue;
            if (answer.size() <= 7) {
                bus.injectRxFrame(singleFrame(answer));
            } else {
                bus.injectRxFrame(firstFrame(answer));
                waitingCfs = consecutiveFrames(answer);
            }
        }
    }
}

// Starts the probe and confirms the session; the scan's first request goes out in this
// pass. Returns the clock for the next pass.
static unsigned long startScan(HondaCANModule& module, MockCanBus& bus, SystemState& state) {
    startProbe(module, state);
    bus.injectRxFrame(singleFrame({VEHICLE_CL250_SESSION_POSITIVE_SID, VEHICLE_CL250_SESSION_SUBFUNCTION,
                                   0x00, 0x32, 0x01, 0xF4}));
    setClock(5);
    module.update(state);
    return 5;
}

// ---------------------------------------------------------------------------

void test_table_indexes_used_by_these_tests(void) {
    TEST_ASSERT_TRUE(requestOf(kObdPids0) == Bytes({0x01, 0x00}));
    TEST_ASSERT_TRUE(requestOf(kObdPids1) == Bytes({0x01, 0x20}));
    TEST_ASSERT_TRUE(requestOf(kObdInfotypes) == Bytes({0x09, 0x00}));
    TEST_ASSERT_TRUE(requestOf(kVin) == Bytes({0x09, 0x02}));
    TEST_ASSERT_TRUE(requestOf(kDtcList) == Bytes({0x19, 0x02, 0xFF}));
    TEST_ASSERT_TRUE(requestOf(kDid0) == Bytes({0x22, 0xF4, 0x00}));
}

void test_rx_queue_and_drain_hold_the_whole_cf_burst(void) {
    TEST_ASSERT_TRUE(kCanRxQueueLen >= VEHICLE_CL250_FC_MAX_CF_BURST + kCanRxQueueBurstMargin);
    TEST_ASSERT_TRUE(kMaxRxFramesPerPass >= kCanRxQueueLen);
    // The generated burst bound matches the generated FF_DL bound: ceil((255 - 6) / 7).
    TEST_ASSERT_EQUAL_UINT32((VEHICLE_CL250_MAX_FF_DL - 6 + 6) / 7, VEHICLE_CL250_FC_MAX_CF_BURST);
}

void test_scan_waits_for_the_session_then_runs_once_then_polls_dids(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    TEST_ASSERT_TRUE(startProbe(module, state));
    // No session answer yet: nothing but the session request.
    setClock(50);
    module.update(state);
    TEST_ASSERT_EQUAL(2, bus.txLog.size());

    unsigned long t = 50;
    runScan(module, bus, state, t, defaultEcu);
    TEST_ASSERT_TRUE(module.discoveryScanDone());
    TEST_ASSERT_TRUE(anyLine("session confirmed"));
    TEST_ASSERT_TRUE(anyLine("[SCAN] done:"));
    TEST_ASSERT_EQUAL(0, bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED));

    // Afterwards: normal polling, and the scan never runs again.
    int firstEntry = countPrimaryRequests(bus, requestOf(kObdPids0));
    TEST_ASSERT_EQUAL(1, firstEntry);
    for (unsigned long end = t + 3000; t < end; t += 10) {
        setClock(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > 0);
    TEST_ASSERT_EQUAL(1, countPrimaryRequests(bus, requestOf(kObdPids0)));
}

void test_scan_starts_unconfirmed_one_retry_interval_after_the_session_request(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startProbe(module, state);
    setClock(kSessionRetryMs - 5);
    module.update(state);
    TEST_ASSERT_EQUAL(0, countPrimaryRequests(bus, requestOf(kObdPids0)));
    setClock(kSessionRetryMs + 5);
    module.update(state); // the session retry goes out first; the scan waits the base timeout
    setClock(kSessionRetryMs + 5 + kResponseTimeoutMs - 1);
    module.update(state);
    TEST_ASSERT_EQUAL(0, countPrimaryRequests(bus, requestOf(kObdPids0)));
    setClock(kSessionRetryMs + 5 + kResponseTimeoutMs);
    module.update(state);
    TEST_ASSERT_EQUAL(1, countPrimaryRequests(bus, requestOf(kObdPids0)));
    TEST_ASSERT_TRUE(anyLine("session NOT confirmed"));
}

void test_every_probe_frame_is_an_allowed_single_frame_or_the_exact_fc(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = startScan(module, bus, state);
    runScan(module, bus, state, t, defaultEcu);
    TEST_ASSERT_TRUE(module.discoveryScanDone());
    int fcs = 0;
    for (const CanFrame& f : bus.txLog) {
        TEST_ASSERT_TRUE(f.id == VEHICLE_CL250_REQUEST_ID || f.id == VEHICLE_CL250_FALLBACK_REQUEST_ID);
        TEST_ASSERT_TRUE(f.extended == (f.id == VEHICLE_CL250_REQUEST_ID));
        TEST_ASSERT_TRUE(vehicle_cl250_frame_allowed(f.data, f.dlc));
        bool single = (f.data[0] & 0xF0) == 0;
        TEST_ASSERT_TRUE(single || isFcCts(f));
        fcs += isFcCts(f) ? 1 : 0;
    }
    // One FC per segmented answer: VIN, CALID, ECU name, DTC list.
    TEST_ASSERT_EQUAL(4, fcs);
    TEST_ASSERT_EQUAL_UINT32(0, module.blockedFrameCount());
}

void test_one_request_in_flight_across_the_whole_scan(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = startScan(module, bus, state);
    ScanLog log;
    log.sentAt.push_back(t);
    log.answered.push_back(true); // the first request (01 00) is answered below
    bus.injectRxFrame(singleFrame(defaultEcu(requestOf(kObdPids0))));
    runScan(module, bus, state, t, defaultEcu, &log);
    TEST_ASSERT_TRUE(log.sentAt.size() > 10);
    for (size_t k = 0; k + 1 < log.sentAt.size(); k++) {
        // The next request only after an answer, or after the response timeout.
        TEST_ASSERT_TRUE(log.answered[k] || log.sentAt[k + 1] - log.sentAt[k] > kResponseTimeoutMs);
    }
}

void test_bitmap_gates_skip_unsupported_entries_and_their_chain(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = startScan(module, bus, state);
    runScan(module, bus, state, t, defaultEcu);
    // 01 00 marks 0x20, so 01 20 is sent; 01 20 does not mark 0x40: 01 40 .. 01 E0 skipped.
    TEST_ASSERT_EQUAL(1, countPrimaryRequests(bus, requestOf(kObdPids1)));
    for (uint8_t i = 2; i < 8; i++) {
        TEST_ASSERT_EQUAL(0, countPrimaryRequests(bus, requestOf(i)));
        TEST_ASSERT_TRUE(module.testScan().status(i) == DiscoveryScan::Status::SKIPPED);
    }
    // 22 F4 00 marks F420; 22 F4 20 gets no answer, so its dependents are skipped too.
    TEST_ASSERT_TRUE(module.testScan().status(kDid0 + 1) == DiscoveryScan::Status::NO_ANSWER);
    for (uint8_t i = kDid0 + 2; i < VEHICLE_CL250_SCAN_COUNT; i++) {
        TEST_ASSERT_TRUE(module.testScan().status(i) == DiscoveryScan::Status::SKIPPED);
    }
    TEST_ASSERT_TRUE(module.testScan().status(13) == DiscoveryScan::Status::POSITIVE); // 09 0A
    TEST_ASSERT_TRUE(module.testScan().status(12) == DiscoveryScan::Status::NRC);      // 09 08
    TEST_ASSERT_TRUE(anyLine("supported: 01 03 04 05 06 07 0C 0D 0E 0F 10 11 13 15 1C 1F 20"));
    TEST_ASSERT_TRUE(anyLine("supported: F404 F405 F420"));
}

void test_a_gate_with_an_nrc_skips_its_dependents(void) {
    DiscoveryScan scan(captureLine);
    scan.recordNrc(kObdInfotypes, 0x12);
    TEST_ASSERT_FALSE(scan.gateSupports(kObdInfotypes, 0x02));
    const uint8_t bitmap[] = {0x49, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};
    DiscoveryScan scan2(captureLine);
    scan2.recordPositive(kObdInfotypes, bitmap, 4); // too short for the bitmap
    TEST_ASSERT_FALSE(scan2.gateSupports(kObdInfotypes, 0x02));
    scan2.recordPositive(kObdInfotypes, bitmap, sizeof(bitmap));
    TEST_ASSERT_TRUE(scan2.gateSupports(kObdInfotypes, 0x02));
    TEST_ASSERT_TRUE(scan2.gateSupports(kObdInfotypes, 0x20)); // bit 0 of the last byte
    TEST_ASSERT_FALSE(scan2.gateSupports(kObdInfotypes, 0x00)); // outside the bitmap
    TEST_ASSERT_FALSE(scan2.gateSupports(kObdInfotypes, 0x21));
}

// Drives the scan up to the VIN request: every earlier entry answered by defaultEcu.
static unsigned long scanUpTo(HondaCANModule& module, MockCanBus& bus, SystemState& state, uint8_t entry) {
    unsigned long t = startScan(module, bus, state);
    size_t seen = 0;
    std::vector<CanFrame> waitingCfs;
    while (!lastPrimaryRequestIs(bus, requestOf(entry)) && t < 60000) {
        for (; seen < bus.txLog.size(); seen++) {
            const CanFrame& f = bus.txLog[seen];
            if (isFcCts(f)) {
                for (const CanFrame& cf : waitingCfs) bus.injectRxFrame(cf);
                waitingCfs.clear();
                continue;
            }
            Bytes req = primaryRequest(f);
            if (req.empty() || req[0] == VEHICLE_CL250_TESTER_PRESENT_SID || req[0] == VEHICLE_CL250_SESSION_SID) continue;
            Bytes a = defaultEcu(req);
            if (a.empty()) continue;
            if (a.size() <= 7) {
                bus.injectRxFrame(singleFrame(a));
            } else {
                bus.injectRxFrame(firstFrame(a));
                waitingCfs = consecutiveFrames(a);
            }
        }
        t += 5;
        setClock(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(lastPrimaryRequestIs(bus, requestOf(entry)));
    bus.clearTxLog();
    g_lines.clear();
    return t;
}

void test_fc_is_sent_once_byte_exact_on_the_paired_request_id(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = defaultEcu(requestOf(kDtcList));
    bus.injectRxFrame(firstFrame(answer));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.txLog.size());
    TEST_ASSERT_TRUE(isFcCts(bus.txLog[0]));
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, bus.txLog[0].id);
    TEST_ASSERT_TRUE(bus.txLog[0].extended);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(vehicle_cl250_fc_cts, bus.txLog[0].data, 8);
    TEST_ASSERT_TRUE(module.receivingSegmented());
    for (const CanFrame& cf : consecutiveFrames(answer)) bus.injectRxFrame(cf);
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_FALSE(module.receivingSegmented());
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
    TEST_ASSERT_TRUE(anyLine("positive, 15 bytes"));
    TEST_ASSERT_TRUE(anyLine("+000: 59 02 FF 01 23 45 2F 06 78 9A 08 0B CD EF 2F"));
    TEST_ASSERT_EQUAL(1, countFc(bus));
}

void test_fc_for_a_fallback_answer_goes_to_the_fallback_request_id(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = defaultEcu(requestOf(kDtcList));
    bus.injectRxFrame(firstFrame(answer, VEHICLE_CL250_FALLBACK_RESPONSE_ID, false));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.txLog.size());
    TEST_ASSERT_TRUE(isFcCts(bus.txLog[0]));
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_FALLBACK_REQUEST_ID, bus.txLog[0].id);
    TEST_ASSERT_FALSE(bus.txLog[0].extended);
    // CFs on the other response ID are not part of this reception.
    for (const CanFrame& cf : consecutiveFrames(answer)) bus.injectRxFrame(cf);
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.receivingSegmented());
    for (const CanFrame& cf : consecutiveFrames(answer, VEHICLE_CL250_FALLBACK_RESPONSE_ID, false)) bus.injectRxFrame(cf);
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
}

void test_no_fc_for_a_first_frame_that_does_not_answer_the_request_in_flight(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    // Another service's answer, and the right service with the wrong report type.
    bus.injectRxFrame(firstFrame(withText({0x49, 0x02, 0x01}, kVinText, 17)));
    setClock(t += 5);
    module.update(state);
    Bytes wrongEcho = defaultEcu(requestOf(kDtcList));
    wrongEcho[1] = 0x01;
    bus.injectRxFrame(firstFrame(wrongEcho));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(0, countFc(bus));
    TEST_ASSERT_FALSE(module.receivingSegmented());
    // The request in flight then times out as usual; nothing is sent in the quiet time.
    TEST_ASSERT_EQUAL(0, bus.txLog.size());
}

void test_no_fc_without_a_scan_request_in_flight(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = startScan(module, bus, state);
    runScan(module, bus, state, t, defaultEcu);
    bus.clearTxLog();
    // Normal polling: an FF answering the DID in flight gets no FC and is no DID timeout.
    while (bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) == 0) {
        setClock(t += 5);
        module.update(state);
    }
    Bytes did = {0x62, 0xF4, 0x0C, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    bus.injectRxFrame(firstFrame(did));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(0, countFc(bus));
    TEST_ASSERT_EQUAL_UINT32(0, module.unansweredDidCount());
    // An unsolicited FF while idle: no FC either.
    bus.injectRxFrame(firstFrame(defaultEcu(requestOf(kDtcList))));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(0, countFc(bus));
}

void test_a_too_long_or_invalid_first_frame_gets_no_fc_and_a_quiet_time(void) {
    for (int announced : {(int)VEHICLE_CL250_MAX_FF_DL + 1, 7, 0}) {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        unsigned long t = scanUpTo(module, bus, state, kDtcList);
        bus.injectRxFrame(firstFrame(defaultEcu(requestOf(kDtcList)), VEHICLE_CL250_RESPONSE_ID, true, announced));
        setClock(t += 5);
        module.update(state);
        TEST_ASSERT_EQUAL(0, countFc(bus));
        // FF_DL 0 announces the 32-bit escape length: its SID is not at byte 2, so it cannot
        // be matched to the request in flight, which then just runs into its timeout.
        TEST_ASSERT_TRUE(module.testScan().status(kDtcList) ==
                         (announced == 0 ? DiscoveryScan::Status::PENDING : DiscoveryScan::Status::FF_REFUSED));
        // yaml: no request at all (not even tester present) for response_timeout_max_ms.
        unsigned long refusedAt = t;
        while (t < refusedAt + kResponsePendingMaxMs - 5) {
            setClock(t += 5);
            module.update(state);
        }
        TEST_ASSERT_EQUAL(0, bus.txLog.size());
        setClock(t += 20);
        module.update(state);
        TEST_ASSERT_TRUE(bus.txLog.size() > 0);
    }
}

void test_a_full_burst_of_the_maximum_length_completes_in_one_pass(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = {0x59, 0x02, 0xFF};
    while (answer.size() < VEHICLE_CL250_MAX_FF_DL) answer.push_back((uint8_t)answer.size());
    bus.injectRxFrame(firstFrame(answer));
    setClock(t += 5);
    module.update(state);
    std::vector<CanFrame> cfs = consecutiveFrames(answer);
    TEST_ASSERT_EQUAL(VEHICLE_CL250_FC_MAX_CF_BURST, cfs.size());
    for (const CanFrame& cf : cfs) bus.injectRxFrame(cf);
    setClock(t += 1);
    module.update(state);
    TEST_ASSERT_FALSE(module.receivingSegmented());
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
    TEST_ASSERT_TRUE(anyLine("positive, 255 bytes"));
}

static void expectAbortedWithoutData(HondaCANModule& module, const char* reason) {
    TEST_ASSERT_FALSE(module.receivingSegmented());
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::ABORTED);
    TEST_ASSERT_TRUE(anyLine(reason));
    TEST_ASSERT_FALSE(anyLine("+000:"));
}

void test_a_sequence_gap_aborts_the_reception(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = {0x59, 0x02, 0xFF};
    while (answer.size() < 40) answer.push_back(0x11);
    bus.injectRxFrame(firstFrame(answer));
    setClock(t += 5);
    module.update(state);
    std::vector<CanFrame> cfs = consecutiveFrames(answer);
    bus.injectRxFrame(cfs[0]);
    bus.injectRxFrame(cfs[2]); // SN 1, then 3
    setClock(t += 5);
    module.update(state);
    expectAbortedWithoutData(module, "sequence gap");
    // The rest of the burst is dropped, and nothing is sent in the quiet time.
    bus.clearTxLog();
    for (size_t k = 3; k < cfs.size(); k++) bus.injectRxFrame(cfs[k]);
    for (int k = 0; k < 100; k++) {
        setClock(t += 5);
        module.update(state);
    }
    TEST_ASSERT_EQUAL(0, bus.txLog.size()); // no request and no new FC
}

void test_n_cr_aborts_but_cfs_already_queued_after_a_slow_pass_are_not_late(void) {
    {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        unsigned long t = scanUpTo(module, bus, state, kDtcList);
        Bytes answer = defaultEcu(requestOf(kDtcList));
        bus.injectRxFrame(firstFrame(answer));
        setClock(t += 5);
        module.update(state);
        setClock(t += VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS);
        module.update(state);
        TEST_ASSERT_TRUE(module.receivingSegmented()); // exactly N_Cr: not yet
        setClock(t += 1);
        module.update(state);
        expectAbortedWithoutData(module, "N_Cr timeout");
    }
    g_lines.clear();
    {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        unsigned long t = scanUpTo(module, bus, state, kDtcList);
        Bytes answer = defaultEcu(requestOf(kDtcList));
        bus.injectRxFrame(firstFrame(answer));
        setClock(t += 5);
        module.update(state);
        for (const CanFrame& cf : consecutiveFrames(answer)) bus.injectRxFrame(cf);
        setClock(t += 500); // a slow loop pass (BLE, GPS): the CFs waited in the queue
        module.update(state);
        TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
    }
}

void test_a_lost_frame_aborts_the_reception(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = {0x59, 0x02, 0xFF};
    while (answer.size() < 40) answer.push_back(0x22);
    bus.injectRxFrame(firstFrame(answer));
    setClock(t += 5);
    module.update(state);
    std::vector<CanFrame> cfs = consecutiveFrames(answer);
    bus.injectRxFrame(cfs[0]);
    bus.rxLost++; // the queue overflowed
    setClock(t += 5);
    module.update(state);
    expectAbortedWithoutData(module, "lost frame");
}

void test_a_second_first_frame_or_a_single_frame_aborts_without_a_second_fc(void) {
    for (int variant = 0; variant < 2; variant++) {
        g_lines.clear();
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        unsigned long t = scanUpTo(module, bus, state, kDtcList);
        Bytes answer = defaultEcu(requestOf(kDtcList));
        bus.injectRxFrame(firstFrame(answer));
        setClock(t += 5);
        module.update(state);
        TEST_ASSERT_EQUAL(1, countFc(bus));
        bus.injectRxFrame(variant == 0 ? firstFrame(answer) : singleFrame({0x7F, 0x19, 0x31}));
        setClock(t += 5);
        module.update(state);
        TEST_ASSERT_EQUAL(1, countFc(bus));
        expectAbortedWithoutData(module, "unexpected frame");
    }
}

void test_nothing_is_sent_while_a_segmented_answer_is_received(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = {0x59, 0x02, 0xFF};
    while (answer.size() < 6 + 7 * 20) answer.push_back(0x33); // 20 CFs
    bus.injectRxFrame(firstFrame(answer));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.txLog.size()); // the FC
    std::vector<CanFrame> cfs = consecutiveFrames(answer);
    // One CF every 90 ms (< N_Cr): 1.8 s, longer than the tester present period.
    for (const CanFrame& cf : cfs) {
        for (int k = 0; k < 18; k++) {
            setClock(t += 5);
            module.update(state);
        }
        bus.injectRxFrame(cf);
    }
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
    TEST_ASSERT_EQUAL(1, bus.txLog.size()); // still only the FC: no tester present, no request
}

void test_scan_nrc_0x78_extends_the_wait(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    bus.injectRxFrame(singleFrame({0x7F, 0x19, UDS_NRC_RESPONSE_PENDING}));
    setClock(t += 5);
    module.update(state);
    setClock(t += kResponseTimeoutMs + 20); // past the base timeout, within the doubled one
    module.update(state);
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::PENDING);
    bus.injectRxFrame(singleFrame({0x59, 0x02, 0xFF}));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
}

// ---------------------------------------------------------------------------
// VIN mask (D-059 item 3, D-033)
// ---------------------------------------------------------------------------

static void expectNoVinLeak(void) {
    for (const std::string& l : g_lines) {
        TEST_ASSERT_TRUE(l.find("SECRET") == std::string::npos);
        TEST_ASSERT_TRUE(l.find("54 53 54") == std::string::npos); // "TST" as hex
        TEST_ASSERT_TRUE(l.find("53 45 43") == std::string::npos); // "SEC" as hex
    }
}

void test_vin_prints_the_wmi_only(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = startScan(module, bus, state);
    runScan(module, bus, state, t, defaultEcu);
    TEST_ASSERT_TRUE(module.testScan().status(kVin) == DiscoveryScan::Status::POSITIVE);
    TEST_ASSERT_TRUE(anyLine("VIN WMI=TST (other 14 characters masked"));
    expectNoVinLeak();
    // The other infotypes are not sensitive and are printed.
    TEST_ASSERT_TRUE(anyLine("43 41 4C 49 44")); // "CALID"
}

void test_vin_without_nodi_and_with_an_odd_length(void) {
    const uint8_t noNodi[] = {0x49, 0x02, 'T', 'S', 'T', 'S', 'E', 'C', 'R', 'E', 'T', 'V', 'I', 'N', '1', '2', '3', '4', '5'};
    DiscoveryScan a(captureLine);
    a.recordPositive(kVin, noNodi, sizeof(noNodi));
    TEST_ASSERT_TRUE(anyLine("VIN WMI=TST"));
    g_lines.clear();
    const uint8_t odd[] = {0x49, 0x02, 0x01, 'T', 'S', 'T', 'S', 'E', 'C', 'R', 'E', 'T'};
    DiscoveryScan b(captureLine);
    b.recordPositive(kVin, odd, sizeof(odd));
    TEST_ASSERT_TRUE(anyLine("positive, 12 bytes (masked, D-033)"));
    TEST_ASSERT_FALSE(anyLine("WMI="));
    expectNoVinLeak();
    g_lines.clear();
    uint8_t unprintable[20];
    memcpy(unprintable, noNodi, sizeof(noNodi));
    unprintable[10] = 0x00;
    DiscoveryScan c(captureLine);
    c.recordPositive(kVin, unprintable, 19);
    TEST_ASSERT_FALSE(anyLine("WMI="));
    expectNoVinLeak();
}

void test_an_aborted_vin_reception_prints_nothing_of_it(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kVin);
    Bytes vin = withText({0x49, 0x02, 0x01}, kVinText, 17);
    bus.injectRxFrame(firstFrame(vin));
    setClock(t += 5);
    module.update(state);
    std::vector<CanFrame> cfs = consecutiveFrames(vin);
    bus.injectRxFrame(cfs[1]); // SN 2 first: gap
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.testScan().status(kVin) == DiscoveryScan::Status::ABORTED);
    TEST_ASSERT_FALSE(anyLine("+000:"));
    expectNoVinLeak();
}

void test_the_vin_entry_is_found_in_the_generated_table(void) {
    DiscoveryScan scan(captureLine);
    TEST_ASSERT_EQUAL_UINT8(kVin, scan.vinIndex());
    TEST_ASSERT_TRUE(scan.sensitive(kVin));
    TEST_ASSERT_FALSE(scan.sensitive(kVin + 1)); // CALID
    TEST_ASSERT_FALSE(scan.sensitive(kObdInfotypes));
}

void test_the_mask_fails_closed_when_the_vin_entry_is_not_unique(void) {
    vehicle_cl250_scan_t table[3];
    memcpy(table, &vehicle_cl250_discovery_scan[kObdInfotypes], sizeof(table)); // 09 00, 09 02, 09 04
    TEST_ASSERT_EQUAL_UINT8(1, DiscoveryScan::findVinIndex(table, 3));
    table[2] = table[1]; // two VIN entries
    TEST_ASSERT_EQUAL_UINT8(DiscoveryScan::kNone, DiscoveryScan::findVinIndex(table, 3));
    table[1] = table[0];
    table[2] = table[0]; // none
    TEST_ASSERT_EQUAL_UINT8(DiscoveryScan::kNone, DiscoveryScan::findVinIndex(table, 3));

    // Without an identified VIN entry, no service 0x09 answer but the bitmap is printed.
    DiscoveryScan scan(captureLine);
    scan.testSetVinIndex(DiscoveryScan::kNone);
    TEST_ASSERT_TRUE(scan.sensitive(kVin));
    TEST_ASSERT_TRUE(scan.sensitive(kVin + 1));
    TEST_ASSERT_FALSE(scan.sensitive(kObdInfotypes));
    TEST_ASSERT_FALSE(scan.sensitive(kDtcList));
    Bytes vin = withText({0x49, 0x02, 0x01}, kVinText, 17);
    scan.recordPositive(kVin, vin.data(), (uint16_t)vin.size());
    TEST_ASSERT_TRUE(anyLine("positive, 20 bytes (masked, D-033)"));
    expectNoVinLeak();
}

// ---------------------------------------------------------------------------
// safety-reviewer follow-ups (MAJOR-1/2, MINOR-1..4)
// ---------------------------------------------------------------------------

void test_a_foreign_request_behind_the_first_frame_latches_before_any_fc(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    bus.injectRxFrame(firstFrame(defaultEcu(requestOf(kDtcList))));
    CanFrame foreign = frameOn((VEHICLE_CL250_REQUEST_ID & ~0xFFu) | 0xF2u, true); // another tester's SA
    foreign.data[0] = 0x02;
    foreign.data[1] = 0x3E;
    foreign.data[2] = 0x80;
    bus.injectRxFrame(foreign);
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL(0, bus.txLog.size()); // no FC: the drain saw the foreign tester first
}

void test_the_fc_waits_for_a_backlogged_drain_and_is_dropped_when_late(void) {
    for (int late = 0; late < 2; late++) {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        unsigned long t = scanUpTo(module, bus, state, kDtcList);
        Bytes answer = defaultEcu(requestOf(kDtcList));
        bus.injectRxFrame(firstFrame(answer));
        CanFrame noise = frameOn(0x123, false);
        for (unsigned k = 0; k < kMaxRxFramesPerPass; k++) bus.injectRxFrame(noise);
        setClock(t += 5);
        module.update(state); // BACKLOG: nothing is sent
        TEST_ASSERT_EQUAL(0, bus.txLog.size());
        setClock(t += (late ? kResponseTimeoutMs + 5 : 5));
        module.update(state);
        if (late) {
            TEST_ASSERT_EQUAL(0, countFc(bus));
            TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::ABORTED);
            TEST_ASSERT_TRUE(anyLine("FC too late"));
        } else {
            TEST_ASSERT_EQUAL(1, countFc(bus));
            for (const CanFrame& cf : consecutiveFrames(answer)) bus.injectRxFrame(cf);
            setClock(t += 5);
            module.update(state);
            TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
        }
    }
}

void test_a_frame_lost_between_the_request_and_the_first_frame_gives_no_fc(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    bus.rxLost++;
    bus.injectRxFrame(firstFrame(defaultEcu(requestOf(kDtcList))));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(0, countFc(bus));
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::ABORTED);
    TEST_ASSERT_TRUE(anyLine("lost frame before the FC"));
}

void test_tester_present_and_the_next_scan_request_never_share_a_pass(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    // Answer, and let the tester present period run out in the same step.
    bus.injectRxFrame(singleFrame({0x59, 0x02, 0xFF}));
    setClock(t += kTesterPresentPeriodMs);
    module.update(state); // the answer resolves the request
    size_t before = bus.txLog.size();
    setClock(t += 5);
    module.update(state); // tester present is due: it goes alone
    int requests = 0;
    for (size_t i = before; i < bus.txLog.size(); i++) requests += primaryRequest(bus.txLog[i]).empty() ? 0 : 1;
    TEST_ASSERT_EQUAL(1, requests);
    TEST_ASSERT_EQUAL_HEX8(VEHICLE_CL250_TESTER_PRESENT_SID, bus.txLog[before].data[1]);
    // The next scan request waits for the base response timeout after the tester present.
    unsigned long tpAt = t;
    while (!lastPrimaryRequestIs(bus, requestOf(kDid0)) && t < tpAt + 1000) {
        setClock(t += 1);
        module.update(state);
    }
    TEST_ASSERT_TRUE(lastPrimaryRequestIs(bus, requestOf(kDid0)));
    TEST_ASSERT_TRUE(t - tpAt >= kResponseTimeoutMs);
}

void test_the_whole_scan_sends_at_most_one_request_per_pass(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = startScan(module, bus, state);
    // A slow ECU: every answer arrives 400 ms late, so tester present falls due often.
    size_t seen = 0;
    std::vector<std::pair<unsigned long, Bytes>> due;
    std::vector<CanFrame> waitingCfs;
    while (!module.discoveryScanDone() && t < 200000) {
        t += 5;
        setClock(t);
        for (size_t k = 0; k < due.size();) {
            if (due[k].first <= t) {
                const Bytes& a = due[k].second;
                if (a.size() <= 7) {
                    bus.injectRxFrame(singleFrame(a));
                } else {
                    bus.injectRxFrame(firstFrame(a));
                    waitingCfs = consecutiveFrames(a);
                }
                due.erase(due.begin() + (long)k);
            } else {
                k++;
            }
        }
        size_t before = bus.txLog.size();
        module.update(state);
        int requests = 0;
        for (size_t i = before; i < bus.txLog.size(); i++) {
            const CanFrame& f = bus.txLog[i];
            if (isFcCts(f)) {
                for (const CanFrame& cf : waitingCfs) bus.injectRxFrame(cf);
                waitingCfs.clear();
                continue;
            }
            Bytes req = primaryRequest(f);
            if (req.empty()) continue;
            requests++;
            if (req[0] == VEHICLE_CL250_TESTER_PRESENT_SID || req[0] == VEHICLE_CL250_SESSION_SID) continue;
            Bytes a = defaultEcu(req);
            if (!a.empty()) due.push_back(std::make_pair(t + 40, a));
        }
        TEST_ASSERT_TRUE(requests <= 1);
        (void)seen;
    }
    TEST_ASSERT_TRUE(module.discoveryScanDone());
}

void test_a_stray_first_frame_starts_a_quiet_time_in_which_ours_gets_no_fc(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    bus.injectRxFrame(firstFrame(withText({0x49, 0x02, 0x01}, kVinText, 17))); // not ours
    setClock(t += 5);
    module.update(state);
    bus.injectRxFrame(firstFrame(defaultEcu(requestOf(kDtcList)))); // ours, in the quiet time
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(0, countFc(bus));
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::FF_REFUSED);
}

void test_first_frames_on_both_response_ids_get_one_fc_and_the_first_completes(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = defaultEcu(requestOf(kDtcList));
    bus.injectRxFrame(firstFrame(answer));
    bus.injectRxFrame(firstFrame(answer, VEHICLE_CL250_FALLBACK_RESPONSE_ID, false));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(1, countFc(bus));
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, bus.txLog[0].id);
    for (const CanFrame& cf : consecutiveFrames(answer)) bus.injectRxFrame(cf);
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
    // A late FF on the other ID during the reception does not abort it either.
    TEST_ASSERT_EQUAL(1, countFc(bus));
}

void test_a_reception_longer_than_the_maximum_wait_is_aborted(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    Bytes answer = {0x59, 0x02, 0xFF};
    while (answer.size() < VEHICLE_CL250_MAX_FF_DL) answer.push_back(0x44); // 36 CFs
    bus.injectRxFrame(firstFrame(answer));
    setClock(t += 5);
    module.update(state);
    unsigned long started = t;
    // One CF every 90 ms (< N_Cr): 36 CFs would take 3.2 s.
    for (const CanFrame& cf : consecutiveFrames(answer)) {
        for (int k = 0; k < 18 && module.receivingSegmented(); k++) {
            setClock(t += 5);
            module.update(state);
        }
        if (!module.receivingSegmented()) break;
        bus.injectRxFrame(cf);
    }
    TEST_ASSERT_FALSE(module.receivingSegmented());
    TEST_ASSERT_TRUE(t - started <= kResponsePendingMaxMs + 100);
    expectAbortedWithoutData(module, "reception too long");
}

void test_nrc_0x78_then_a_segmented_answer(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    unsigned long t = scanUpTo(module, bus, state, kDtcList);
    bus.injectRxFrame(singleFrame({0x7F, 0x19, UDS_NRC_RESPONSE_PENDING}));
    setClock(t += 150);
    module.update(state);
    Bytes answer = defaultEcu(requestOf(kDtcList));
    bus.injectRxFrame(firstFrame(answer));
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_EQUAL(1, countFc(bus));
    for (const CanFrame& cf : consecutiveFrames(answer)) bus.injectRxFrame(cf);
    setClock(t += 5);
    module.update(state);
    TEST_ASSERT_TRUE(module.testScan().status(kDtcList) == DiscoveryScan::Status::POSITIVE);
}

void test_supported_ids_do_not_wrap_at_the_end_of_a_range(void) {
    DiscoveryScan scan(captureLine);
    const uint8_t last = VEHICLE_CL250_SCAN_COUNT - 1; // 22 F4 E0
    TEST_ASSERT_TRUE(requestOf(last) == Bytes({0x22, 0xF4, 0xE0}));
    const uint8_t answer[] = {0x62, 0xF4, 0xE0, 0x80, 0x00, 0x00, 0x01};
    scan.recordPositive(last, answer, sizeof(answer));
    TEST_ASSERT_TRUE(anyLine("supported: F4E1 F500"));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_table_indexes_used_by_these_tests);
    RUN_TEST(test_rx_queue_and_drain_hold_the_whole_cf_burst);
    RUN_TEST(test_scan_waits_for_the_session_then_runs_once_then_polls_dids);
    RUN_TEST(test_scan_starts_unconfirmed_one_retry_interval_after_the_session_request);
    RUN_TEST(test_every_probe_frame_is_an_allowed_single_frame_or_the_exact_fc);
    RUN_TEST(test_one_request_in_flight_across_the_whole_scan);
    RUN_TEST(test_bitmap_gates_skip_unsupported_entries_and_their_chain);
    RUN_TEST(test_a_gate_with_an_nrc_skips_its_dependents);
    RUN_TEST(test_fc_is_sent_once_byte_exact_on_the_paired_request_id);
    RUN_TEST(test_fc_for_a_fallback_answer_goes_to_the_fallback_request_id);
    RUN_TEST(test_no_fc_for_a_first_frame_that_does_not_answer_the_request_in_flight);
    RUN_TEST(test_no_fc_without_a_scan_request_in_flight);
    RUN_TEST(test_a_too_long_or_invalid_first_frame_gets_no_fc_and_a_quiet_time);
    RUN_TEST(test_a_full_burst_of_the_maximum_length_completes_in_one_pass);
    RUN_TEST(test_a_sequence_gap_aborts_the_reception);
    RUN_TEST(test_n_cr_aborts_but_cfs_already_queued_after_a_slow_pass_are_not_late);
    RUN_TEST(test_a_lost_frame_aborts_the_reception);
    RUN_TEST(test_a_second_first_frame_or_a_single_frame_aborts_without_a_second_fc);
    RUN_TEST(test_nothing_is_sent_while_a_segmented_answer_is_received);
    RUN_TEST(test_scan_nrc_0x78_extends_the_wait);
    RUN_TEST(test_vin_prints_the_wmi_only);
    RUN_TEST(test_vin_without_nodi_and_with_an_odd_length);
    RUN_TEST(test_an_aborted_vin_reception_prints_nothing_of_it);
    RUN_TEST(test_the_vin_entry_is_found_in_the_generated_table);
    RUN_TEST(test_the_mask_fails_closed_when_the_vin_entry_is_not_unique);
    RUN_TEST(test_a_foreign_request_behind_the_first_frame_latches_before_any_fc);
    RUN_TEST(test_the_fc_waits_for_a_backlogged_drain_and_is_dropped_when_late);
    RUN_TEST(test_a_frame_lost_between_the_request_and_the_first_frame_gives_no_fc);
    RUN_TEST(test_tester_present_and_the_next_scan_request_never_share_a_pass);
    RUN_TEST(test_the_whole_scan_sends_at_most_one_request_per_pass);
    RUN_TEST(test_a_stray_first_frame_starts_a_quiet_time_in_which_ours_gets_no_fc);
    RUN_TEST(test_first_frames_on_both_response_ids_get_one_fc_and_the_first_completes);
    RUN_TEST(test_a_reception_longer_than_the_maximum_wait_is_aborted);
    RUN_TEST(test_nrc_0x78_then_a_segmented_answer);
    RUN_TEST(test_supported_ids_do_not_wrap_at_the_end_of_a_range);
    return UNITY_END();
}
