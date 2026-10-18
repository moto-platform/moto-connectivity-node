#include <unity.h>
#include "../../src/HondaCANModule.h"
#include "../mocks/MockCanBus.h"
#include "../mocks/MockTesterLatchStore.h"
#include "../../src/IsoTpCan.h"
#include "uds_iso14229.h" // generated (gen/c/conn/, D-040)
#include <vector>
#include <cstring>
#include <cstdlib>

// [env:native] sets test_build_src = no (most of src/ needs ESP32-only libraries
// that don't exist on the host), so this suite pulls in the one real implementation
// file it needs directly, as a single translation unit -- HondaCANModule.cpp itself
// is completely unmodified for this to work; it only depends on ICanBus + the two
// stubbed Arduino symbols (Serial, millis/delay) already provided for test_native.
#include "../../src/HondaCANModule.cpp"
#include "../../external/moto-vehicle-defs/gen/c/conn/vehicle_cl250.c"

// G5.1 -- Native tests for HondaCANModule's actual UDS protocol logic (request state
// machine, NRC handling, ISO-TP frame checking, bus-off recovery, ECU-presence
// detection), run against a MockCanBus instead of real TWAI hardware. This is the
// exact same HondaCANModule.cpp/.h that runs on the ESP32 with a TwaiCanBus -- only
// the injected ICanBus differs, so this suite exercises real, unmodified protocol code.

static unsigned long gT0; // the clock base; a test may move it to straddle the millis() wrap
static const unsigned long kT0 = 5000 + kListenOnlyWindowMs + 10;

void setUp(void) { gT0 = kT0; }
void tearDown(void) {}

// Q-018: begin() only starts the listen-only window, so a test's own timeline starts
// when the window is over. setClock(t) is relative to that moment: setClock(0) is the
// first normal-mode pass, which sends the session request (what begin() did before the
// window existed).

static void setClock(unsigned long relativeMs) {
    test_setMillis(gT0 + relativeMs);
}

// Engine speed is the first table entry, so it is the first DID due after the session
// request at setClock(0): one generated poll period later (110 ms since D-053).
static const unsigned long kRpmDueMs = vehicle_cl250_dids[VEHICLE_CL250_IDX_ENGINE_SPEED].poll_period_ms;

static bool startPolling(HondaCANModule& module, SystemState& state) {
    test_setMillis(gT0 - kListenOnlyWindowMs - 10);
    if (!module.begin()) return false;
    module.update(state); // first pass: the listen-only window starts
    test_setMillis(gT0 - 10);
    module.update(state); // window over: normal mode
    setClock(0);
    module.update(state); // drain, then the session request
    return !module.listening() && !module.latchedOff();
}

// Builds a positive ReadDataByIdentifier response (SID 0x62) as a real Honda ECU
// would send it: [PCI][0x62][DID_hi][DID_lo][payload...].
static CanFrame makePositiveResponse(uint16_t did, uint8_t d4, uint8_t d5 = 0, uint8_t dlc = 6) {
    CanFrame f;
    f.id = VEHICLE_CL250_RESPONSE_ID;
    f.extended = true;
    f.dlc = dlc;
    f.data[0] = dlc - 1; // PCI: single frame, low nibble = payload length (not checked by value)
    f.data[1] = 0x62;
    f.data[2] = (did >> 8) & 0xFF;
    f.data[3] = did & 0xFF;
    f.data[4] = d4;
    f.data[5] = d5;
    return f;
}

static CanFrame makeNegativeResponse(uint8_t echoedSid, uint8_t nrc) {
    CanFrame f;
    f.id = VEHICLE_CL250_RESPONSE_ID;
    f.extended = true;
    f.dlc = 4;
    f.data[0] = 0x03;
    f.data[1] = 0x7F;
    f.data[2] = echoedSid;
    f.data[3] = nrc;
    return f;
}

static CanFrame makeSessionConfirm() {
    // Positive DiagnosticSessionControl response: [PCI=6][0x50][sub=0x03][P2 hi/lo][P2* hi/lo]
    CanFrame f;
    f.id = VEHICLE_CL250_RESPONSE_ID;
    f.extended = true;
    f.dlc = 8;
    const uint8_t d[8] = {0x06, VEHICLE_CL250_SESSION_POSITIVE_SID, VEHICLE_CL250_SESSION_SUBFUNCTION,
                          0x00, 0x32, 0x01, 0xF4, VEHICLE_CL250_PADDING_BYTE};
    for (int i = 0; i < 8; i++) f.data[i] = d[i];
    return f;
}

static CanFrame makeMultiFrameFirstFrame() {
    // ISO-TP First Frame: PCI high nibble 0x1. data[1] is intentionally NOT 0x62/0x7F/
    // 0x50 -- it's part of the multi-frame length field, and must never be parsed as a SID.
    CanFrame f;
    f.id = VEHICLE_CL250_RESPONSE_ID;
    f.extended = true;
    f.dlc = 8;
    f.data[0] = 0x10; // First Frame, length high nibble
    f.data[1] = 0x14; // length low byte -- NOT a SID
    return f;
}

// ---------------------------------------------------------------------------

void test_rpm_decode_updates_state_and_ecu_presence(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;

    TEST_ASSERT_TRUE(startPolling(module, state));

    setClock(100);
    module.update(state); // IDLE -> sends RPM request (highest priority, cadence 50ms) -> WAITING

    // 4500 RPM encoded as raw*4 per HondaCANModule's decode (raw/4.0f == 4500 -> raw=18000=0x4650)
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));

    setClock(110);
    module.update(state); // drains the response -> state.engine.rpm updated, WAITING -> COMPLETE -> IDLE

    TEST_ASSERT_EQUAL_FLOAT(4500.0f, state.engine.rpm);
    TEST_ASSERT_EQUAL_UINT32(gT0 + 110, state.engine.rpmUpdatedMs);

    // Q-018: RX is drained at the start of update(), before ECU presence is derived,
    // so presence flips true in the same pass that recorded the response.
    TEST_ASSERT_TRUE(state.engine.ecuPresent);
}

void test_session_confirm_is_recognized(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    bus.injectRxFrame(makeSessionConfirm());
    setClock(100);
    module.update(state);

    // No direct getter for _sessionConfirmed, but a confirmed session must stop the
    // 2-second retry loop from re-sending 0x10 0x03 -- verify no new session request
    // appears well past SESSION_RETRY_INTERVAL_MS (2000ms).
    size_t sessionReqsBefore = 0;
    for (const CanFrame& f : bus.txLog) {
        if (f.dlc >= 3 && f.data[1] == 0x10 && f.data[2] == 0x03) sessionReqsBefore++;
    }

    setClock(5000);
    module.update(state);

    size_t sessionReqsAfter = 0;
    for (const CanFrame& f : bus.txLog) {
        if (f.dlc >= 3 && f.data[1] == 0x10 && f.data[2] == 0x03) sessionReqsAfter++;
    }
    TEST_ASSERT_EQUAL(sessionReqsBefore, sessionReqsAfter);
}

void test_nrc_other_than_pending_resolves_request_without_corrupting_state(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    setClock(50);
    module.update(state); // RPM request sent, WAITING

    bus.injectRxFrame(makeNegativeResponse(0x22, 0x31)); // requestOutOfRange, not 0x78
    setClock(60);
    module.update(state); // NRC resolves WAITING -> COMPLETE -> IDLE; must not touch engine.rpm

    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm); // untouched, no positive response ever arrived

    int rpmReqsBefore = bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED);
    setClock(120); // 60ms later, well past RPM's 50ms cadence -> should retry promptly, not stay stuck
    module.update(state);
    TEST_ASSERT_TRUE(bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > rpmReqsBefore);
}

void test_multiframe_response_is_dropped_not_misparsed(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    setClock(50);
    module.update(state); // RPM request sent, WAITING

    bus.injectRxFrame(makeMultiFrameFirstFrame());
    setClock(60);
    module.update(state); // must be dropped, not parsed as if data[1] were a SID

    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm);
    TEST_ASSERT_EQUAL_UINT32(0, state.engine.rpmUpdatedMs);
}

// D-059 item 3: segmented reception exists only in the probe env (test_discovery_probe).
// In this build a First Frame, even a valid one answering the request in flight, never
// gets a Flow Control: every transmitted frame stays a Single Frame.
void test_first_frame_never_gets_a_flow_control_outside_the_probe(void) {
    TEST_ASSERT_EQUAL(0, CONN_DISCOVERY_PROBE);
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    for (unsigned long t = 1; t < 3000; t += 5) {
        setClock(t);
        size_t before = bus.txLog.size();
        module.update(state);
        for (size_t i = before; i < bus.txLog.size(); i++) {
            const CanFrame& f = bus.txLog[i];
            if (f.id == VEHICLE_CL250_REQUEST_ID && f.data[1] == UDS_SID_READ_DATA_BY_IDENTIFIER) {
                CanFrame ff = makeMultiFrameFirstFrame();
                ff.data[1] = 0x0A; // FF_DL 10
                ff.data[2] = 0x62; // positive answer to the DID in flight
                ff.data[3] = f.data[2];
                ff.data[4] = f.data[3];
                bus.injectRxFrame(ff);
            }
        }
    }
    TEST_ASSERT_TRUE(bus.txLog.size() > 10);
    for (const CanFrame& f : bus.txLog) {
        TEST_ASSERT_EQUAL_HEX8(0x0, f.data[0] & 0xF0); // Single Frames only, never an FC
    }
}

void test_bus_off_triggers_recovery_with_backoff(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    bus.setState(CanBusState::BUS_OFF);

    setClock(1000);
    module.update(state); // enters bus-off, attempts recovery immediately
    TEST_ASSERT_EQUAL(1, bus.initiateRecoveryCallCount);

    setClock(1100); // only 100ms later -- well under the 1000ms initial backoff
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.initiateRecoveryCallCount); // must not retry yet

    // The first attempt immediately doubled _recoveryBackoffMs to 2000ms (from
    // _lastRecoveryAttempt=1000), so the second attempt isn't due until t=3000.
    setClock(2100);
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.initiateRecoveryCallCount); // still not due

    setClock(3100); // past the (now doubled) 2000ms backoff window
    module.update(state);
    TEST_ASSERT_EQUAL(2, bus.initiateRecoveryCallCount);
}

void test_bus_recovery_complete_restarts_driver(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    bus.setState(CanBusState::BUS_OFF);
    setClock(1000);
    module.update(state);

    bus.setState(CanBusState::STOPPED); // what initiateRecovery() leads to on real hardware
    setClock(1200);
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.startCallCount);
}

void test_did_skipped_after_max_consecutive_timeouts_then_resumes(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    // Never inject any response. Drive exactly 5 send/timeout cycles for RPM (first
    // table entry, generated cadence, 100ms base timeout) by hand, matching the module's
    // real IDLE->WAITING->TIMEOUT->IDLE transitions one call at a time.
    unsigned long t = kRpmDueMs;
    for (int cycle = 0; cycle < 5; cycle++) {
        setClock(t);
        module.update(state); // IDLE -> send RPM request -> WAITING
        t += 151;             // exceed the 100ms base timeout
        setClock(t);
        module.update(state); // WAITING -> TIMEOUT -> IDLE, consecutiveTimeouts++
        t += 1;
    }
    // The 5th timeout should have just triggered a 5s skip for DID VEHICLE_CL250_DID_ENGINE_SPEED.
    const unsigned long skipStart = t - 1;
    int rpmReqsAtSkipStart = bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED);

    // Stay well inside the 5s cooldown and confirm RPM is never re-requested, even
    // though other DID slots keep cycling through their own request/timeout dance.
    for (int i = 0; i < 10; i++) {
        t += 400;
        setClock(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(t < skipStart + VEHICLE_CL250_DID_SKIP_COOLDOWN_MS); // still inside the cooldown
    TEST_ASSERT_EQUAL(rpmReqsAtSkipStart, bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED));

    // Advance well past the cooldown and confirm RPM gets requested again.
    for (int i = 0; i < 15; i++) {
        t += 400;
        setClock(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > rpmReqsAtSkipStart);
}

// ---------------------------------------------------------------------------
// D-023 port: generated table, D-020 guard, fallback addressing
// ---------------------------------------------------------------------------

void test_every_transmitted_frame_passes_the_d020_guard(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    // Run long enough to cover session retries, tester present, every DID and timeouts.
    for (unsigned long t = 0; t < 30000; t += 25) {
        setClock(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(bus.txLog.size() > 100);
    for (const CanFrame& f : bus.txLog) {
        TEST_ASSERT_TRUE(f.id == VEHICLE_CL250_REQUEST_ID || f.id == VEHICLE_CL250_FALLBACK_REQUEST_ID);
        TEST_ASSERT_TRUE(f.extended == (f.id == VEHICLE_CL250_REQUEST_ID)); // 29-bit primary, 11-bit fallback
        TEST_ASSERT_TRUE(vehicle_cl250_frame_allowed(f.data, f.dlc));
    }
    TEST_ASSERT_EQUAL_UINT32(0, module.blockedFrameCount());
}

void test_requests_go_to_primary_and_fallback_ids(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state); // session request on both IDs

    TEST_ASSERT_EQUAL(2, bus.txLog.size());
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_REQUEST_ID, bus.txLog[0].id);
    TEST_ASSERT_TRUE(bus.txLog[0].extended);
    TEST_ASSERT_EQUAL_HEX32(VEHICLE_CL250_FALLBACK_REQUEST_ID, bus.txLog[1].id);
    TEST_ASSERT_FALSE(bus.txLog[1].extended);
    const uint8_t expected[8] = {0x02, 0x10, 0x03, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, bus.txLog[0].data, 8);
}

void test_fallback_response_is_decoded(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);

    setClock(100);
    module.update(state); // RPM request
    CanFrame f = makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x1F, 0x40); // 8000 / 4 = 2000 rpm
    f.id = VEHICLE_CL250_FALLBACK_RESPONSE_ID;
    f.extended = false;
    bus.injectRxFrame(f);
    setClock(110);
    module.update(state);
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, state.engine.rpm);
}

void test_all_dids_decode_with_generated_formulas(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    setClock(100);

    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_VEHICLE_SPEED, 88, 0, 5));     // 88 km/h
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_COOLANT_TEMP, 130, 0, 5));    // 130 - 40 = 90 degC
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_THROTTLE_POS, 255, 0, 5));    // 100 %
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_BATTERY_VOLTAGE, 0x30, 0x70));   // 12400 mV
    module.update(state);

    TEST_ASSERT_EQUAL_UINT8(88, state.engine.speed);
    TEST_ASSERT_EQUAL_INT16(90, state.engine.coolantTemp);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, state.engine.throttlePos);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.4f, state.engine.batteryVoltage);
}

void test_short_battery_response_is_dropped(void) {
    // The legacy A/10 fallback for short VEHICLE_CL250_DID_BATTERY_VOLTAGE responses was not carried over
    // (docs/legacy-telemetry-notes.md in moto-vehicle-defs): a one-byte answer is dropped.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    setClock(100);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_BATTERY_VOLTAGE, 124, 0, 5)); // PCI 4: SID + DID + 1 byte
    module.update(state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.batteryVoltage);
    TEST_ASSERT_EQUAL_UINT32(0, state.engine.batteryVoltageUpdatedMs);
}

void test_frames_from_other_ids_are_ignored(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    setClock(100);
    CanFrame f = makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50);
    f.id = 0x18DAF111; // another ECU
    bus.injectRxFrame(f);
    module.update(state);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm);
}

// ---------------------------------------------------------------------------
// Safety-review additions (D-020 / D-021 / D-023)
// ---------------------------------------------------------------------------

// Every frame this node may ever send, as [PCI + payload] (padding checked separately).
static bool isAllowedEmittedFrame(const CanFrame& f) {
    if (f.dlc != VEHICLE_CL250_FRAME_DLC) return false;
    uint8_t len = f.data[0];
    if (len < 2 || len > 7) return false;
    for (uint8_t i = len + 1; i < 8; i++) {
        if (f.data[i] != VEHICLE_CL250_PADDING_BYTE) return false;
    }
    if (len == 2 && f.data[1] == VEHICLE_CL250_SESSION_SID && f.data[2] == VEHICLE_CL250_SESSION_SUBFUNCTION) return true;
    if (len == 2 && f.data[1] == VEHICLE_CL250_TESTER_PRESENT_SID &&
        f.data[2] == VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION) return true;
    if (len == 3 && f.data[1] == UDS_SID_READ_DATA_BY_IDENTIFIER) {
        uint16_t did = (uint16_t)((f.data[2] << 8) | f.data[3]);
        return vehicle_cl250_find(did) != nullptr;
    }
    return false;
}

static void assertOnlyAllowedFrames(const MockCanBus& bus) {
    for (const CanFrame& f : bus.txLog) {
        TEST_ASSERT_TRUE(f.id == VEHICLE_CL250_REQUEST_ID || f.id == VEHICLE_CL250_FALLBACK_REQUEST_ID);
        TEST_ASSERT_TRUE(isAllowedEmittedFrame(f));
    }
}

void test_emitted_set_is_exact_across_ecu_behaviours(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    for (unsigned long t = 0; t < 20000; t += 10) {
        setClock(t);
        uint8_t phase = (uint8_t)((t / 2000) % 4);
        if (phase == 0 && t % 40 == 0) bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x1A, 0xF8));
        if (phase == 1 && t % 30 == 0) bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31));
        if (phase == 2 && t % 20 == 0) bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
        if (phase == 3 && t == 6500) bus.setState(CanBusState::BUS_OFF);
        if (phase == 3 && t == 6600) bus.setState(CanBusState::STOPPED);
        if (phase == 3 && t == 6700) bus.setState(CanBusState::RUNNING);
        if (t == 1000) bus.injectRxFrame(makeSessionConfirm());
        module.update(state);
    }
    TEST_ASSERT_TRUE(bus.txLog.size() > 100);
    assertOnlyAllowedFrames(bus);
    TEST_ASSERT_EQUAL_UINT32(0, module.blockedFrameCount());
}

void test_forbidden_requests_are_refused_and_never_transmitted(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    bus.enterNormalMode(); // the TX gate is under test, not the listen-only window
    const uint8_t forbidden[][2] = {
        {0x10, 0x02}, {0x10, 0x82}, {0x11, 0x01}, {0x14, 0xFF}, {0x27, 0x01}, {0x2E, 0xF1},
        {0x2F, 0xF1}, {0x31, 0x01}, {0x34, 0x00}, {0x35, 0x00}, {0x36, 0x01}, {0x37, 0x00},
        {0x28, 0x03}, {0x85, 0x02}, {0x3E, 0x01},
    };
    uint32_t expectedBlocked = 0;
    for (const auto& req : forbidden) {
        TEST_ASSERT_FALSE(module.testSendFrame(VEHICLE_CL250_REQUEST_ID, req, 2));
        expectedBlocked++;
    }
    const uint8_t ok[] = {VEHICLE_CL250_SESSION_SID, VEHICLE_CL250_SESSION_SUBFUNCTION};
    for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        // The OBD functional IDs are watch-only (D-040): never sent on.
        TEST_ASSERT_FALSE(module.testSendFrame(vehicle_cl250_functional_watch[i].id, ok, 2));
        expectedBlocked++;
    }
    TEST_ASSERT_FALSE(module.testSendFrame(0x18DA11F1, ok, 2)); // another ECU
    TEST_ASSERT_FALSE(module.testSendFrame(VEHICLE_CL250_REQUEST_ID, ok, 0));
    const uint8_t eight[8] = {0x22, 0xF4, 0x0C, 0, 0, 0, 0, 0};
    TEST_ASSERT_FALSE(module.testSendFrame(VEHICLE_CL250_REQUEST_ID, eight, 8)); // not a Single Frame
    expectedBlocked += 3;
    TEST_ASSERT_EQUAL_UINT32(0, bus.txLog.size());
    TEST_ASSERT_EQUAL_UINT32(expectedBlocked, module.blockedFrameCount());
    TEST_ASSERT_TRUE(module.testSendFrame(VEHICLE_CL250_REQUEST_ID, ok, 2));
    TEST_ASSERT_EQUAL_UINT32(1, bus.txLog.size());
}

void test_response_pending_storm_still_times_out(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.clearTxLog();
    setClock(kRpmDueMs);
    module.update(state); // engine speed request goes out
    TEST_ASSERT_EQUAL_UINT32(2, (uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED)); // primary + fallback
    // The ECU answers 0x78 forever; the request must still end within the generated maximum.
    unsigned long t = kRpmDueMs;
    for (; t < kRpmDueMs + VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS + 400; t += 20) {
        setClock(t);
        bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
        module.update(state);
        if ((uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > 2) break;
    }
    TEST_ASSERT_TRUE((uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > 2);
    TEST_ASSERT_TRUE(t <= kRpmDueMs + VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 100);
}

void test_nrc_for_other_service_does_not_resolve_pending_read(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.clearTxLog();
    setClock(kRpmDueMs);
    module.update(state); // engine speed request pending
    TEST_ASSERT_EQUAL_UINT32(2, (uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED));
    size_t before = bus.txLog.size();
    // NRC for TesterPresent must not end the pending ReadDataByIdentifier early. Without
    // the fix the request completes and the next DID is requested at once.
    setClock(kRpmDueMs + 10);
    bus.injectRxFrame(makeNegativeResponse(VEHICLE_CL250_TESTER_PRESENT_SID, 0x12));
    module.update(state);
    setClock(kRpmDueMs + 20);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(before, bus.txLog.size()); // still waiting, nothing new sent
}

void test_foreign_tester_latches_poller_off(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    setClock(10);
    module.update(state);
    CanFrame foreign;
    foreign.id = VEHICLE_CL250_REQUEST_ID;
    foreign.extended = true;
    foreign.dlc = 8;
    const uint8_t d[8] = {0x03, UDS_SID_READ_DATA_BY_IDENTIFIER, 0xF4, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA};
    for (int i = 0; i < 8; i++) foreign.data[i] = d[i];
    bus.injectRxFrame(foreign);
    setClock(20);
    module.update(state);
    TEST_ASSERT_TRUE(module.foreignTesterDetected());
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL_INT(1, bus.stopCallCount);
    size_t sent = bus.txLog.size();
    for (unsigned long t = 30; t < 10000; t += 25) {
        setClock(t);
        module.update(state);
    }
    TEST_ASSERT_EQUAL_UINT32(sent, bus.txLog.size()); // nothing transmitted after the latch
    TEST_ASSERT_FALSE(state.engine.ecuPresent);
}

void test_repeated_bus_off_latches_off(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 10;
    for (int event = 0; event < 6 && !module.latchedOff(); event++) {
        bus.setState(CanBusState::BUS_OFF);
        setClock(t += 10);
        module.update(state);
        bus.setState(CanBusState::RUNNING);
        setClock(t += 10);
        module.update(state);
    }
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_FALSE(module.foreignTesterDetected());
    TEST_ASSERT_EQUAL_INT(1, bus.stopCallCount);
}

void test_session_confirm_requires_extended_subfunction(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    CanFrame defaultSession = makeSessionConfirm();
    defaultSession.data[2] = 0x01; // 50 01 = default session, not what we asked for
    bus.injectRxFrame(defaultSession);
    setClock(10);
    module.update(state);
    bus.clearTxLog();
    setClock(10 + VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 1);
    module.update(state);
    bool retried = false;
    for (const CanFrame& f : bus.txLog) {
        if (f.data[1] == VEHICLE_CL250_SESSION_SID) retried = true;
    }
    TEST_ASSERT_TRUE(retried); // still unconfirmed -> session request repeated
}

// ---------------------------------------------------------------------------
// D-032: CAN/tester health for BLE telemetry v3 (read-only, never transmits)
// ---------------------------------------------------------------------------

void test_health_reports_bus_state_and_error_counters(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.setErrorCounters(12, 3);
    setClock(10);
    module.update(state);
    TEST_ASSERT_EQUAL((int)CanHealthState::RUNNING, (int)state.can.busState);
    TEST_ASSERT_EQUAL_UINT16(12, state.can.txErrorCount);
    TEST_ASSERT_EQUAL_UINT16(3, state.can.rxErrorCount);
    TEST_ASSERT_EQUAL_HEX8(CAN_HEALTH_FLAG_POLLER_ENABLED, state.can.flags);

    bus.setErrorCounters(CAN_ERROR_WARNING_LIMIT, 0); // running, but at the warning limit
    setClock(20);
    module.update(state);
    TEST_ASSERT_EQUAL((int)CanHealthState::ERROR_WARNING, (int)state.can.busState);

    bus.setState(CanBusState::BUS_OFF);
    bus.setErrorCounters(256, 0);
    setClock(30);
    module.update(state);
    TEST_ASSERT_EQUAL((int)CanHealthState::BUS_OFF, (int)state.can.busState);
    TEST_ASSERT_EQUAL_UINT32(1, state.can.busOffCount);
    TEST_ASSERT_EQUAL_UINT16(256, state.can.txErrorCount);
}

void test_health_counts_unanswered_did_requests(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = kRpmDueMs;
    for (int cycle = 0; cycle < 3; cycle++) {
        setClock(t);
        module.update(state); // send
        t += 151;
        setClock(t);
        module.update(state); // timeout
        t += 1;
    }
    TEST_ASSERT_EQUAL_UINT32(3, module.unansweredDidCount());
    TEST_ASSERT_EQUAL_UINT32(3, state.can.unansweredDidCount);

    // An answered request does not count.
    setClock(t);
    module.update(state);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x1F, 0x40));
    setClock(t + 5);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(3, state.can.unansweredDidCount);
}

void test_health_marks_foreign_tester_latch_and_freezes(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    CanFrame foreign;
    foreign.id = VEHICLE_CL250_REQUEST_ID;
    foreign.extended = true;
    foreign.dlc = 8;
    const uint8_t d[8] = {0x03, UDS_SID_READ_DATA_BY_IDENTIFIER, 0xF4, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA};
    for (int i = 0; i < 8; i++) foreign.data[i] = d[i];
    bus.injectRxFrame(foreign);
    setClock(10);
    module.update(state);
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL((int)CanHealthState::STOPPED, (int)state.can.busState);
    TEST_ASSERT_EQUAL_HEX8(CAN_HEALTH_FLAG_POLLER_ENABLED | CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER, state.can.flags);
    size_t sent = bus.txLog.size();
    bus.setErrorCounters(99, 99); // driver gone: counters are no longer read
    setClock(500);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(0, state.can.txErrorCount);
    TEST_ASSERT_EQUAL_UINT32(sent, bus.txLog.size());
}

void test_health_marks_bus_off_latch(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 10;
    for (int event = 0; event < 6 && !module.latchedOff(); event++) {
        bus.setState(CanBusState::BUS_OFF);
        setClock(t += 10);
        module.update(state);
        bus.setState(CanBusState::RUNNING);
        setClock(t += 10);
        module.update(state);
    }
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL((int)CanHealthState::STOPPED, (int)state.can.busState);
    TEST_ASSERT_EQUAL_HEX8(CAN_HEALTH_FLAG_POLLER_ENABLED | CAN_HEALTH_FLAG_LATCHED_BUS_OFF, state.can.flags);
    TEST_ASSERT_EQUAL_UINT32(kMaxBusOffEvents, state.can.busOffCount); // D-030 latch threshold
}

// ---------------------------------------------------------------------------
// Q-018: tester hardening (latch across resets, listen-only window, RX drain first)
// ---------------------------------------------------------------------------

static CanFrame makeForeignRequest(uint32_t id = VEHICLE_CL250_REQUEST_ID, bool extended = true) {
    CanFrame f;
    f.id = id;
    f.extended = extended;
    f.dlc = 8;
    const uint8_t d[8] = {0x03, UDS_SID_READ_DATA_BY_IDENTIFIER, 0xF4, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA};
    for (int i = 0; i < 8; i++) f.data[i] = d[i];
    return f;
}

static CanFrame makeNoise() {
    CanFrame f;
    f.id = 0x123; // not a diagnostic ID
    f.dlc = 8;
    return f;
}

// Drives every pass of the listen-only window; returns false if anything went out.
static bool runSilently(HondaCANModule& module, MockCanBus& bus, SystemState& state,
                        unsigned long from, unsigned long to) {
    for (unsigned long t = from; t < to; t += 10) {
        test_setMillis(t);
        module.update(state);
    }
    return bus.txLog.empty() && bus.txOutsideNormalMode == 0;
}

void test_latch_record_round_trip_and_corruption(void) {
    TesterLatchRecord r;
    TesterLatchStatus st;
    tester_latch::encode(TesterLatchReason::NONE, 3, r);
    TEST_ASSERT_TRUE(tester_latch::decode(r, st));
    TEST_ASSERT_EQUAL((int)TesterLatchReason::NONE, (int)st.reason); // armed
    TEST_ASSERT_EQUAL_UINT32(3, st.busOffCount);
    tester_latch::encode(TesterLatchReason::FOREIGN_TESTER, 1, r);
    TEST_ASSERT_TRUE(tester_latch::decode(r, st));
    TEST_ASSERT_EQUAL((int)TesterLatchReason::FOREIGN_TESTER, (int)st.reason);
    tester_latch::encode(TesterLatchReason::BUS_OFF, 5, r);
    TEST_ASSERT_TRUE(tester_latch::decode(r, st));
    TEST_ASSERT_EQUAL((int)TesterLatchReason::BUS_OFF, (int)st.reason);
    TEST_ASSERT_EQUAL_UINT32(5, st.busOffCount);

    // Any single flipped bit (RTC memory corruption) fails to decode.
    for (int word = 0; word < 5; word++) {
        for (int bit = 0; bit < 32; bit++) {
            TesterLatchRecord c = r;
            reinterpret_cast<uint32_t*>(&c)[word] ^= (1u << bit);
            TEST_ASSERT_FALSE(tester_latch::decode(c, st));
        }
    }
    TesterLatchRecord zero = {0, 0, 0, 0, 0};
    TEST_ASSERT_FALSE(tester_latch::decode(zero, st));
    TesterLatchRecord ones = {0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu};
    TEST_ASSERT_FALSE(tester_latch::decode(ones, st));

    // A well-formed record with an unknown state does not decode either.
    TesterLatchRecord unknown;
    unknown.magic = tester_latch::kMagic;
    unknown.state = 7;
    unknown.stateInverted = ~7u;
    unknown.busOffCount = 0;
    unknown.crc = tester_latch::recordCrc(unknown);
    TEST_ASSERT_FALSE(tester_latch::decode(unknown, st));
}

void test_restore_rule_is_fail_safe(void) {
    TesterLatchRecord r = {0x12345678u, 0, 0, 0, 0}; // garbage, as after power-on
    // Power-on: re-armed, and the record now holds a valid ARMED state.
    TesterLatchStatus st = tester_latch::restore(r, true);
    TEST_ASSERT_EQUAL((int)TesterLatchReason::NONE, (int)st.reason);
    TesterLatchStatus again = tester_latch::restore(r, false);
    TEST_ASSERT_EQUAL((int)TesterLatchReason::NONE, (int)again.reason);
    // Latch, then a non-power-on reset: still latched.
    tester_latch::encode(TesterLatchReason::FOREIGN_TESTER, 0, r);
    TEST_ASSERT_EQUAL((int)TesterLatchReason::FOREIGN_TESTER, (int)tester_latch::restore(r, false).reason);
    // Invalid record after a non-power-on reset: latched, cause unknown.
    r.crc ^= 1u;
    TEST_ASSERT_EQUAL((int)TesterLatchReason::UNKNOWN, (int)tester_latch::restore(r, false).reason);
    tester_latch::clear(r);
    TEST_ASSERT_EQUAL((int)TesterLatchReason::UNKNOWN, (int)tester_latch::restore(r, false).reason);
    // Power-on clears any latch.
    tester_latch::encode(TesterLatchReason::BUS_OFF, 5, r);
    TEST_ASSERT_EQUAL((int)TesterLatchReason::NONE, (int)tester_latch::restore(r, true).reason);
}

void test_nothing_is_sent_during_the_listen_only_window(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(1000);
    TEST_ASSERT_TRUE(module.begin());
    TEST_ASSERT_TRUE(module.listening());
    TEST_ASSERT_EQUAL((int)MockCanBus::Mode::LISTEN_ONLY, (int)bus.mode);

    TEST_ASSERT_TRUE(runSilently(module, bus, state, 1000, 1000 + kListenOnlyWindowMs));
    TEST_ASSERT_TRUE(module.listening());
    TEST_ASSERT_EQUAL_INT(0, bus.enterNormalModeCallCount);
    TEST_ASSERT_FALSE(state.engine.ecuPresent);

    // Window over: normal mode, but the session request waits for the next pass's drain.
    test_setMillis(1000 + kListenOnlyWindowMs);
    module.update(state);
    TEST_ASSERT_FALSE(module.listening());
    TEST_ASSERT_EQUAL((int)MockCanBus::Mode::NORMAL, (int)bus.mode);
    TEST_ASSERT_EQUAL_UINT32(0, bus.txLog.size());

    test_setMillis(1000 + kListenOnlyWindowMs + 10);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(2, bus.txLog.size()); // session request, primary + fallback
    TEST_ASSERT_EQUAL_HEX8(VEHICLE_CL250_SESSION_SID, bus.txLog[0].data[1]);
    TEST_ASSERT_EQUAL_INT(0, bus.txOutsideNormalMode);
}

void test_listen_window_starts_at_first_update_not_begin(void) {
    // setup() runs the other modules' begin() between our begin() and first update().
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    TEST_ASSERT_TRUE(runSilently(module, bus, state, 5000, 5000 + kListenOnlyWindowMs));
    TEST_ASSERT_TRUE(module.listening());
    test_setMillis(5000 + kListenOnlyWindowMs);
    module.update(state);
    TEST_ASSERT_FALSE(module.listening());
}

void test_lost_frames_restart_the_listen_window(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    TEST_ASSERT_TRUE(runSilently(module, bus, state, 0, 1500));
    bus.rxLost = 3; // queue overflowed: a foreign request may have been among them
    TEST_ASSERT_TRUE(runSilently(module, bus, state, 1500, 1500 + kListenOnlyWindowMs));
    TEST_ASSERT_TRUE(module.listening()); // the original end (t=2000) did not count
    test_setMillis(1500 + kListenOnlyWindowMs);
    module.update(state);
    TEST_ASSERT_FALSE(module.listening());
}

void test_rx_backlog_restarts_the_listen_window(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    TEST_ASSERT_TRUE(runSilently(module, bus, state, 0, kListenOnlyWindowMs));
    for (int i = 0; i < 100; i++) bus.injectRxFrame(makeNoise());
    test_setMillis(kListenOnlyWindowMs); // would end the window, but the queue is not empty
    module.update(state);
    TEST_ASSERT_TRUE(module.listening());
    TEST_ASSERT_EQUAL_UINT32(0, bus.txLog.size());
}

void test_listen_window_survives_millis_wrap(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    const unsigned long start = (unsigned long)0 - 500; // wraps 500 ms into the window
    test_setMillis(start);
    module.begin();
    module.update(state); // window starts
    test_setMillis(start + kListenOnlyWindowMs - 1);
    module.update(state);
    TEST_ASSERT_TRUE(module.listening());
    test_setMillis(start + kListenOnlyWindowMs);
    module.update(state);
    TEST_ASSERT_FALSE(module.listening());
}

void test_foreign_tester_during_listen_window_latches_before_any_tx(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    test_setMillis(500);
    module.update(state);
    bus.injectRxFrame(makeForeignRequest());
    test_setMillis(510);
    module.update(state);

    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_TRUE(module.foreignTesterDetected());
    TEST_ASSERT_EQUAL_INT(1, bus.stopCallCount);
    TEST_ASSERT_EQUAL((int)TesterLatchReason::FOREIGN_TESTER, (int)latchStore.peek().reason);
    TEST_ASSERT_TRUE(runSilently(module, bus, state, 520, 20000));
    TEST_ASSERT_EQUAL_INT(0, bus.enterNormalModeCallCount); // never left listen-only
    TEST_ASSERT_EQUAL_HEX8(CAN_HEALTH_FLAG_POLLER_ENABLED | CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER, state.can.flags);
}

void test_unsolicited_ecu_answer_during_listen_window_latches(void) {
    // We asked nothing yet, so an ECU answer means another tester asked (its request
    // may have been lost or sent before the window began).
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    module.update(state);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x1F, 0x40));
    test_setMillis(100);
    module.update(state);
    TEST_ASSERT_TRUE(module.foreignTesterDetected());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm); // someone else's data is not stored
    TEST_ASSERT_TRUE(runSilently(module, bus, state, 110, 10000));
}

static bool latchesOn(const CanFrame& f) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.injectRxFrame(f);
    setClock(10);
    module.update(state);
    return module.foreignTesterDetected();
}

// Known-answer check of the gen/ watch table (safety review of C-1, MINOR-1): the
// literals below are ISO 15765-4 OBD functional request IDs used as golden vectors, not
// signal definitions. If a defs release drops or mis-flags an entry, this fails instead
// of the latch silently getting weaker.
void test_obd_functional_request_ids_known_answer(void) {
    for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        const vehicle_cl250_watch_id_t& w = vehicle_cl250_functional_watch[i];
        TEST_ASSERT_TRUE(w.extended == isotp::isExtendedId(w.id));
    }
    TEST_ASSERT_TRUE(latchesOn(makeForeignRequest(0x7DFu, false)));       // 11-bit functional
    TEST_ASSERT_TRUE(latchesOn(makeForeignRequest(0x18DB33F1u, true)));   // 29-bit, SA 0xF1
    TEST_ASSERT_TRUE(latchesOn(makeForeignRequest(0x18DB33F2u, true)));   // 29-bit, any SA
    // Not a functional request to the OBD target address 0x33.
    TEST_ASSERT_FALSE(latchesOn(makeForeignRequest(0x7DFu, true)));       // 0x7DF as a 29-bit ID
    TEST_ASSERT_FALSE(latchesOn(makeForeignRequest(0x7DEu, false)));
    TEST_ASSERT_FALSE(latchesOn(makeForeignRequest(0x18DB34F1u, true)));  // other target address
    TEST_ASSERT_FALSE(latchesOn(makeForeignRequest(0x18DA33F1u, true)));  // physical, not our ECU
}

void test_any_diagnostic_request_id_counts_as_foreign_tester(void) {
    const uint32_t otherSource = (VEHICLE_CL250_REQUEST_ID & ~isotp::kNormalFixedSourceAddressMask) | 0xF2u;
    std::vector<CanFrame> frames = {
        makeForeignRequest(otherSource, true),                                   // same ECU, other tester address
        makeForeignRequest(VEHICLE_CL250_FALLBACK_REQUEST_ID, false),
    };
    // Q-021/D-040: every OBD functional watch ID from gen/; a 29-bit one also from
    // another source address (0x18DB<TA><SA>).
    for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        const vehicle_cl250_watch_id_t& w = vehicle_cl250_functional_watch[i];
        frames.push_back(makeForeignRequest(w.id, w.extended));
        if (w.extended) {
            frames.push_back(makeForeignRequest((w.id & ~isotp::kNormalFixedSourceAddressMask) | 0xF2u, true));
        }
    }
    TEST_ASSERT_EQUAL_UINT32(2 + VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT + 1, frames.size()); // one 29-bit entry
    for (const CanFrame& f : frames) {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        startPolling(module, state);
        bus.injectRxFrame(f);
        setClock(10);
        module.update(state);
        TEST_ASSERT_TRUE(module.foreignTesterDetected());
    }
    // The ECU's own answers and other traffic are not requests.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.injectRxFrame(makeNoise());
    bus.injectRxFrame(makeForeignRequest(VEHICLE_CL250_RESPONSE_ID, true));
    setClock(10);
    module.update(state);
    TEST_ASSERT_FALSE(module.latchedOff());
}

void test_foreign_frame_right_after_normal_mode_blocks_the_session_request(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    module.update(state);
    test_setMillis(kListenOnlyWindowMs);
    module.update(state); // normal mode, session pending
    TEST_ASSERT_FALSE(module.listening());
    bus.injectRxFrame(makeForeignRequest()); // arrived while the driver was reinstalled
    test_setMillis(kListenOnlyWindowMs + 10);
    module.update(state);
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL_UINT32(0, bus.txLog.size());
}

void test_foreign_tester_latch_survives_reset(void) {
    MockTesterLatchStore latchStore; // the RTC no-init record, shared across the "reset"
    {
        MockCanBus bus;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        startPolling(module, state);
        bus.injectRxFrame(makeForeignRequest());
        setClock(10);
        module.update(state);
        TEST_ASSERT_TRUE(module.latchedOff());
    }
    // Watchdog/panic/brownout reset: a fresh module on a fresh driver, same record.
    MockCanBus bus;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    TEST_ASSERT_TRUE(module.begin()); // healthy, so CAN health keeps reporting the latch
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_TRUE(module.foreignTesterDetected());
    TEST_ASSERT_EQUAL_INT(0, bus.beginListenOnlyCallCount); // driver never installed
    TEST_ASSERT_EQUAL_INT(1, bus.stopCallCount);             // TX held recessive
    TEST_ASSERT_TRUE(runSilently(module, bus, state, 0, 20000));
    TEST_ASSERT_EQUAL_INT(0, bus.enterNormalModeCallCount);
    TEST_ASSERT_FALSE(state.engine.ecuPresent);
    TEST_ASSERT_EQUAL((int)CanHealthState::STOPPED, (int)state.can.busState);
    TEST_ASSERT_EQUAL_HEX8(CAN_HEALTH_FLAG_POLLER_ENABLED | CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER, state.can.flags);
}

static void causeBusOffs(HondaCANModule& module, MockCanBus& bus, SystemState& state, int events) {
    unsigned long t = 10;
    for (int event = 0; event < events && !module.latchedOff(); event++) {
        bus.setState(CanBusState::BUS_OFF);
        setClock(t += 10);
        module.update(state);
        bus.setState(CanBusState::RUNNING);
        setClock(t += 10);
        module.update(state);
    }
}

void test_bus_off_latch_survives_reset(void) {
    MockTesterLatchStore latchStore;
    {
        MockCanBus bus;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        startPolling(module, state);
        causeBusOffs(module, bus, state, 6);
        TEST_ASSERT_TRUE(module.latchedOff());
    }
    MockCanBus bus;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    test_setMillis(10);
    module.update(state);
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_FALSE(module.foreignTesterDetected());
    TEST_ASSERT_EQUAL_INT(0, bus.beginListenOnlyCallCount);
    TEST_ASSERT_EQUAL_HEX8(CAN_HEALTH_FLAG_POLLER_ENABLED | CAN_HEALTH_FLAG_LATCHED_BUS_OFF, state.can.flags);
    TEST_ASSERT_EQUAL_UINT32(kMaxBusOffEvents, state.can.busOffCount);
}

void test_bus_off_budget_survives_reset(void) {
    MockTesterLatchStore latchStore;
    {
        MockCanBus bus;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        startPolling(module, state);
        causeBusOffs(module, bus, state, (int)kMaxBusOffEvents - 2);
        TEST_ASSERT_FALSE(module.latchedOff());
    }
    // A reset (e.g. watchdog) does not refill the budget: two more bus-offs latch.
    MockCanBus bus;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    causeBusOffs(module, bus, state, 1);
    TEST_ASSERT_FALSE(module.latchedOff());
    causeBusOffs(module, bus, state, 1);
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL((int)TesterLatchReason::BUS_OFF, (int)module.latchReason());
}

void test_spent_bus_off_budget_in_an_armed_record_latches(void) {
    MockTesterLatchStore latchStore;
    latchStore.save(TesterLatchReason::NONE, kMaxBusOffEvents);
    latchStore.powerOnAtNextLoad = false;
    MockCanBus bus;
    HondaCANModule module(bus, latchStore);
    test_setMillis(0);
    module.begin();
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL((int)TesterLatchReason::BUS_OFF, (int)module.latchReason());
    TEST_ASSERT_EQUAL_INT(0, bus.beginListenOnlyCallCount);
}

void test_invalid_record_after_reset_latches_and_power_on_rearms(void) {
    MockTesterLatchStore latchStore;
    latchStore.save(TesterLatchReason::NONE, 0);
    latchStore.record.crc ^= 0x1u;       // corrupted in RTC memory
    latchStore.powerOnAtNextLoad = false; // e.g. a watchdog reset
    {
        MockCanBus bus;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        test_setMillis(0);
        TEST_ASSERT_TRUE(module.begin());
        TEST_ASSERT_TRUE(module.latchedOff());
        TEST_ASSERT_EQUAL((int)TesterLatchReason::UNKNOWN, (int)module.latchReason());
        TEST_ASSERT_EQUAL_INT(0, bus.beginListenOnlyCallCount);
        TEST_ASSERT_TRUE(runSilently(module, bus, state, 0, 5000));
        TEST_ASSERT_EQUAL_HEX8(CAN_HEALTH_FLAG_POLLER_ENABLED | CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER |
                               CAN_HEALTH_FLAG_LATCHED_BUS_OFF, state.can.flags); // cause unknown
    }
    latchStore.powerOn(); // power-cycle: the deliberate way to re-enable the tester
    MockCanBus bus;
    HondaCANModule module(bus, latchStore);
    test_setMillis(0);
    module.begin();
    TEST_ASSERT_FALSE(module.latchedOff());
    TEST_ASSERT_TRUE(module.listening()); // still has to pass the window first
}

void test_latch_is_saved_before_the_driver_stops(void) {
    // The store must hold the latch even if a reset follows stop() immediately.
    struct OrderCheckingBus : public MockCanBus {
        MockTesterLatchStore* store = nullptr;
        TesterLatchReason seenAtStop = TesterLatchReason::NONE;
        void stop() override {
            seenAtStop = store->peek().reason;
            MockCanBus::stop();
        }
    };
    MockTesterLatchStore latchStore;
    OrderCheckingBus bus;
    bus.store = &latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.injectRxFrame(makeForeignRequest());
    setClock(10);
    module.update(state);
    TEST_ASSERT_EQUAL((int)TesterLatchReason::FOREIGN_TESTER, (int)bus.seenAtStop);
}

void test_normal_mode_failure_disables_the_poller(void) {
    MockCanBus bus;
    bus.failEnterNormalMode = true;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(0);
    module.begin();
    module.update(state);
    test_setMillis(kListenOnlyWindowMs);
    module.update(state);
    TEST_ASSERT_FALSE(module.isHealthy()); // main.cpp stops calling update()
    TEST_ASSERT_EQUAL_INT(1, bus.stopCallCount);
    TEST_ASSERT_EQUAL_UINT32(0, bus.txLog.size());
    TEST_ASSERT_EQUAL_INT(0, bus.txOutsideNormalMode);
}

void test_driver_install_failure_fails_begin(void) {
    MockCanBus bus;
    bus.failBeginListenOnly = true;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    TEST_ASSERT_FALSE(module.begin());
    TEST_ASSERT_FALSE(module.isHealthy());
    TEST_ASSERT_EQUAL_INT(1, bus.stopCallCount); // TX held recessive
}

void test_queued_response_is_not_counted_as_timeout_after_slow_pass(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    setClock(100);
    module.update(state); // engine speed request, WAITING
    // The answer arrives in time, but the next pass runs long after the timeout.
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x1F, 0x40));
    setClock(100 + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 500);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(0, module.unansweredDidCount());
    TEST_ASSERT_EQUAL_UINT32(0, state.can.unansweredDidCount);
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, state.engine.rpm);
}

void test_foreign_tester_in_queue_stops_all_tx_in_that_pass(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.clearTxLog();
    bus.injectRxFrame(makeForeignRequest());
    // Tester present, the session retry and every DID are all due in this pass.
    setClock(VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS + VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 1000);
    module.update(state);
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL_UINT32(0, bus.txLog.size());
}

void test_rx_drain_is_bounded_and_backlog_pass_sends_nothing(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.clearTxLog();
    for (int i = 0; i < 200; i++) bus.injectRxFrame(makeNoise());
    // Tester present, the session retry and every DID are all due in this pass.
    setClock(VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS + VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 1000);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(200 - kMaxRxFramesPerPass, bus.rxPending());
    TEST_ASSERT_EQUAL_UINT32(0, bus.txLog.size());
}

// ---------------------------------------------------------------------------
// D-058: tester statistics (step gap, per-DID round trip), exposed in state.tester
// ---------------------------------------------------------------------------

static const uint8_t kRpmIdx = VEHICLE_CL250_IDX_ENGINE_SPEED;

// The DID read (SID 0x22) among the frames sent since txLog index `from`, if any (each
// request goes out on two IDs with the same payload).
static bool newDidRequest(const MockCanBus& bus, size_t from, uint16_t& did) {
    for (size_t i = from; i < bus.txLog.size(); i++) {
        const CanFrame& f = bus.txLog[i];
        if (f.dlc >= 4 && f.data[1] == UDS_SID_READ_DATA_BY_IDENTIFIER) {
            did = (uint16_t)((f.data[2] << 8) | f.data[3]);
            return true;
        }
    }
    return false;
}

static int didIndexOf(uint16_t did) {
    for (int i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (vehicle_cl250_dids[i].did == did) return i;
    }
    TEST_FAIL_MESSAGE("unknown DID");
    return -1;
}

// Steps the module one millisecond at a time (relative clock `t`) until `did` has been
// requested `requests` times (each request goes out on two IDs). Returns the pass time of
// the send, which is the module's first-send stamp of that request. Reads of other DIDs are
// answered on the next pass, so none of them times out (a timeout suppresses round-trip
// samples for a while, see test_round_trip_none_while_a_late_answer_can_still_arrive).
static unsigned long runUntilRequested(HondaCANModule& module, MockCanBus& bus, SystemState& state,
                                       uint16_t did, int requests, unsigned long& t) {
    for (unsigned long guard = 0; guard < 5000; guard++) {
        const size_t before = bus.txLog.size();
        t++;
        setClock(t);
        module.update(state);
        if (bus.countDidRequests(did) >= 2 * requests) {
            return t;
        }
        uint16_t other = 0;
        if (newDidRequest(bus, before, other)) {
            bus.injectRxFrame(makePositiveResponse(other, 0x46, 0x50));
        }
    }
    TEST_FAIL_MESSAGE("DID was never requested");
    return 0;
}

void test_step_gap_max_and_over_count_round_up_to_ms(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state); // micros() stays 0 so far: gaps of 0
    TEST_ASSERT_TRUE(state.tester.available);
    TEST_ASSERT_EQUAL_UINT16(0, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(0, state.tester.stepGapOverCount);

    unsigned long us = 0;
    const unsigned long limitMs = VEHICLE_CL250_CLIENT_STEP_MAX_MS;
    // Exactly the limit: 10.000 ms is not above it.
    us += limitMs * 1000;
    test_setMicros(us);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(limitMs, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(0, state.tester.stepGapOverCount);
    // One microsecond more rounds up to 11 ms: over the limit.
    us += limitMs * 1000 + 1;
    test_setMicros(us);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(limitMs + 1, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(1, state.tester.stepGapOverCount);
    // 1 us rounds up to 1 ms (max unchanged); a zero gap is 0 ms.
    us += 1;
    test_setMicros(us);
    module.update(state);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(limitMs + 1, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(1, state.tester.stepGapOverCount);
    // A longer gap raises the maximum and counts again.
    us += 25000;
    test_setMicros(us);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(25, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(2, state.tester.stepGapOverCount);
    // The accessor and the published copy agree.
    TEST_ASSERT_EQUAL_UINT16(25, module.testerStats().stepGapMaxMs);
    // A jump across most of the 32-bit range saturates; then 32 us across the micros() wrap at
    // 2^32 is a 1 ms gap, not a huge one (the subtraction is wrap-safe).
    test_setMicros(0xFFFFFFF0ul);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(65535, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(3, state.tester.stepGapOverCount);
    test_setMicros(0x10ul);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(3, state.tester.stepGapOverCount);
    test_setMicros(0);
}

void test_step_gap_counts_every_entry_also_in_the_listen_window_and_when_latched(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    test_setMillis(5000);
    TEST_ASSERT_TRUE(module.begin());
    test_setMicros(100000);
    module.update(state); // listen window starts; the first entry only sets the reference
    TEST_ASSERT_TRUE(module.listening());
    test_setMicros(115000);
    module.update(state); // 15 ms gap while only listening
    TEST_ASSERT_EQUAL_UINT16(15, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(1, state.tester.stepGapOverCount);

    // A foreign request latches the poller off; the step still runs and is still counted.
    CanFrame foreign;
    foreign.id = VEHICLE_CL250_REQUEST_ID;
    foreign.extended = true;
    foreign.dlc = 8;
    foreign.data[0] = 0x02;
    foreign.data[1] = 0x3E;
    bus.injectRxFrame(foreign);
    test_setMicros(145000);
    module.update(state);
    TEST_ASSERT_TRUE(module.latchedOff());
    TEST_ASSERT_EQUAL_UINT16(30, state.tester.stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(2, state.tester.stepGapOverCount);
    test_setMicros(157000);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(3, state.tester.stepGapOverCount);
    TEST_ASSERT_EQUAL_UINT16(30, state.tester.stepGapMaxMs);
    TEST_ASSERT_TRUE(state.tester.available);
    test_setMicros(0);
}

void test_step_gap_saturates_at_65535(void) {
    TesterStatsTracker t;
    t.onStepEntry(0);
    t.onStepEntry(100u * 1000u * 1000u); // 100 s: 100000 ms saturates the maximum
    TEST_ASSERT_EQUAL_UINT16(65535, t.stats().stepGapMaxMs);
    TEST_ASSERT_EQUAL_UINT16(1, t.stats().stepGapOverCount);
    uint32_t us = 100u * 1000u * 1000u;
    for (int i = 0; i < 70000; i++) {
        us += 11000; // 11 ms: over the limit
        t.onStepEntry(us);
    }
    TEST_ASSERT_EQUAL_UINT16(65535, t.stats().stepGapOverCount);
}

void test_round_trip_sample_on_positive_response(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    unsigned long sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[kRpmIdx].count);
    TEST_ASSERT_EQUAL_UINT16(TESTER_RTT_NONE_MS, state.tester.rtt[kRpmIdx].minMs);
    // Answer 17 ms after the send; the sample is measured at the step that drains it.
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    setClock(sent + 17);
    module.update(state);
    const DidRoundTripStats& r = state.tester.rtt[kRpmIdx];
    TEST_ASSERT_EQUAL_UINT32(1, r.count);
    TEST_ASSERT_EQUAL_UINT16(17, r.minMs);
    TEST_ASSERT_EQUAL_UINT16(17, r.maxMs);
    TEST_ASSERT_EQUAL_UINT32(17, r.sumMs);
    TEST_ASSERT_EQUAL_UINT16(0, r.nrc78Count);
    // Other DIDs are untouched.
    TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[VEHICLE_CL250_IDX_VEHICLE_SPEED].count);

    // Second sample, 40 ms: min/max/sum/count follow.
    t = sent + 17;
    sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 2, t);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    setClock(sent + 40);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(2, r.count);
    TEST_ASSERT_EQUAL_UINT16(17, r.minMs);
    TEST_ASSERT_EQUAL_UINT16(40, r.maxMs);
    TEST_ASSERT_EQUAL_UINT32(57, r.sumMs);
}

void test_round_trip_none_on_timeout(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    unsigned long sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    setClock(sent + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 5);
    module.update(state); // no answer: timeout
    TEST_ASSERT_EQUAL_UINT32(1, state.can.unansweredDidCount);
    TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[kRpmIdx].count);
    TEST_ASSERT_EQUAL_UINT16(TESTER_RTT_NONE_MS, state.tester.rtt[kRpmIdx].minMs);
    TEST_ASSERT_EQUAL_UINT16(0, state.tester.rtt[kRpmIdx].maxMs);
    TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[kRpmIdx].sumMs);
}

void test_round_trip_nrc78_counted_once_per_request_and_gives_no_sample(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    unsigned long sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    // Two 0x78 for the same request, then the positive answer.
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
    setClock(sent + 20);
    module.update(state);
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
    setClock(sent + 50);
    module.update(state);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    setClock(sent + 90);
    module.update(state);
    DidRoundTripStats r = state.tester.rtt[kRpmIdx];
    TEST_ASSERT_EQUAL_UINT16(1, r.nrc78Count);
    TEST_ASSERT_EQUAL_UINT32(0, r.count); // answered after 0x78: no sample
    TEST_ASSERT_EQUAL_UINT16(TESTER_RTT_NONE_MS, r.minMs);
    TEST_ASSERT_EQUAL_FLOAT(4500.0f, state.engine.rpm); // the value itself was still used

    // The next request of the same DID is a fresh one: a clean answer is sampled again.
    t = sent + 90;
    sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 2, t);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    setClock(sent + 25);
    module.update(state);
    r = state.tester.rtt[kRpmIdx];
    TEST_ASSERT_EQUAL_UINT32(1, r.count);
    TEST_ASSERT_EQUAL_UINT16(25, r.minMs);
    TEST_ASSERT_EQUAL_UINT16(1, r.nrc78Count);

    // A third request with 0x78 counts again, and a 0x78 for another service does not.
    t = sent + 25;
    sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 3, t);
    bus.injectRxFrame(makeNegativeResponse(VEHICLE_CL250_TESTER_PRESENT_SID, UDS_NRC_RESPONSE_PENDING));
    setClock(sent + 5);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(1, state.tester.rtt[kRpmIdx].nrc78Count);
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
    setClock(sent + 10);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(2, state.tester.rtt[kRpmIdx].nrc78Count);
}

void test_round_trip_sample_for_nrc_other_than_0x78(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    unsigned long sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    // An NRC for another service does not resolve the request: no sample.
    bus.injectRxFrame(makeNegativeResponse(VEHICLE_CL250_TESTER_PRESENT_SID, 0x12));
    setClock(sent + 5);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[kRpmIdx].count);
    // requestOutOfRange for the DID read resolves it and is a round-trip sample.
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31));
    setClock(sent + 12);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(1, state.tester.rtt[kRpmIdx].count);
    TEST_ASSERT_EQUAL_UINT16(12, state.tester.rtt[kRpmIdx].minMs);
    TEST_ASSERT_EQUAL_UINT16(0, state.tester.rtt[kRpmIdx].nrc78Count);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm); // no value from an NRC
}

void test_round_trip_counters_saturate(void) {
    TesterStatsTracker t;
    // Samples above 16 bits clamp the 16-bit fields; 65535 stays reserved for "no sample".
    t.recordRoundTrip(0, 0xFFFFFFF0u);
    TEST_ASSERT_EQUAL_UINT16(65534, t.stats().rtt[0].minMs);
    TEST_ASSERT_EQUAL_UINT16(65535, t.stats().rtt[0].maxMs);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFF0u, t.stats().rtt[0].sumMs);
    // The 32-bit sum saturates instead of wrapping.
    t.recordRoundTrip(0, 100);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, t.stats().rtt[0].sumMs);
    TEST_ASSERT_EQUAL_UINT32(2, t.stats().rtt[0].count);
    TEST_ASSERT_EQUAL_UINT16(100, t.stats().rtt[0].minMs);
    // The 32-bit count saturates.
    DidRoundTripStats almost;
    almost.count = 0xFFFFFFFFu;
    almost.minMs = 5;
    almost.maxMs = 9;
    almost.sumMs = 50;
    t.testSetRoundTrip(1, almost);
    t.recordRoundTrip(1, 7);
    TEST_ASSERT_EQUAL_UINT32(0xFFFFFFFFu, t.stats().rtt[1].count);
    TEST_ASSERT_EQUAL_UINT32(57, t.stats().rtt[1].sumMs);
    // The 16-bit 0x78 count saturates.
    for (int i = 0; i < 70000; i++) {
        t.recordNrc78(2);
    }
    TEST_ASSERT_EQUAL_UINT16(65535, t.stats().rtt[2].nrc78Count);
    // An index outside the DID table is ignored.
    t.recordRoundTrip(VEHICLE_CL250_DID_COUNT, 5);
    t.recordNrc78(VEHICLE_CL250_DID_COUNT);
}

// Steps one millisecond at a time up to relative time `until`, answering every DID read
// (positively, drained on the next pass) except `silentDid`, so no other request times out.
static void runAnswering(HondaCANModule& module, MockCanBus& bus, SystemState& state, unsigned long& t,
                         unsigned long until, int silentDid = -1) {
    while (t < until) {
        const size_t before = bus.txLog.size();
        t++;
        setClock(t);
        module.update(state);
        uint16_t did = 0;
        if (newDidRequest(bus, before, did) && (int)did != silentDid) {
            bus.injectRxFrame(makePositiveResponse(did, 0x46, 0x50));
        }
    }
}

void test_round_trip_none_while_a_late_answer_can_still_arrive(void) {
    // m1: the answer to a timed-out request can arrive late and resolve a later request
    // early. Requests sent within RESPONSE_TIMEOUT_MAX_MS after a timeout give no sample.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    unsigned long sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    t = sent + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 5;
    setClock(t);
    module.update(state); // no answer: the first engine-speed request times out
    TEST_ASSERT_EQUAL_UINT32(1, state.can.unansweredDidCount);
    const unsigned long timeoutAt = t;

    // Other DIDs are answered promptly meanwhile, but they too were sent in the window.
    runAnswering(module, bus, state, t, t + 1, VEHICLE_CL250_DID_ENGINE_SPEED);
    while (bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) < 4) {
        runAnswering(module, bus, state, t, t + 1, VEHICLE_CL250_DID_ENGINE_SPEED);
        TEST_ASSERT_TRUE(t < timeoutAt + 1000);
    }
    TEST_ASSERT_EQUAL_UINT32(1, state.can.unansweredDidCount);
    for (int i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[i].count);
    }
    // The late answer of the first request arrives 2 ms after the second one went out and
    // resolves it: its value is used, but it is no round-trip sample.
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    t += 2;
    setClock(t);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[kRpmIdx].count);
    TEST_ASSERT_EQUAL_FLOAT(4500.0f, state.engine.rpm);

    // A late NRC carries no DID: whichever read is pending next would be resolved by it.
    size_t before = bus.txLog.size();
    uint16_t nextDid = 0;
    while (!newDidRequest(bus, before, nextDid)) {
        t++;
        setClock(t);
        module.update(state);
        TEST_ASSERT_TRUE(t < timeoutAt + 1500);
    }
    // A late 0x78 is not counted against that DID either.
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT16(0, state.tester.rtt[didIndexOf(nextDid)].nrc78Count);
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT32(0, state.tester.rtt[didIndexOf(nextDid)].count);
    TEST_ASSERT_EQUAL_UINT32(1, state.can.unansweredDidCount);
    // D-066 item 4: nor is it shown as that DID's negative answer on the vehicle-signal block.
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[didIndexOf(nextDid)].lastWasNrc);

    // Requests sent after the window are sampled again (answered on the next 1 ms pass).
    runAnswering(module, bus, state, t, timeoutAt + VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS + 500);
    TEST_ASSERT_EQUAL_UINT32(1, state.can.unansweredDidCount); // no other timeout extended it
    const DidRoundTripStats& r = state.tester.rtt[kRpmIdx];
    TEST_ASSERT_TRUE(r.count >= 1);
    TEST_ASSERT_EQUAL_UINT16(1, r.minMs);
    TEST_ASSERT_EQUAL_UINT16(1, r.maxMs);
    TEST_ASSERT_EQUAL_UINT32(r.count, r.sumMs);
}

// ---------------------------------------------------------------------------
// D-066 item 4: the tester's per-DID bookkeeping for the BLE vehicle-signal block
// ---------------------------------------------------------------------------

void test_vehicle_signals_keep_raw_bytes_of_the_validated_answer(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_FALSE(state.vehicleSignals.dids[i].hasValue);
    }
    unsigned long t = 0;
    runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    t += 1;
    setClock(t);
    module.update(state);
    const VehicleSignalRecord& rpm = state.vehicleSignals.dids[kRpmIdx];
    TEST_ASSERT_TRUE(rpm.hasValue);
    TEST_ASSERT_EQUAL_HEX32(0x4650, rpm.raw); // the 2 data bytes, big-endian, unscaled
    TEST_ASSERT_EQUAL_UINT32(gT0 + t, rpm.updatedMs);
    TEST_ASSERT_FALSE(rpm.lastWasNrc);

    // A one-byte DID keeps one byte, the padding after it is not part of raw.
    const uint8_t speedIdx = VEHICLE_CL250_IDX_VEHICLE_SPEED;
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_VEHICLE_SPEED, 0x2A, 0xAA, 5));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[speedIdx].hasValue);
    TEST_ASSERT_EQUAL_HEX32(0x2A, state.vehicleSignals.dids[speedIdx].raw);

    // An answer the generated parser refuses (out of range / short) is not recorded.
    const uint8_t battIdx = VEHICLE_CL250_IDX_BATTERY_VOLTAGE;
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_BATTERY_VOLTAGE, 0x30, 0x00, 5));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[battIdx].hasValue);
}

void test_vehicle_signals_nrc_of_the_pending_read_then_answer_clears_it(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    // 0x78 is no answer: nothing recorded.
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[kRpmIdx].lastWasNrc);
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[kRpmIdx].lastWasNrc);
    TEST_ASSERT_EQUAL_HEX8(0x31, state.vehicleSignals.dids[kRpmIdx].nrc);
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[kRpmIdx].hasValue);
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (i != kRpmIdx) TEST_ASSERT_FALSE(state.vehicleSignals.dids[i].lastWasNrc);
    }
    // An NRC for another service (session control) is never attributed to a DID.
    runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 2, t);
    bus.injectRxFrame(makeNegativeResponse(0x10, 0x22));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_EQUAL_HEX8(0x31, state.vehicleSignals.dids[kRpmIdx].nrc);
    // The next positive answer clears it.
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x10, 0x00));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[kRpmIdx].lastWasNrc);
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[kRpmIdx].hasValue);
    TEST_ASSERT_EQUAL_HEX32(0x1000, state.vehicleSignals.dids[kRpmIdx].raw);
}

void test_vehicle_signals_mirror_the_skip_cooldown(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = kRpmDueMs;
    for (unsigned cycle = 0; cycle < VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS; cycle++) {
        TEST_ASSERT_FALSE(state.vehicleSignals.dids[kRpmIdx].skipped);
        setClock(t);
        module.update(state);
        t += 151;
        setClock(t);
        module.update(state);
        t += 1;
    }
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[kRpmIdx].skipped);
    t += VEHICLE_CL250_DID_SKIP_COOLDOWN_MS + 10;
    setClock(t);
    module.update(state);
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[kRpmIdx].skipped);
}

void test_vehicle_signals_nrc_cleared_by_a_timeout_and_by_the_latch(void) {
    // safety-reviewer M1: an NRC is shown only while it is the DID's latest answer.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[kRpmIdx].lastWasNrc);
    // The next engine-speed read gets no answer at all: the NRC is no longer current.
    unsigned long sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 2, t);
    t = sent + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 5;
    setClock(t);
    module.update(state);
    TEST_ASSERT_TRUE(state.can.unansweredDidCount >= 1);
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[kRpmIdx].lastWasNrc);
    TEST_ASSERT_EQUAL_UINT8(BLE_VSIG_STATE_NEVER_RECEIVED,
                            vsigState(state.vehicleSignals.dids[kRpmIdx], vehicle_cl250_dids[kRpmIdx].stale_after_ms,
                                      (uint32_t)(gT0 + t)));

    // An NRC, then a foreign tester latches the poller off: no NRC and no skip stay current.
    // First leave the quiet window after the timeout (an NRC in it is not attributed),
    // answering every engine-speed read meanwhile so no new timeout extends it.
    const unsigned long quietEnd = t + VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS + 10;
    int reads = 3;
    for (;;) {
        sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, reads++, t);
        if (sent > quietEnd) {
            break;
        }
        bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    }
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x22));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[kRpmIdx].lastWasNrc);
    CanFrame foreign;
    foreign.id = VEHICLE_CL250_REQUEST_ID;
    foreign.extended = true;
    foreign.dlc = 8;
    const uint8_t d[8] = {0x03, UDS_SID_READ_DATA_BY_IDENTIFIER, 0xF4, 0x0D, 0xAA, 0xAA, 0xAA, 0xAA};
    for (int i = 0; i < 8; i++) foreign.data[i] = d[i];
    bus.injectRxFrame(foreign);
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_TRUE(module.latchedOff());
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_FALSE(state.vehicleSignals.dids[i].lastWasNrc);
        TEST_ASSERT_FALSE(state.vehicleSignals.dids[i].skipped);
    }
}

void test_vehicle_signals_quiet_window_nrc_then_answer_stays_valid(void) {
    // An NRC in the quiet window after a timeout (it may be the timed-out read's) is not
    // shown, and the next positive answer makes the record VALID.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    unsigned long sent = runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 1, t);
    t = sent + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 5;
    setClock(t);
    module.update(state); // timeout: the quiet window starts
    runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 2, t);
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[kRpmIdx].lastWasNrc);
    runUntilRequested(module, bus, state, VEHICLE_CL250_DID_ENGINE_SPEED, 3, t);
    bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
    t += 1;
    setClock(t);
    module.update(state);
    TEST_ASSERT_EQUAL_UINT8(BLE_VSIG_STATE_VALID,
                            vsigState(state.vehicleSignals.dids[kRpmIdx], vehicle_cl250_dids[kRpmIdx].stale_after_ms,
                                      (uint32_t)(gT0 + t)));
}

void test_tester_stats_do_not_change_what_is_sent(void) {
    // Same ECU behaviour (answers, 0x78, an NRC, silences that time out), with wildly
    // different micros() values: every transmitted frame, byte for byte, goes out in the
    // same pass and order, so the counters never feed back into the poller.
    struct Sent {
        unsigned long pass;
        CanFrame frame;
    };
    std::vector<Sent> runs[2];
    for (int run = 0; run < 2; run++) {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        startPolling(module, state);
        size_t logged = 0;
        int rpmReads = 0;
        unsigned long deferredAt = 0;
        bool sawValue = false, sawNrc = false, sawSkip = false;
        for (unsigned long t = 1; t < 6000; t++) {
            test_setMicros(run == 0 ? 0 : (uint32_t)(t * 997u + (t % 7u) * 12345u));
            setClock(t);
            const bool silent = t >= 1000 && t < 2000; // one silent second: timeouts
            if (deferredAt == t) {
                bus.injectRxFrame(makePositiveResponse(VEHICLE_CL250_DID_ENGINE_SPEED, 0x46, 0x50));
            }
            if (!silent && t % 263 == 0) {
                bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, 0x31));
            }
            const size_t before = bus.txLog.size();
            module.update(state);
            if (run == 1) {
                // D-066 item 4: note what the bookkeeping saw, then overwrite it with garbage
                // and pack it: what is sent must not depend on it.
                for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
                    const VehicleSignalRecord& r = state.vehicleSignals.dids[i];
                    sawValue = sawValue || r.hasValue;
                    sawNrc = sawNrc || r.lastWasNrc;
                    sawSkip = sawSkip || r.skipped;
                }
                for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
                    VehicleSignalRecord& r = state.vehicleSignals.dids[i];
                    r.raw = 0xDEADBEEFu ^ (uint32_t)t;
                    r.updatedMs = (uint32_t)(t * 31u);
                    r.hasValue = (t & 1u) != 0u;
                    r.lastWasNrc = (t & 2u) != 0u;
                    r.nrc = 0x7F;
                    r.skipped = (t & 4u) != 0u;
                }
                uint8_t block[VSIG_BLOCK_MAX_BYTES];
                (void)packVehicleSignalBlock(state.vehicleSignals, 0, VEHICLE_CL250_DID_COUNT, (uint8_t)t,
                                             (uint32_t)t, block, sizeof(block));
            }
            uint16_t did = 0;
            if (!silent && newDidRequest(bus, before, did)) {
                if (did == VEHICLE_CL250_DID_ENGINE_SPEED && ++rpmReads % 4 == 0) {
                    // Every 4th engine-speed read: 0x78 first, the answer 3 ms later.
                    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
                    deferredAt = t + 3;
                } else {
                    bus.injectRxFrame(makePositiveResponse(did, 0x46, 0x50)); // drained next pass
                }
            }
            for (; logged < bus.txLog.size(); logged++) {
                runs[run].push_back(Sent{t, bus.txLog[logged]});
            }
        }
        if (run == 1) {
            // The scenario did exercise the statistics.
            TEST_ASSERT_TRUE(state.can.unansweredDidCount > 0);
            TEST_ASSERT_TRUE(state.tester.rtt[kRpmIdx].count > 0);
            TEST_ASSERT_TRUE(state.tester.rtt[kRpmIdx].nrc78Count > 0);
            TEST_ASSERT_TRUE(state.tester.stepGapMaxMs > 0);
            TEST_ASSERT_TRUE(sawValue);
            TEST_ASSERT_TRUE(sawNrc);
            TEST_ASSERT_TRUE(sawSkip);
        }
        test_setMicros(0);
    }
    TEST_ASSERT_TRUE(runs[0].size() > 50);
    TEST_ASSERT_EQUAL_UINT32(runs[0].size(), runs[1].size());
    for (size_t i = 0; i < runs[0].size(); i++) {
        const Sent& a = runs[0][i];
        const Sent& b = runs[1][i];
        TEST_ASSERT_EQUAL_UINT32(a.pass, b.pass);
        TEST_ASSERT_EQUAL_HEX32(a.frame.id, b.frame.id);
        TEST_ASSERT_EQUAL(a.frame.extended, b.frame.extended);
        TEST_ASSERT_EQUAL_UINT8(a.frame.dlc, b.frame.dlc);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(a.frame.data, b.frame.data, 8);
    }
}

// ---------------------------------------------------------------------------
// D-068 item 2: unverified DIDs are read once per confirmed session, then only if served
// ---------------------------------------------------------------------------

// What the simulated ECU does with a DID read.
enum class EcuReply : uint8_t { ANSWER, NRC_OUT_OF_RANGE, SILENT, PENDING, NRC_THEN_ANSWER, ANSWER_ZERO };
typedef EcuReply (*EcuBehaviour)(uint16_t did);

static EcuReply ecuAnswersAll(uint16_t) { return EcuReply::ANSWER; }

// First unverified DID of the table (D-068 appends them after the verified ones).
static uint8_t firstUnverifiedIdx() {
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (!vehicle_cl250_dids[i].verified) return i;
    }
    TEST_FAIL_MESSAGE("no unverified DID in the table");
    return 0;
}

static CanFrame makeAnswerFor(uint16_t did) {
    const vehicle_cl250_did_t* e = vehicle_cl250_find(did);
    return makePositiveResponse(did, 0x46, 0x50, (uint8_t)(4u + e->length));
}

// Steps 1 ms at a time up to relative time `until`; a DID read is answered on the next pass
// as `ecu` says. Returns the largest gap between two requests of `watch` (0 = no watch).
static unsigned long runEcu(HondaCANModule& module, MockCanBus& bus, SystemState& state,
                            unsigned long& t, unsigned long until, EcuBehaviour ecu,
                            uint16_t watch = 0) {
    unsigned long lastWatch = 0, maxGap = 0;
    while (t < until) {
        const size_t before = bus.txLog.size();
        t++;
        setClock(t);
        module.update(state);
        uint16_t did = 0;
        if (!newDidRequest(bus, before, did)) continue;
        if (did == watch) {
            if (lastWatch != 0 && t - lastWatch > maxGap) maxGap = t - lastWatch;
            lastWatch = t;
        }
        switch (ecu(did)) {
            case EcuReply::ANSWER: bus.injectRxFrame(makeAnswerFor(did)); break;
            case EcuReply::NRC_OUT_OF_RANGE:
                bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_REQUEST_OUT_OF_RANGE));
                break;
            case EcuReply::SILENT: break;
            case EcuReply::ANSWER_ZERO: // raw 0 (e.g. engine speed with the engine off)
                bus.injectRxFrame(makePositiveResponse(did, 0x00, 0x00, (uint8_t)(4u + vehicle_cl250_find(did)->length)));
                break;
            case EcuReply::PENDING: // NRC 0x78, then nothing
                bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
                break;
            case EcuReply::NRC_THEN_ANSWER: // the two response IDs disagree (D-068 MAJOR-2)
                bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_REQUEST_OUT_OF_RANGE));
                bus.injectRxFrame(makeAnswerFor(did));
                break;
        }
    }
    return maxGap;
}

// The ECU confirms the session; the pass that takes the 0x50 already sends the first
// unverified read, which `ecu` answers like every later one.
static void confirmSession(HondaCANModule& module, MockCanBus& bus, SystemState& state, unsigned long& t,
                           EcuBehaviour ecu) {
    bus.injectRxFrame(makeSessionConfirm());
    runEcu(module, bus, state, t, t + 1, ecu);
}

void test_d068_table_puts_unverified_dids_after_the_verified_ones(void) {
    const uint8_t first = firstUnverifiedIdx();
    TEST_ASSERT_TRUE(first > VEHICLE_CL250_IDX_BATTERY_VOLTAGE);
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL(i < first, vehicle_cl250_dids[i].verified);
    }
}

void test_d068_unverified_dids_wait_for_the_session_then_are_read_at_once(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    // No 0x50 yet: only the verified DIDs are read.
    runEcu(module, bus, state, t, 3000, ecuAnswersAll);
    TEST_ASSERT_TRUE(bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > 0);
    for (uint8_t i = firstUnverifiedIdx(); i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_EQUAL(0, bus.countDidRequests(vehicle_cl250_dids[i].did));
    }
    // Confirmed: every unverified DID is read within the first second, long before its
    // period, and exactly once (each request goes out on two IDs).
    bus.clearTxLog();
    confirmSession(module, bus, state, t, ecuAnswersAll);
    runEcu(module, bus, state, t, t + 1000, ecuAnswersAll);
    for (uint8_t i = firstUnverifiedIdx(); i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_TRUE(vehicle_cl250_dids[i].poll_period_ms > 1000);
        TEST_ASSERT_EQUAL(2, bus.countDidRequests(vehicle_cl250_dids[i].did));
        TEST_ASSERT_TRUE(state.vehicleSignals.dids[i].hasValue);
        TEST_ASSERT_FALSE(module.didStopped(i));
    }
}

static uint16_t gStopDid = 0;
static EcuReply ecuRefusesStopDid(uint16_t did) {
    return did == gStopDid ? EcuReply::NRC_OUT_OF_RANGE : EcuReply::ANSWER;
}
static EcuReply ecuSilentOnStopDid(uint16_t did) {
    return did == gStopDid ? EcuReply::SILENT : EcuReply::ANSWER;
}
static EcuReply ecuSilent(uint16_t) { return EcuReply::SILENT; }

void test_d068_nrc31_stops_an_unverified_did_until_the_session_is_confirmed_again(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuRefusesStopDid);
    runEcu(module, bus, state, t, 20000, ecuRefusesStopDid);
    // Refused on two separate reads (two IDs each), then never asked again.
    TEST_ASSERT_EQUAL(2 * kNrc31ToStop, bus.countDidRequests(gStopDid));
    TEST_ASSERT_TRUE(module.didStopped(idx));
    const VehicleSignalRecord& r = state.vehicleSignals.dids[idx];
    TEST_ASSERT_TRUE(r.lastWasNrc); // the block shows NRC 0x31, not "skipped"
    TEST_ASSERT_EQUAL_HEX8(UDS_NRC_REQUEST_OUT_OF_RANGE, r.nrc);
    TEST_ASSERT_FALSE(r.skipped);
    // The next unverified DID, answered, keeps its period.
    TEST_ASSERT_TRUE(bus.countDidRequests(vehicle_cl250_dids[idx + 1].did) >= 2 * 5);

    // The ECU goes away (no answer for longer than the absent timeout), then comes back:
    // the confirmed session asks the stopped DID again.
    runEcu(module, bus, state, t, t + VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS + 500, ecuSilent);
    TEST_ASSERT_FALSE(state.engine.ecuPresent);
    confirmSession(module, bus, state, t, ecuAnswersAll);
    TEST_ASSERT_FALSE(module.didStopped(idx));
    runEcu(module, bus, state, t, t + 1000, ecuAnswersAll);
    TEST_ASSERT_EQUAL(2 * kNrc31ToStop + 2, bus.countDidRequests(gStopDid));
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[idx].lastWasNrc);
}

void test_d068_never_answered_unverified_did_stops_after_max_timeouts(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuSilentOnStopDid);
    runEcu(module, bus, state, t, 60000, ecuSilentOnStopDid);
    TEST_ASSERT_EQUAL(2 * VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS, bus.countDidRequests(gStopDid));
    TEST_ASSERT_TRUE(module.didStopped(idx));
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[idx].skipped); // shown as skipped
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[idx].lastWasNrc);
}

static int gStopDidAnswers = 0;
static EcuReply ecuAnswersStopDidOnce(uint16_t did) {
    if (did != gStopDid) return EcuReply::ANSWER;
    return gStopDidAnswers++ == 0 ? EcuReply::ANSWER : EcuReply::SILENT;
}

void test_d068_an_answered_unverified_did_keeps_the_skip_and_retry(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    gStopDidAnswers = 0;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuAnswersStopDidOnce);
    // One answer, then silence: served once, so the D-050..D-053 skip and retry, not a stop.
    runEcu(module, bus, state, t, 60000, ecuAnswersStopDidOnce);
    TEST_ASSERT_FALSE(module.didStopped(idx));
    TEST_ASSERT_TRUE(bus.countDidRequests(gStopDid) > (int)(2 * (1 + VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS)));
}

static EcuReply ecuRefusesRpm(uint16_t did) {
    return did == VEHICLE_CL250_DID_ENGINE_SPEED ? EcuReply::NRC_OUT_OF_RANGE : EcuReply::ANSWER;
}

void test_d068_nrc31_never_stops_a_verified_did(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuRefusesRpm);
    runEcu(module, bus, state, t, 2000, ecuRefusesRpm);
    const int before = bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED);
    runEcu(module, bus, state, t, 4000, ecuRefusesRpm);
    TEST_ASSERT_FALSE(module.didStopped(kRpmIdx));
    TEST_ASSERT_TRUE(bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > before);
}

static EcuReply ecuServesOnlyVerified(uint16_t did) {
    return vehicle_cl250_find(did)->verified ? EcuReply::ANSWER : EcuReply::SILENT;
}

void test_d068_verified_dids_stay_fresh_while_every_unverified_did_times_out(void) {
    // D-068 item 1: every unverified read times out (the worst case of an ECU that ignores
    // unknown DIDs), yet RPM and speed wait for at most the one read in flight: request gap
    // <= period + base timeout + one step, plus one round trip for speed (RPM is ahead).
    const unsigned long base = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
    const unsigned long step = VEHICLE_CL250_CLIENT_STEP_MAX_MS;
    const unsigned long rtt = VEHICLE_CL250_ASSUMED_ROUND_TRIP_MS;
    struct Case { uint16_t did; uint8_t idx; unsigned long extra; };
    const Case cases[] = {
        {VEHICLE_CL250_DID_ENGINE_SPEED, VEHICLE_CL250_IDX_ENGINE_SPEED, 0},
        {VEHICLE_CL250_DID_VEHICLE_SPEED, VEHICLE_CL250_IDX_VEHICLE_SPEED, rtt},
    };
    for (const Case& c : cases) {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        startPolling(module, state);
        unsigned long t = 0;
        confirmSession(module, bus, state, t, ecuServesOnlyVerified);
        const unsigned long gap = runEcu(module, bus, state, t, 30000, ecuServesOnlyVerified, c.did);
        TEST_ASSERT_TRUE(gap > 0);
        TEST_ASSERT_TRUE(gap <= vehicle_cl250_dids[c.idx].poll_period_ms + base + step + c.extra);
        TEST_ASSERT_TRUE(gap <= vehicle_cl250_dids[c.idx].stale_after_ms);
    }
    // Throttle, coolant and battery: within their stale_after_ms as well.
    for (uint8_t idx = VEHICLE_CL250_IDX_THROTTLE_POS; idx < firstUnverifiedIdx(); idx++) {
        MockCanBus bus;
        MockTesterLatchStore latchStore;
        HondaCANModule module(bus, latchStore);
        SystemState state;
        startPolling(module, state);
        unsigned long t = 0;
        confirmSession(module, bus, state, t, ecuServesOnlyVerified);
        const unsigned long gap = runEcu(module, bus, state, t, 30000, ecuServesOnlyVerified, vehicle_cl250_dids[idx].did);
        TEST_ASSERT_TRUE(gap > 0);
        TEST_ASSERT_TRUE(gap <= vehicle_cl250_dids[idx].stale_after_ms);
    }
}

static EcuReply ecuUnverifiedPending(uint16_t did) {
    return vehicle_cl250_find(did)->verified ? EcuReply::ANSWER : EcuReply::PENDING;
}

void test_d068_an_unverified_did_gets_no_0x78_extension(void) {
    // Safety-reviewer MAJOR-1: with the 0x78 extension an unverified read could hold the
    // slot for up to RESPONSE_TIMEOUT_MAX_MS; without it, RPM keeps its fault-mode gap.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuUnverifiedPending);
    const unsigned long gap = runEcu(module, bus, state, t, 20000, ecuUnverifiedPending,
                                     VEHICLE_CL250_DID_ENGINE_SPEED);
    TEST_ASSERT_TRUE(gap <= kRpmDueMs + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + VEHICLE_CL250_CLIENT_STEP_MAX_MS);
    // A 0x78 is no answer: each such DID is stopped after its timeouts.
    TEST_ASSERT_TRUE(module.didStopped(firstUnverifiedIdx()));
}

static EcuReply ecuRefusesStopDidOnce(uint16_t did) {
    if (did != gStopDid) return EcuReply::ANSWER;
    return gStopDidAnswers++ == 0 ? EcuReply::NRC_OUT_OF_RANGE : EcuReply::ANSWER;
}

void test_d068_one_nrc31_does_not_stop_an_unverified_did(void) {
    // Safety-reviewer MAJOR-2: one 0x31 may be the other response ID's or trail an earlier
    // answer; it takes two separate reads.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    gStopDidAnswers = 0;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuRefusesStopDidOnce);
    runEcu(module, bus, state, t, 10000, ecuRefusesStopDidOnce);
    TEST_ASSERT_FALSE(module.didStopped(idx));
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[idx].hasValue);
}

static EcuReply ecuStopDidNrcThenAnswer(uint16_t did) {
    return did == gStopDid ? EcuReply::NRC_THEN_ANSWER : EcuReply::ANSWER;
}

void test_d068_a_positive_answer_undoes_an_nrc31_stop(void) {
    // Safety-reviewer MAJOR-2 (a): both IDs answer, one with 0x31 first. The validated
    // positive echo proves the DID is served, so it is never left stopped.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuStopDidNrcThenAnswer);
    runEcu(module, bus, state, t, 20000, ecuStopDidNrcThenAnswer);
    TEST_ASSERT_FALSE(module.didStopped(idx));
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[idx].hasValue);
    TEST_ASSERT_TRUE(bus.countDidRequests(gStopDid) >= 2 * 5); // keeps its period
}

static int countSessionRequests(const MockCanBus& bus) {
    int n = 0;
    for (const CanFrame& f : bus.txLog) {
        if (f.dlc >= 3 && f.data[1] == VEHICLE_CL250_SESSION_SID) n++;
    }
    return n;
}

static bool gEngineRunning = false;
static uint16_t gAlwaysRefusedDid = 0;
static EcuReply ecuServesStopDidOnlyWithEngineRunning(uint16_t did) {
    if (did == VEHICLE_CL250_DID_ENGINE_SPEED) {
        return gEngineRunning ? EcuReply::ANSWER : EcuReply::ANSWER_ZERO;
    }
    if (did == gAlwaysRefusedDid) {
        return EcuReply::NRC_OUT_OF_RANGE;
    }
    if (did == gStopDid) {
        return gEngineRunning ? EcuReply::ANSWER : EcuReply::NRC_OUT_OF_RANGE;
    }
    return EcuReply::ANSWER;
}

static uint8_t secondUnverifiedIdx() {
    bool first = true;
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (vehicle_cl250_dids[i].verified) continue;
        if (!first) return i;
        first = false;
    }
    TEST_FAIL_MESSAGE("fewer than two unverified DIDs in the table");
    return 0;
}

void test_d069_engine_start_rearms_the_stopped_dids_once(void) {
    // D-069 item 3. Key-on sweep with the engine off: the ECU refuses one PID it serves only
    // when running, and another it never serves. The first engine start (two RPM samples
    // above 0 after a valid 0) asks both again; a stall and restart in the same session does
    // not re-arm a second time.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    const uint8_t refusedIdx = secondUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    gAlwaysRefusedDid = vehicle_cl250_dids[refusedIdx].did;
    gEngineRunning = false;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuServesStopDidOnlyWithEngineRunning);
    runEcu(module, bus, state, t, 8000, ecuServesStopDidOnlyWithEngineRunning);
    TEST_ASSERT_TRUE(module.didStopped(idx));
    TEST_ASSERT_TRUE(module.didStopped(refusedIdx));
    TEST_ASSERT_FALSE(state.vehicleSignals.dids[idx].hasValue);

    // Engine start: both are pollable again and the engine-only PID now has a value.
    gEngineRunning = true;
    runEcu(module, bus, state, t, t + 5000, ecuServesStopDidOnlyWithEngineRunning);
    TEST_ASSERT_FALSE(module.didStopped(idx));
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[idx].hasValue);
    TEST_ASSERT_TRUE(module.didStopped(refusedIdx)); // asked again, refused twice, stopped again

    // Stall and restart: no second re-arm in this confirmed session.
    const int refusedRequests = bus.countDidRequests(gAlwaysRefusedDid);
    gEngineRunning = false;
    runEcu(module, bus, state, t, t + 2000, ecuServesStopDidOnlyWithEngineRunning);
    gEngineRunning = true;
    runEcu(module, bus, state, t, t + 5000, ecuServesStopDidOnlyWithEngineRunning);
    TEST_ASSERT_TRUE(module.didStopped(refusedIdx));
    TEST_ASSERT_EQUAL_INT(refusedRequests, bus.countDidRequests(gAlwaysRefusedDid));
    gAlwaysRefusedDid = 0;
}

static int gRpmReads = 0;
static EcuReply ecuRpmRunBrokenByNrc(uint16_t did) {
    if (did == VEHICLE_CL250_DID_ENGINE_SPEED && gEngineRunning) {
        // Running: every other RPM read is refused, so no two answers above 0 are consecutive.
        return (gRpmReads++ % 2 == 0) ? EcuReply::ANSWER : EcuReply::NRC_OUT_OF_RANGE;
    }
    return ecuServesStopDidOnlyWithEngineRunning(did);
}

void test_d069_an_nrc_on_rpm_breaks_the_engine_start_run(void) {
    // safety-reviewer MINOR-2: 0, >0, NRC, >0, NRC ... is no engine start.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    gEngineRunning = false;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuRpmRunBrokenByNrc);
    runEcu(module, bus, state, t, 8000, ecuRpmRunBrokenByNrc);
    TEST_ASSERT_TRUE(module.didStopped(idx));
    gEngineRunning = true;
    gRpmReads = 0;
    runEcu(module, bus, state, t, t + 3000, ecuRpmRunBrokenByNrc);
    TEST_ASSERT_TRUE(module.didStopped(idx));
}

void test_d069_engine_start_rearm_keeps_the_tier_slot(void) {
    // D-069 item 3 (safety-reviewer, Q-030): a re-armed DID is due one cadence after its last
    // request, not at once, so the re-ask never exceeds its tier rate.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    gEngineRunning = false;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuServesStopDidOnlyWithEngineRunning);
    // Stop it, then start the engine right after its last request.
    while (!module.didStopped(idx)) {
        runEcu(module, bus, state, t, t + 10, ecuServesStopDidOnlyWithEngineRunning);
        TEST_ASSERT_TRUE_MESSAGE(t < 8000, "the DID was never stopped");
    }
    const int requests = bus.countDidRequests(gStopDid);
    gEngineRunning = true;
    const unsigned long period = vehicle_cl250_dids[idx].poll_period_ms;
    runEcu(module, bus, state, t, t + period / 2, ecuServesStopDidOnlyWithEngineRunning);
    TEST_ASSERT_FALSE(module.didStopped(idx));                         // re-armed ...
    TEST_ASSERT_EQUAL_INT(requests, bus.countDidRequests(gStopDid));  // ... but not asked yet
    runEcu(module, bus, state, t, t + period + 500, ecuServesStopDidOnlyWithEngineRunning);
    TEST_ASSERT_TRUE(bus.countDidRequests(gStopDid) > requests);
}

void test_d068_session_lost_without_ecu_absence_rearms_the_stopped_dids(void) {
    // Safety-reviewer MINOR-1: the ECU drops to the default session (short reset, S3) and
    // refuses reads with 0x7F; the tester asks for the session again and its confirmation
    // re-arms the unverified DIDs that the 0x31s of the default session stopped.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idx = firstUnverifiedIdx();
    gStopDid = vehicle_cl250_dids[idx].did;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuRefusesStopDid);
    runEcu(module, bus, state, t, 6000, ecuRefusesStopDid);
    TEST_ASSERT_TRUE(module.didStopped(idx));
    bus.clearTxLog();
    // One read refused as not supported in the active session.
    bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION));
    runEcu(module, bus, state, t, t + VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS + 100, ecuAnswersAll);
    TEST_ASSERT_TRUE(countSessionRequests(bus) >= 2); // 0x10 0x03 again, on both IDs
    TEST_ASSERT_TRUE(state.engine.ecuPresent);        // no 3 s silence was needed
    confirmSession(module, bus, state, t, ecuAnswersAll);
    TEST_ASSERT_FALSE(module.didStopped(idx));
    runEcu(module, bus, state, t, t + 1000, ecuAnswersAll);
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[idx].hasValue);
}

static uint16_t gLongGapDid2 = 0;
static EcuReply ecuTwoServedOnlyWithEngineRunning(uint16_t did) {
    if (did == VEHICLE_CL250_DID_ENGINE_SPEED) {
        return gEngineRunning ? EcuReply::ANSWER : EcuReply::ANSWER_ZERO;
    }
    if (did == gStopDid || did == gLongGapDid2) {
        return gEngineRunning ? EcuReply::ANSWER : EcuReply::NRC_OUT_OF_RANGE;
    }
    return EcuReply::ANSWER;
}

// Frames of DID reads in txLog order (each request goes out as two frames, one per ID).
static std::vector<uint16_t> didReadFrames(const MockCanBus& bus) {
    std::vector<uint16_t> dids;
    for (const CanFrame& f : bus.txLog) {
        if (f.dlc >= 4 && f.data[1] == UDS_SID_READ_DATA_BY_IDENTIFIER) {
            dids.push_back((uint16_t)((f.data[2] << 8) | f.data[3]));
        }
    }
    return dids;
}

// What runEcuDelayed() observed.
struct DelayedRun {
    unsigned long maxWatchGap = 0;    // largest gap between two requests of the watched DID
    int watchRequests = 0;            // requests of the watched DID
    int overlaps = 0;                 // requests sent while another one was still pending
    int maxFramesPerPass = 0;         // read frames of one update() (2 = one request, both IDs)
    int mixedPasses = 0;              // passes whose read frames were for more than one DID
};

// Like runEcu, but every read is answered (positive, ecu() must say ANSWER) only `delayMs`
// after its request (the two slow DIDs `slowDelayMs`), so a request stays pending across
// many passes.
static DelayedRun runEcuDelayed(HondaCANModule& module, MockCanBus& bus, SystemState& state,
                                unsigned long& t, unsigned long until, EcuBehaviour ecu,
                                unsigned long delayMs, uint16_t watch, uint16_t slowDid1 = 0,
                                uint16_t slowDid2 = 0, unsigned long slowDelayMs = 0) {
    DelayedRun r;
    bool pending = false;
    uint16_t pendingDid = 0;
    unsigned long dueAt = 0, lastWatch = 0;
    while (t < until) {
        t++;
        setClock(t);
        if (pending && t >= dueAt) {
            bus.injectRxFrame(makeAnswerFor(pendingDid));
            pending = false;
        }
        const size_t before = bus.txLog.size();
        module.update(state);
        int frames = 0;
        uint16_t did = 0;
        for (size_t i = before; i < bus.txLog.size(); i++) {
            const CanFrame& f = bus.txLog[i];
            if (f.dlc < 4 || f.data[1] != UDS_SID_READ_DATA_BY_IDENTIFIER) continue;
            const uint16_t d = (uint16_t)((f.data[2] << 8) | f.data[3]);
            if (frames > 0 && d != did) r.mixedPasses++;
            did = d;
            frames++;
        }
        if (frames > r.maxFramesPerPass) r.maxFramesPerPass = frames;
        if (frames == 0) continue;
        if (pending) r.overlaps++;
        TEST_ASSERT_TRUE(ecu(did) == EcuReply::ANSWER);
        pending = true;
        pendingDid = did;
        dueAt = t + ((did == slowDid1 || did == slowDid2) ? slowDelayMs : delayMs);
        if (did == watch) {
            r.watchRequests++;
            if (lastWatch != 0 && t - lastWatch > r.maxWatchGap) r.maxWatchGap = t - lastWatch;
            lastWatch = t;
        }
    }
    return r;
}

void test_d069_long_gap_rearm_all_due_together(void) {
    // conn#18 MINOR-3. Two stopped DIDs of different tiers; a long gap since their last request
    // (engine off for 60 s); at the engine start both are due at the same moment. The per-DID
    // rate and the one-request-at-a-time rule still hold, and the RPM read is not starved.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    const uint8_t idxA = firstUnverifiedIdx();
    uint8_t idxB = idxA;
    for (uint8_t i = idxA; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (!vehicle_cl250_dids[i].verified &&
            vehicle_cl250_dids[i].poll_period_ms > vehicle_cl250_dids[idxB].poll_period_ms) {
            idxB = i; // slowest tier of the table
        }
    }
    TEST_ASSERT_TRUE(vehicle_cl250_dids[idxB].poll_period_ms > vehicle_cl250_dids[idxA].poll_period_ms);
    gStopDid = vehicle_cl250_dids[idxA].did;
    gLongGapDid2 = vehicle_cl250_dids[idxB].did;
    gEngineRunning = false;
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuTwoServedOnlyWithEngineRunning);
    while (!module.didStopped(idxA) || !module.didStopped(idxB)) {
        runEcu(module, bus, state, t, t + 100, ecuTwoServedOnlyWithEngineRunning);
        TEST_ASSERT_TRUE_MESSAGE(t < 30000, "the DIDs were never stopped");
    }
    runEcu(module, bus, state, t, t + 60000, ecuTwoServedOnlyWithEngineRunning);
    TEST_ASSERT_TRUE(module.didStopped(idxA));
    TEST_ASSERT_TRUE(module.didStopped(idxB));

    // Engine start: the RPM samples re-arm every stopped DID; all of them are overdue.
    bus.clearTxLog();
    gEngineRunning = true;
    // Answers come half a response timeout late, the re-armed DIDs' just inside it, so every
    // request stays pending for many passes and the RPM read is stressed. (Answers just
    // inside the timeout for all DIDs would overload the verified 110 ms reads alone and
    // starve the unverified ones: that is the scheduler's priority, not tested here.)
    // Bound: the start needs kEngineStartSamples (2) RPM reads, each at most one RPM period
    // plus one slot (response timeout base + client step) apart; then every unverified DID
    // takes at most one slot, and so does each verified DID (one read of each ahead of any
    // given DID).
    const unsigned long slotMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + VEHICLE_CL250_CLIENT_STEP_MAX_MS;
    const unsigned long answerDelayMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS / 2;
    const unsigned long slowDelayMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS - VEHICLE_CL250_CLIENT_STEP_MAX_MS;
    unsigned long unverified = 0;
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        if (!vehicle_cl250_dids[i].verified) unverified++;
    }
    const unsigned long verifiedCount = VEHICLE_CL250_DID_COUNT - unverified;
    const unsigned long burstBoundMs = 2 * (kRpmDueMs + slotMs) + (unverified + verifiedCount) * slotMs;
    const uint32_t timeoutsBefore = module.unansweredDidCount();
    const DelayedRun burst = runEcuDelayed(module, bus, state, t, t + burstBoundMs,
                                           ecuTwoServedOnlyWithEngineRunning, answerDelayMs,
                                           VEHICLE_CL250_DID_ENGINE_SPEED, gStopDid, gLongGapDid2,
                                           slowDelayMs);
    TEST_ASSERT_FALSE(module.didStopped(idxA));
    TEST_ASSERT_FALSE(module.didStopped(idxB));
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[idxA].hasValue);
    TEST_ASSERT_TRUE(state.vehicleSignals.dids[idxB].hasValue);
    // (i) both asked again within the bound; the 10000 ms DID exactly once, the 2000 ms DID
    // at most once per its period (plus the first request).
    TEST_ASSERT_EQUAL_INT(2, bus.countDidRequests(gLongGapDid2));
    TEST_ASSERT_TRUE(bus.countDidRequests(gStopDid) >= 2);
    TEST_ASSERT_TRUE(bus.countDidRequests(gStopDid) / 2 <=
                     (int)(burstBoundMs / vehicle_cl250_dids[idxA].poll_period_ms + 1));
    // (iv) RPM keeps its rate: asked at least twice (no vacuous gap) and never later than a
    // period plus one slot.
    TEST_ASSERT_TRUE(burst.watchRequests >= 2);
    TEST_ASSERT_TRUE(burst.maxWatchGap <= kRpmDueMs + slotMs);
    // (ii) One request outstanding at a time: at most one new request (two frames, one DID)
    // per update(), none while another one waits for its answer, and no timeout meanwhile.
    TEST_ASSERT_EQUAL_INT(0, burst.overlaps);
    TEST_ASSERT_EQUAL_INT(0, burst.mixedPasses);
    TEST_ASSERT_TRUE(burst.maxFramesPerPass <= 2);
    TEST_ASSERT_EQUAL_UINT32(timeoutsBefore, module.unansweredDidCount());

    // (iii) After the burst each is asked at most once per its period: over three periods of
    // it, at most 3 + 1 requests.
    const unsigned long periodA = vehicle_cl250_dids[idxA].poll_period_ms;
    const unsigned long periodB = vehicle_cl250_dids[idxB].poll_period_ms;
    bus.clearTxLog();
    runEcu(module, bus, state, t, t + 3 * periodA, ecuTwoServedOnlyWithEngineRunning);
    TEST_ASSERT_TRUE(bus.countDidRequests(gStopDid) / 2 <= 3 + 1);
    bus.clearTxLog();
    runEcu(module, bus, state, t, t + 3 * periodB, ecuTwoServedOnlyWithEngineRunning);
    TEST_ASSERT_TRUE(bus.countDidRequests(gLongGapDid2) / 2 <= 3 + 1);
    gLongGapDid2 = 0;
}

static EcuReply ecuUnverifiedSilent(uint16_t did) {
    return vehicle_cl250_find(did)->verified ? EcuReply::ANSWER : EcuReply::SILENT;
}

void test_timeout_warnings_are_rate_limited(void) {
    // conn#18 follow-up: at most one timeout happens per pass, so the DID timeout warning is
    // bounded per time window. The counters and the stop rule do not depend on the logging.
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    Serial.watchA = "Timeout waiting for DID";
    Serial.watchB = "DID timeout warnings suppressed";
    Serial.hitsA = Serial.hitsB = 0;
    Serial.linesA.clear();
    Serial.linesB.clear();
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecuUnverifiedSilent);
    const unsigned long durationMs = 60000;
    runEcu(module, bus, state, t, durationMs, ecuUnverifiedSilent);
    // Timeouts have stopped (every silent DID is stopped); the last window's count is
    // reported once on a later pass.
    runEcu(module, bus, state, t, t + 3000, ecuUnverifiedSilent);
    const int printed = Serial.hitsA;
    const int flushes = Serial.hitsB;
    unsigned long suppressed = 0;
    for (const std::string& line : Serial.linesA) {
        const char* p = std::strstr(line.c_str(), "(+");
        if (p) suppressed += std::strtoul(p + 2, nullptr, 10); // folded into the next window's line
    }
    for (const std::string& line : Serial.linesB) {
        suppressed += std::strtoul(line.c_str() + std::strlen("[UDS WARNING] "), nullptr, 10);
    }
    Serial.watchA.clear();
    Serial.watchB.clear();

    // The window opens at a timeout, so at most durationMs / window + 1 windows were used.
    const int windows = (int)(durationMs / 1000) + 1;
    TEST_ASSERT_TRUE(printed > 0);
    TEST_ASSERT_TRUE(printed <= windows * (int)HondaCANModule::maxTimeoutLogLinesPerWindow());
    TEST_ASSERT_TRUE(suppressed > 0);
    TEST_ASSERT_TRUE(flushes >= 1 && flushes <= windows); // at most one summary per window
    // Every timeout is accounted for, in a printed line or a suppressed count.
    TEST_ASSERT_EQUAL_UINT32(module.unansweredDidCount(), (uint32_t)printed + suppressed);
    // The stop rule ran as without the limit: every silent unverified DID ended up stopped
    // (the verified ones answer, so the ECU stays present).
    for (uint8_t i = firstUnverifiedIdx(); i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_TRUE(module.didStopped(i));
    }
}

// ---------------------------------------------------------------------------
// E-19 (3): armUnverifiedReads() and the stop rule around the millis() wrap
// ---------------------------------------------------------------------------
// Everything below is unsigned-long arithmetic on a deadline kept as a timestamp, so a run
// shifted across the wrap must behave exactly like the same run far from it. (The host's
// unsigned long is 64-bit, the target's 32-bit; the arithmetic is the same, the wrap is
// at 2^64 here.) The reference run uses the normal base; the shifted runs put the wrap
// before, at and after the session confirmation (arming: lastRequestMs = now - cadence
// underflows when now < cadence) and in the middle of the stop-rule sequences.

struct WrapRun {
    int requests1[VEHICLE_CL250_DID_COUNT]; // after the first stage
    int requests2[VEHICLE_CL250_DID_COUNT]; // after the ECU absence and the second confirmation
    bool stopped1[VEHICLE_CL250_DID_COUNT];
    bool stopped2[VEHICLE_CL250_DID_COUNT];
    bool hasValue2[VEHICLE_CL250_DID_COUNT];
    int atOneSecond[VEHICLE_CL250_DID_COUNT]; // requests within 1 s of the first confirmation
};

// base = the clock value of setClock(0). Stage 1: confirm, run 60 s. Stage 2: the ECU is
// silent for longer than the absent timeout, then the session is confirmed again (arming
// again, with the stop flags set by stage 1) and the run goes on for 1 s.
static WrapRun runWrapScenario(unsigned long base, EcuBehaviour ecu) {
    gT0 = base;
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    WrapRun r;
    startPolling(module, state);
    unsigned long t = 0;
    confirmSession(module, bus, state, t, ecu);
    runEcu(module, bus, state, t, 1000, ecu);
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        r.atOneSecond[i] = bus.countDidRequests(vehicle_cl250_dids[i].did);
    }
    runEcu(module, bus, state, t, 60000, ecu);
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        r.requests1[i] = bus.countDidRequests(vehicle_cl250_dids[i].did);
        r.stopped1[i] = module.didStopped(i);
    }
    runEcu(module, bus, state, t, t + VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS + 500, ecuSilent);
    confirmSession(module, bus, state, t, ecuAnswersAll);
    runEcu(module, bus, state, t, t + 1000, ecuAnswersAll);
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        r.requests2[i] = bus.countDidRequests(vehicle_cl250_dids[i].did);
        r.stopped2[i] = module.didStopped(i);
        r.hasValue2[i] = state.vehicleSignals.dids[i].hasValue;
    }
    gT0 = kT0;
    return r;
}

// Wrap 1500 ms after the confirmation, 21 ms before, exactly at, 49 ms after (arming with
// now < cadence), and one inside the 60 s stage.
static const long kWrapOffsets[] = {-1500, -21, -1, 49, -30000};

static void assertWrapRunsMatch(EcuBehaviour ecu, const WrapRun& ref) {
    for (size_t k = 0; k < sizeof(kWrapOffsets) / sizeof(kWrapOffsets[0]); k++) {
        // base such that setClock(0) lies kWrapOffsets[k] ms from the wrap
        const WrapRun w = runWrapScenario((unsigned long)0 + (unsigned long)kWrapOffsets[k], ecu);
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            TEST_ASSERT_EQUAL_MESSAGE(ref.atOneSecond[i], w.atOneSecond[i], "requests in the first second");
            TEST_ASSERT_EQUAL_MESSAGE(ref.requests1[i], w.requests1[i], "requests after stage 1");
            TEST_ASSERT_EQUAL_MESSAGE(ref.requests2[i], w.requests2[i], "requests after re-arming");
            TEST_ASSERT_EQUAL_MESSAGE(ref.stopped1[i], w.stopped1[i], "stopped after stage 1");
            TEST_ASSERT_EQUAL_MESSAGE(ref.stopped2[i], w.stopped2[i], "stopped after re-arming");
            TEST_ASSERT_EQUAL_MESSAGE(ref.hasValue2[i], w.hasValue2[i], "value after re-arming");
        }
    }
}

void test_d068_arming_across_the_millis_wrap_reads_every_unverified_did_once(void) {
    const WrapRun ref = runWrapScenario(kT0, ecuAnswersAll);
    // The reference itself is the D-068 behaviour: all unverified DIDs read once (two IDs)
    // within the first second, long before their period, then kept on their period.
    for (uint8_t i = firstUnverifiedIdx(); i < VEHICLE_CL250_DID_COUNT; i++) {
        TEST_ASSERT_TRUE(vehicle_cl250_dids[i].poll_period_ms > 1000);
        TEST_ASSERT_EQUAL(2, ref.atOneSecond[i]);
        TEST_ASSERT_FALSE(ref.stopped2[i]);
        TEST_ASSERT_TRUE(ref.hasValue2[i]);
    }
    assertWrapRunsMatch(ecuAnswersAll, ref);
}

void test_d068_nrc31_stop_and_rearm_across_the_millis_wrap(void) {
    gStopDid = vehicle_cl250_dids[firstUnverifiedIdx()].did;
    const uint8_t idx = firstUnverifiedIdx();
    const WrapRun ref = runWrapScenario(kT0, ecuRefusesStopDid);
    TEST_ASSERT_EQUAL(2 * kNrc31ToStop, ref.requests1[idx]); // refused on two separate reads
    TEST_ASSERT_TRUE(ref.stopped1[idx]);
    TEST_ASSERT_FALSE(ref.stopped2[idx]); // the second confirmation armed it again
    assertWrapRunsMatch(ecuRefusesStopDid, ref);
}

void test_d068_max_timeouts_stop_and_rearm_across_the_millis_wrap(void) {
    gStopDid = vehicle_cl250_dids[firstUnverifiedIdx()].did;
    const uint8_t idx = firstUnverifiedIdx();
    const WrapRun ref = runWrapScenario(kT0, ecuSilentOnStopDid);
    TEST_ASSERT_EQUAL(2 * VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS, ref.requests1[idx]);
    TEST_ASSERT_TRUE(ref.stopped1[idx]);
    TEST_ASSERT_FALSE(ref.stopped2[idx]);
    assertWrapRunsMatch(ecuSilentOnStopDid, ref);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rpm_decode_updates_state_and_ecu_presence);
    RUN_TEST(test_session_confirm_is_recognized);
    RUN_TEST(test_nrc_other_than_pending_resolves_request_without_corrupting_state);
    RUN_TEST(test_multiframe_response_is_dropped_not_misparsed);
    RUN_TEST(test_first_frame_never_gets_a_flow_control_outside_the_probe);
    RUN_TEST(test_bus_off_triggers_recovery_with_backoff);
    RUN_TEST(test_bus_recovery_complete_restarts_driver);
    RUN_TEST(test_did_skipped_after_max_consecutive_timeouts_then_resumes);
    RUN_TEST(test_every_transmitted_frame_passes_the_d020_guard);
    RUN_TEST(test_requests_go_to_primary_and_fallback_ids);
    RUN_TEST(test_fallback_response_is_decoded);
    RUN_TEST(test_all_dids_decode_with_generated_formulas);
    RUN_TEST(test_short_battery_response_is_dropped);
    RUN_TEST(test_frames_from_other_ids_are_ignored);
    RUN_TEST(test_emitted_set_is_exact_across_ecu_behaviours);
    RUN_TEST(test_forbidden_requests_are_refused_and_never_transmitted);
    RUN_TEST(test_response_pending_storm_still_times_out);
    RUN_TEST(test_nrc_for_other_service_does_not_resolve_pending_read);
    RUN_TEST(test_foreign_tester_latches_poller_off);
    RUN_TEST(test_repeated_bus_off_latches_off);
    RUN_TEST(test_session_confirm_requires_extended_subfunction);
    RUN_TEST(test_health_reports_bus_state_and_error_counters);
    RUN_TEST(test_health_counts_unanswered_did_requests);
    RUN_TEST(test_health_marks_foreign_tester_latch_and_freezes);
    RUN_TEST(test_health_marks_bus_off_latch);
    RUN_TEST(test_latch_record_round_trip_and_corruption);
    RUN_TEST(test_restore_rule_is_fail_safe);
    RUN_TEST(test_nothing_is_sent_during_the_listen_only_window);
    RUN_TEST(test_listen_window_starts_at_first_update_not_begin);
    RUN_TEST(test_lost_frames_restart_the_listen_window);
    RUN_TEST(test_rx_backlog_restarts_the_listen_window);
    RUN_TEST(test_listen_window_survives_millis_wrap);
    RUN_TEST(test_foreign_tester_during_listen_window_latches_before_any_tx);
    RUN_TEST(test_unsolicited_ecu_answer_during_listen_window_latches);
    RUN_TEST(test_any_diagnostic_request_id_counts_as_foreign_tester);
    RUN_TEST(test_obd_functional_request_ids_known_answer);
    RUN_TEST(test_foreign_frame_right_after_normal_mode_blocks_the_session_request);
    RUN_TEST(test_foreign_tester_latch_survives_reset);
    RUN_TEST(test_bus_off_latch_survives_reset);
    RUN_TEST(test_bus_off_budget_survives_reset);
    RUN_TEST(test_spent_bus_off_budget_in_an_armed_record_latches);
    RUN_TEST(test_invalid_record_after_reset_latches_and_power_on_rearms);
    RUN_TEST(test_latch_is_saved_before_the_driver_stops);
    RUN_TEST(test_normal_mode_failure_disables_the_poller);
    RUN_TEST(test_driver_install_failure_fails_begin);
    RUN_TEST(test_queued_response_is_not_counted_as_timeout_after_slow_pass);
    RUN_TEST(test_foreign_tester_in_queue_stops_all_tx_in_that_pass);
    RUN_TEST(test_rx_drain_is_bounded_and_backlog_pass_sends_nothing);
    RUN_TEST(test_step_gap_max_and_over_count_round_up_to_ms);
    RUN_TEST(test_step_gap_counts_every_entry_also_in_the_listen_window_and_when_latched);
    RUN_TEST(test_step_gap_saturates_at_65535);
    RUN_TEST(test_round_trip_sample_on_positive_response);
    RUN_TEST(test_round_trip_none_on_timeout);
    RUN_TEST(test_round_trip_nrc78_counted_once_per_request_and_gives_no_sample);
    RUN_TEST(test_round_trip_sample_for_nrc_other_than_0x78);
    RUN_TEST(test_round_trip_counters_saturate);
    RUN_TEST(test_round_trip_none_while_a_late_answer_can_still_arrive);
    RUN_TEST(test_vehicle_signals_keep_raw_bytes_of_the_validated_answer);
    RUN_TEST(test_vehicle_signals_nrc_of_the_pending_read_then_answer_clears_it);
    RUN_TEST(test_vehicle_signals_mirror_the_skip_cooldown);
    RUN_TEST(test_vehicle_signals_nrc_cleared_by_a_timeout_and_by_the_latch);
    RUN_TEST(test_vehicle_signals_quiet_window_nrc_then_answer_stays_valid);
    RUN_TEST(test_tester_stats_do_not_change_what_is_sent);
    RUN_TEST(test_d068_table_puts_unverified_dids_after_the_verified_ones);
    RUN_TEST(test_d068_unverified_dids_wait_for_the_session_then_are_read_at_once);
    RUN_TEST(test_d068_nrc31_stops_an_unverified_did_until_the_session_is_confirmed_again);
    RUN_TEST(test_d068_never_answered_unverified_did_stops_after_max_timeouts);
    RUN_TEST(test_d068_an_answered_unverified_did_keeps_the_skip_and_retry);
    RUN_TEST(test_d068_nrc31_never_stops_a_verified_did);
    RUN_TEST(test_d068_verified_dids_stay_fresh_while_every_unverified_did_times_out);
    RUN_TEST(test_d068_an_unverified_did_gets_no_0x78_extension);
    RUN_TEST(test_d068_one_nrc31_does_not_stop_an_unverified_did);
    RUN_TEST(test_d068_a_positive_answer_undoes_an_nrc31_stop);
    RUN_TEST(test_d068_session_lost_without_ecu_absence_rearms_the_stopped_dids);
    RUN_TEST(test_d069_engine_start_rearms_the_stopped_dids_once);
    RUN_TEST(test_d069_engine_start_rearm_keeps_the_tier_slot);
    RUN_TEST(test_d069_an_nrc_on_rpm_breaks_the_engine_start_run);
    RUN_TEST(test_d069_long_gap_rearm_all_due_together);
    RUN_TEST(test_d068_arming_across_the_millis_wrap_reads_every_unverified_did_once);
    RUN_TEST(test_d068_nrc31_stop_and_rearm_across_the_millis_wrap);
    RUN_TEST(test_d068_max_timeouts_stop_and_rearm_across_the_millis_wrap);
    RUN_TEST(test_timeout_warnings_are_rate_limited);
    return UNITY_END();
}
