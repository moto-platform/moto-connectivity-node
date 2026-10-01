#include <unity.h>
#include "../../src/HondaCANModule.h"
#include "../mocks/MockCanBus.h"
#include "../mocks/MockTesterLatchStore.h"
#include "../../src/IsoTpCan.h"
#include "uds_iso14229.h" // generated (gen/c/conn/, D-040)
#include <vector>

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

void setUp(void) {}
void tearDown(void) {}

// Q-018: begin() only starts the listen-only window, so a test's own timeline starts
// when the window is over. setClock(t) is relative to that moment: setClock(0) is the
// first normal-mode pass, which sends the session request (what begin() did before the
// window existed).
static const unsigned long kT0 = 5000 + kListenOnlyWindowMs + 10;

static void setClock(unsigned long relativeMs) {
    test_setMillis(kT0 + relativeMs);
}

static bool startPolling(HondaCANModule& module, SystemState& state) {
    test_setMillis(kT0 - kListenOnlyWindowMs - 10);
    if (!module.begin()) return false;
    module.update(state); // first pass: the listen-only window starts
    test_setMillis(kT0 - 10);
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
    TEST_ASSERT_EQUAL_UINT32(kT0 + 110, state.engine.rpmUpdatedMs);

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

    // Never inject any response. Drive exactly 5 send/timeout cycles for RPM (highest
    // priority DID, 50ms cadence, 100ms base timeout) by hand, matching the module's
    // real IDLE->WAITING->TIMEOUT->IDLE transitions one call at a time.
    unsigned long t = 50;
    for (int cycle = 0; cycle < 5; cycle++) {
        setClock(t);
        module.update(state); // IDLE -> send RPM request -> WAITING
        t += 151;             // exceed the 100ms base timeout
        setClock(t);
        module.update(state); // WAITING -> TIMEOUT -> IDLE, consecutiveTimeouts++
        t += 1;
    }
    // The 5th timeout should have just triggered a 5s skip for DID VEHICLE_CL250_DID_ENGINE_SPEED.
    int rpmReqsAtSkipStart = bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED);

    // Stay well inside the 5s cooldown and confirm RPM is never re-requested, even
    // though other DID slots keep cycling through their own request/timeout dance.
    for (int i = 0; i < 10; i++) {
        t += 400;
        setClock(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(t < 5809 + 50); // sanity check we're still inside the cooldown window
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
    setClock(100);
    module.update(state); // engine speed request goes out
    TEST_ASSERT_EQUAL_UINT32(2, (uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED)); // primary + fallback
    // The ECU answers 0x78 forever; the request must still end within the generated maximum.
    unsigned long t = 100;
    for (; t < 100 + VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS + 400; t += 20) {
        setClock(t);
        bus.injectRxFrame(makeNegativeResponse(UDS_SID_READ_DATA_BY_IDENTIFIER, UDS_NRC_RESPONSE_PENDING));
        module.update(state);
        if ((uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > 2) break;
    }
    TEST_ASSERT_TRUE((uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED) > 2);
    TEST_ASSERT_TRUE(t <= 100 + VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS + VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + 100);
}

void test_nrc_for_other_service_does_not_resolve_pending_read(void) {
    MockCanBus bus;
    MockTesterLatchStore latchStore;
    HondaCANModule module(bus, latchStore);
    SystemState state;
    startPolling(module, state);
    bus.clearTxLog();
    setClock(100);
    module.update(state); // engine speed request pending
    TEST_ASSERT_EQUAL_UINT32(2, (uint32_t)bus.countDidRequests(VEHICLE_CL250_DID_ENGINE_SPEED));
    size_t before = bus.txLog.size();
    // NRC for TesterPresent must not end the pending ReadDataByIdentifier early. Without
    // the fix the request completes and the next DID is requested at once.
    setClock(110);
    bus.injectRxFrame(makeNegativeResponse(VEHICLE_CL250_TESTER_PRESENT_SID, 0x12));
    module.update(state);
    setClock(120);
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
    unsigned long t = 50;
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

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_rpm_decode_updates_state_and_ecu_presence);
    RUN_TEST(test_session_confirm_is_recognized);
    RUN_TEST(test_nrc_other_than_pending_resolves_request_without_corrupting_state);
    RUN_TEST(test_multiframe_response_is_dropped_not_misparsed);
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
    return UNITY_END();
}
