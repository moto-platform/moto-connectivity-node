#include <unity.h>
#include "../../src/HondaCANModule.h"
#include "../mocks/MockCanBus.h"

// [env:native] sets test_build_src = no (most of src/ needs ESP32-only libraries
// that don't exist on the host), so this suite pulls in the one real implementation
// file it needs directly, as a single translation unit -- HondaCANModule.cpp itself
// is completely unmodified for this to work; it only depends on ICanBus + the two
// stubbed Arduino symbols (Serial, millis/delay) already provided for test_native.
#include "../../src/HondaCANModule.cpp"

// G5.1 -- Native tests for HondaCANModule's actual UDS protocol logic (request state
// machine, NRC handling, ISO-TP frame checking, bus-off recovery, ECU-presence
// detection), run against a MockCanBus instead of real TWAI hardware. This is the
// exact same HondaCANModule.cpp/.h that runs on the ESP32 with a TwaiCanBus -- only
// the injected ICanBus differs, so this suite exercises real, unmodified protocol code.

void setUp(void) {}
void tearDown(void) {}

// Builds a positive ReadDataByIdentifier response (SID 0x62) as a real Honda ECU
// would send it: [PCI][0x62][DID_hi][DID_lo][payload...].
static CanFrame makePositiveResponse(uint16_t did, uint8_t d4, uint8_t d5 = 0, uint8_t dlc = 6) {
    CanFrame f;
    f.id = HondaCANModule::UDS_RESP_29BIT;
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
    f.id = HondaCANModule::UDS_RESP_29BIT;
    f.extended = true;
    f.dlc = 4;
    f.data[0] = 0x03;
    f.data[1] = 0x7F;
    f.data[2] = echoedSid;
    f.data[3] = nrc;
    return f;
}

static CanFrame makeSessionConfirm() {
    CanFrame f;
    f.id = HondaCANModule::UDS_RESP_29BIT;
    f.extended = true;
    f.dlc = 2;
    f.data[0] = 0x02;
    f.data[1] = 0x50;
    return f;
}

static CanFrame makeMultiFrameFirstFrame() {
    // ISO-TP First Frame: PCI high nibble 0x1. data[1] is intentionally NOT 0x62/0x7F/
    // 0x50 -- it's part of the multi-frame length field, and must never be parsed as a SID.
    CanFrame f;
    f.id = HondaCANModule::UDS_RESP_29BIT;
    f.extended = true;
    f.dlc = 8;
    f.data[0] = 0x10; // First Frame, length high nibble
    f.data[1] = 0x14; // length low byte -- NOT a SID
    return f;
}

// ---------------------------------------------------------------------------

void test_rpm_decode_updates_state_and_ecu_presence(void) {
    MockCanBus bus;
    HondaCANModule module(bus);
    SystemState state;

    TEST_ASSERT_TRUE(module.begin());

    test_setMillis(100);
    module.update(state); // IDLE -> sends RPM request (highest priority, cadence 50ms) -> WAITING

    // 4500 RPM encoded as raw*4 per HondaCANModule's decode (raw/4.0f == 4500 -> raw=18000=0x4650)
    bus.injectRxFrame(makePositiveResponse(0xF40C, 0x46, 0x50));

    test_setMillis(110);
    module.update(state); // drains the response -> state.engine.rpm updated, WAITING -> COMPLETE -> IDLE

    TEST_ASSERT_EQUAL_FLOAT(4500.0f, state.engine.rpm);
    TEST_ASSERT_EQUAL_UINT32(110, state.engine.rpmUpdatedMs);

    // ecuPresent is derived from _lastGoodResponseMs at the START of update(), so it
    // only flips true on the call AFTER the one that recorded the response.
    TEST_ASSERT_FALSE(state.engine.ecuPresent);
    test_setMillis(120);
    module.update(state);
    TEST_ASSERT_TRUE(state.engine.ecuPresent);
}

void test_session_confirm_is_recognized(void) {
    MockCanBus bus;
    HondaCANModule module(bus);
    SystemState state;
    module.begin();

    bus.injectRxFrame(makeSessionConfirm());
    test_setMillis(100);
    module.update(state);

    // No direct getter for _sessionConfirmed, but a confirmed session must stop the
    // 2-second retry loop from re-sending 0x10 0x03 -- verify no new session request
    // appears well past SESSION_RETRY_INTERVAL_MS (2000ms).
    size_t sessionReqsBefore = 0;
    for (const CanFrame& f : bus.txLog) {
        if (f.dlc >= 3 && f.data[1] == 0x10 && f.data[2] == 0x03) sessionReqsBefore++;
    }

    test_setMillis(5000);
    module.update(state);

    size_t sessionReqsAfter = 0;
    for (const CanFrame& f : bus.txLog) {
        if (f.dlc >= 3 && f.data[1] == 0x10 && f.data[2] == 0x03) sessionReqsAfter++;
    }
    TEST_ASSERT_EQUAL(sessionReqsBefore, sessionReqsAfter);
}

void test_nrc_other_than_pending_resolves_request_without_corrupting_state(void) {
    MockCanBus bus;
    HondaCANModule module(bus);
    SystemState state;
    module.begin();

    test_setMillis(50);
    module.update(state); // RPM request sent, WAITING

    bus.injectRxFrame(makeNegativeResponse(0x22, 0x31)); // requestOutOfRange, not 0x78
    test_setMillis(60);
    module.update(state); // NRC resolves WAITING -> COMPLETE -> IDLE; must not touch engine.rpm

    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm); // untouched, no positive response ever arrived

    int rpmReqsBefore = bus.countDidRequests(0xF40C);
    test_setMillis(120); // 60ms later, well past RPM's 50ms cadence -> should retry promptly, not stay stuck
    module.update(state);
    TEST_ASSERT_TRUE(bus.countDidRequests(0xF40C) > rpmReqsBefore);
}

void test_multiframe_response_is_dropped_not_misparsed(void) {
    MockCanBus bus;
    HondaCANModule module(bus);
    SystemState state;
    module.begin();

    test_setMillis(50);
    module.update(state); // RPM request sent, WAITING

    bus.injectRxFrame(makeMultiFrameFirstFrame());
    test_setMillis(60);
    module.update(state); // must be dropped, not parsed as if data[1] were a SID

    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.engine.rpm);
    TEST_ASSERT_EQUAL_UINT32(0, state.engine.rpmUpdatedMs);
}

void test_bus_off_triggers_recovery_with_backoff(void) {
    MockCanBus bus;
    HondaCANModule module(bus);
    SystemState state;
    module.begin();

    bus.setState(CanBusState::BUS_OFF);

    test_setMillis(1000);
    module.update(state); // enters bus-off, attempts recovery immediately
    TEST_ASSERT_EQUAL(1, bus.initiateRecoveryCallCount);

    test_setMillis(1100); // only 100ms later -- well under the 1000ms initial backoff
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.initiateRecoveryCallCount); // must not retry yet

    // The first attempt immediately doubled _recoveryBackoffMs to 2000ms (from
    // _lastRecoveryAttempt=1000), so the second attempt isn't due until t=3000.
    test_setMillis(2100);
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.initiateRecoveryCallCount); // still not due

    test_setMillis(3100); // past the (now doubled) 2000ms backoff window
    module.update(state);
    TEST_ASSERT_EQUAL(2, bus.initiateRecoveryCallCount);
}

void test_bus_recovery_complete_restarts_driver(void) {
    MockCanBus bus;
    HondaCANModule module(bus);
    SystemState state;
    module.begin();

    bus.setState(CanBusState::BUS_OFF);
    test_setMillis(1000);
    module.update(state);

    bus.setState(CanBusState::STOPPED); // what initiateRecovery() leads to on real hardware
    test_setMillis(1200);
    module.update(state);
    TEST_ASSERT_EQUAL(1, bus.startCallCount);
}

void test_did_skipped_after_max_consecutive_timeouts_then_resumes(void) {
    MockCanBus bus;
    HondaCANModule module(bus);
    SystemState state;
    module.begin();

    // Never inject any response. Drive exactly 5 send/timeout cycles for RPM (highest
    // priority DID, 50ms cadence, 100ms base timeout) by hand, matching the module's
    // real IDLE->WAITING->TIMEOUT->IDLE transitions one call at a time.
    unsigned long t = 50;
    for (int cycle = 0; cycle < 5; cycle++) {
        test_setMillis(t);
        module.update(state); // IDLE -> send RPM request -> WAITING
        t += 151;             // exceed the 100ms base timeout
        test_setMillis(t);
        module.update(state); // WAITING -> TIMEOUT -> IDLE, consecutiveTimeouts++
        t += 1;
    }
    // The 5th timeout should have just triggered a 5s skip for DID 0xF40C.
    int rpmReqsAtSkipStart = bus.countDidRequests(0xF40C);

    // Stay well inside the 5s cooldown and confirm RPM is never re-requested, even
    // though other DID slots keep cycling through their own request/timeout dance.
    for (int i = 0; i < 10; i++) {
        t += 400;
        test_setMillis(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(t < 5809 + 50); // sanity check we're still inside the cooldown window
    TEST_ASSERT_EQUAL(rpmReqsAtSkipStart, bus.countDidRequests(0xF40C));

    // Advance well past the cooldown and confirm RPM gets requested again.
    for (int i = 0; i < 15; i++) {
        t += 400;
        test_setMillis(t);
        module.update(state);
    }
    TEST_ASSERT_TRUE(bus.countDidRequests(0xF40C) > rpmReqsAtSkipStart);
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
    return UNITY_END();
}
