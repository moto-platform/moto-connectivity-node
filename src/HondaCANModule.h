#ifndef HONDA_CAN_MODULE_H
#define HONDA_CAN_MODULE_H

#include "IModule.h"
#include "hal/ICanBus.h"
#include "TesterLatch.h"
#include "vehicle_cl250.h" // generated: external/moto-vehicle-defs/gen/c/conn/

/**
 * @brief UDS tester for the Honda CL250 engine ECU over a generic ICanBus.
 * Sends every request on the 29-bit primary ID and the 11-bit fallback ID.
 *
 * This node is the TEMPORARY sole vehicle-bus tester (D-023). Once moto-rt-core polls
 * the ECU, this poller must be disabled: two testers are never allowed (D-021).
 *
 * All IDs, DIDs, formulas and timings come from the generated table
 * (moto-vehicle-defs uds/vehicle_cl250.yaml). Every frame goes through
 * vehicle_cl250_frame_allowed() before it is sent, so only the D-020 service allow-list
 * can ever reach the vehicle bus.
 *
 * G3.2 -- depends only on ICanBus, never on driver/twai.h: main.cpp injects a
 * TwaiCanBus on real hardware and test/test_can_protocol injects a MockCanBus, so
 * this exact protocol logic (request state machine, NRC handling, ISO-TP frame
 * checking, bus-off recovery) runs identically either way and is unit-testable
 * with no ESP32 device attached.
 *
 * Q-018 hardening of the D-030 safeguards:
 *  - The latch (foreign tester / repeated bus-off) and the bus-off count are saved to an
 *    ITesterLatchStore and restored by begin(), so a watchdog/panic/brownout reset cannot
 *    re-enable the poller or reset the bus-off budget. An invalid record after a
 *    non-power-on reset counts as latched. A restored latch means the CAN driver is
 *    never installed.
 *  - begin() starts the bus listen-only. Nothing is sent until kListenOnlyWindowMs of
 *    complete reception (no lost frames, queue drained) passed with no diagnostic
 *    request on the bus and no ECU answer (we asked nothing yet, so an answer means
 *    someone else did).
 *  - Every pass drains the RX queue first, before any transmission and before the
 *    response-timeout check, so a slow pass cannot count a queued answer as a timeout.
 *    If the bounded drain cannot empty the queue, the pass transmits nothing.
 */
class HondaCANModule : public IProducerModule {
public:
    HondaCANModule(ICanBus& bus, ITesterLatchStore& latchStore);
    bool begin() override;
    void update(SystemState& state) override;
    bool isHealthy() const override { return _initialized; }

    // Frames refused by the D-020 guard since boot. Must stay 0; exposed for tests/logs.
    uint32_t blockedFrameCount() const { return _blockedFrames; }
    // True once the poller stopped for good (foreign tester or repeated bus-off), in
    // this boot or restored from before the last reset.
    bool latchedOff() const { return _latchedOff; }
    // True during the listen-only observation window after begin().
    bool listening() const { return _listening; }
    TesterLatchReason latchReason() const { return _latchReason; }
    bool foreignTesterDetected() const { return _latchReason == TesterLatchReason::FOREIGN_TESTER; }
    // DID requests that timed out with no response at all since boot (BLE v3 health).
    uint32_t unansweredDidCount() const { return _unansweredDidCount; }

#ifdef CONN_NATIVE_TEST
    // Test hook: drives the real TX gate with an arbitrary request (never built on target).
    bool testSendFrame(uint32_t id, const uint8_t* request, uint8_t len) { return sendFrame(id, request, len); }
#endif

private:
    ICanBus& _bus;
    ITesterLatchStore& _latchStore;
    bool _initialized = false;
    bool _listening = false;
    bool _listenStarted = false;       // the window starts at the first update(), see listenStep()
    unsigned long _listenStartMs = 0;
    uint32_t _rxLostBaseline = 0;      // driver counts lost frames from install, i.e. from 0
    uint32_t _listenRestarts = 0;
    bool _sessionStartPending = false; // normal mode entered; session goes out after a drain
    unsigned long _lastKeepAlive = 0;
    uint32_t _blockedFrames = 0;
    bool _latchedOff = false;
    TesterLatchReason _latchReason = TesterLatchReason::NONE;

    // G1.3 -- Bus-off recovery state. Recovery attempts back off exponentially
    // (VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS doubling up to _MAX_MS) instead of
    // hammering initiateRecovery() every loop pass while the bus stays off.
    bool _busOff = false;
    unsigned long _lastRecoveryAttempt = 0;
    unsigned long _recoveryBackoffMs = VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS;
    uint32_t _busOffEventCount = 0;

    // G2.1 -- Negative response (NRC, 0x7F) bookkeeping.
    uint32_t _nrcCount = 0;
    uint32_t _unansweredDidCount = 0;

    // ------------------------------------------------------------------
    // G2.2 -- Single-request-in-flight UDS state machine.
    // UDS is a strict request/response protocol: only one DID is ever awaited at a
    // time: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT -> IDLE.
    // ------------------------------------------------------------------
    enum class UdsRequestState : uint8_t { IDLE, REQUEST_SENT, WAITING, COMPLETE, TIMEOUT };

    struct DidSlot {
        uint16_t did;
        unsigned long cadenceMs;      // desired re-request interval for this DID
        unsigned long lastRequestMs;
        uint8_t consecutiveTimeouts;
        unsigned long skipUntilMs;    // temporarily skipped after repeated timeouts
    };

    // Same order as the generated table, which is priority ordered: engine speed (50 ms)
    // is checked first every cycle, so a slow DID's wait can never starve it for more
    // than one response timeout.
    DidSlot _dids[VEHICLE_CL250_DID_COUNT];

    UdsRequestState _udsState = UdsRequestState::IDLE;
    int8_t _pendingDidIndex = -1;
    uint16_t _pendingDid = 0;
    unsigned long _requestSentMs = 0;
    unsigned long _requestFirstSentMs = 0; // first send of the pending request (0x78 cap)
    unsigned long _responseTimeoutMs = 0;

    // ------------------------------------------------------------------
    // G2.3 -- Diagnostic session management + ECU-presence detection.
    // The extended session request is retried until a positive 0x50 response is seen,
    // and any UDS response (0x50, 0x62 or a 0x7F NRC) marks the ECU "present" for
    // VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS, exposed via state.engine.ecuPresent.
    // ------------------------------------------------------------------
    bool _sessionConfirmed = false;
    unsigned long _lastSessionAttemptMs = 0;

    unsigned long _lastGoodResponseMs = 0;
    bool _ecuPresent = false; // mirrors state.engine.ecuPresent; logged only on change

    /**
     * @brief Sends one UDS request (SID first) as an ISO-TP single frame on the primary
     * and the fallback request ID. Frames the D-020 guard refuses are never sent.
     */
    void sendRequest(const uint8_t* request, uint8_t len);
    bool sendFrame(uint32_t id, const uint8_t* request, uint8_t len);

    /**
     * @brief Transmits a ReadDataByIdentifier ($22) request for a specific DID.
     */
    void requestDID(uint16_t did);

    // The UDS poller itself.
    void poll(SystemState& state);
    // Listen-only window: watch for a foreign tester, then switch to normal mode and
    // start the session. Returns without transmitting while the window is open.
    void listenStep(SystemState& state, unsigned long now);
    enum class DrainResult : uint8_t { DRAINED, BACKLOG, LATCHED };
    // Drains the RX queue, at most kMaxRxFramesPerPass frames. BACKLOG = frames may still
    // be queued; LATCHED = a foreign tester latched the poller off.
    DrainResult drainRx(SystemState& state, unsigned long now);
    // A diagnostic request that this node did not send (TWAI never receives its own).
    static bool isForeignTesterFrame(const CanFrame& frame);
    // Copies bus/tester counters into state.can; read-only, never transmits.
    void publishHealth(SystemState& state);

    void storeValue(SystemState& state, uint8_t didIndex, float value, unsigned long now);

    // Saves the latch, then stops the CAN driver and all polling (survives resets
    // other than power-on, see ITesterLatchStore).
    void latchOff(TesterLatchReason reason, const char* why);
};

#endif // HONDA_CAN_MODULE_H
