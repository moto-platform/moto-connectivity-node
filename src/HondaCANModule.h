#ifndef HONDA_CAN_MODULE_H
#define HONDA_CAN_MODULE_H

#include "IModule.h"
#include "hal/ICanBus.h"
#include "TesterLatch.h"
#include "TesterStatsTracker.h"
#include "hal/CanRxQueue.h" // also defaults CONN_DISCOVERY_PROBE to 0
#include "vehicle_cl250.h" // generated: external/vehicle-defs/gen/c/conn/
#include "DtcSnapshot.h"
#include "IsoTpReceiver.h"
#if CONN_DISCOVERY_PROBE
#include "DiscoveryScan.h"
#endif

/**
 * @brief UDS tester for the Honda CL250 engine ECU over a generic ICanBus.
 * Sends every request on the 29-bit primary ID and the 11-bit fallback ID.
 *
 * This node is the TEMPORARY sole vehicle-bus tester (D-023). Once vehicle-rt-core polls
 * the ECU, this poller must be disabled: two testers are never allowed (D-021).
 *
 * All IDs, DIDs, formulas and timings come from the generated table
 * (vehicle-defs uds/vehicle_cl250.yaml). Every frame goes through
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
 *
 * D-080: at the start of each stop epoch (key-on) the DtcSnapshot run reads the DTC count
 * and, only if a DTC is present, the list, the freeze frames and the extended data (0x19),
 * one request per step after every due DID. A segmented answer to the 0x19 request in
 * flight gets the one generated FC.CTS through the frame gate; nothing else is sent while
 * it is received. Any other First Frame gets no FC and starts the yaml quiet time.
 *
 * D-059 item 3, only in the probe env (CONN_DISCOVERY_PROBE=1, esp32-s3-devkitc-1-probe):
 * after the listen window and the session start, the generated discovery_scan list runs
 * once (one request in flight, the yaml timeouts), then the DIDs are polled as usual. A
 * segmented answer to the in-flight scan request gets the FC.CTS the same way.
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
    // D-058: step gap and per-DID round-trip statistics since boot (BLE v4). Read-only.
    const TesterStats& testerStats() const { return _tester.stats(); }
    // Log policy of the DID timeout warning: lines per time window (tests assert the bound).
    static uint8_t maxTimeoutLogLinesPerWindow();
    // D-068 item 2: true while the tester no longer asks for this DID (table index) in this
    // stop epoch: an unverified DID the ECU refused with NRC 0x31 / 0x7E / 0x7F or never answered.
    // D-075 item 1 (amended, provisional): why (NONE while not stopped).
    uint8_t didStopReason(uint8_t index) const {
        return index < VEHICLE_CL250_DID_COUNT ? (uint8_t)_dids[index].stopReason : 0;
    }
    bool didStopped(uint8_t index) const {
        return index < VEHICLE_CL250_DID_COUNT && _dids[index].unsupported;
    }
    // True while a segmented answer is being received.
    bool receivingSegmented() const { return _isoRx.active(); }
    // D-080: the DTC freeze-frame reader (read-only view for tests and logs).
    const DtcSnapshot& dtcSnapshot() const { return _dtc; }
#if CONN_DISCOVERY_PROBE
    // D-059: true once the discovery scan ran through its list (normal polling from then on).
    bool discoveryScanDone() const { return _scanPhase == ScanPhase::DONE; }
#endif

#ifdef CONN_NATIVE_TEST
    // Test hook: drives the real TX gate with an arbitrary request (never built on target).
    bool testSendFrame(uint32_t id, const uint8_t* request, uint8_t len) { return sendFrame(id, request, len); }
    void testSetSessionNrcCount(uint8_t index, uint8_t value) { _dids[index].sessionNrcCount = value; }
    DtcSnapshot& testDtc() { return _dtc; }
#if CONN_DISCOVERY_PROBE
    DiscoveryScan& testScan() { return _scan; }
#endif
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

    // Rate limit of the DID timeout warning (log only): lines left in the current window,
    // the window start and the lines left out since.
    uint8_t _timeoutLogBudget = 0;
    unsigned long _timeoutLogWindowStartMs = 0;
    uint16_t _timeoutLogsSuppressed = 0;

    // D-058: step gap (taken at every update() entry) and per-DID round trips. Counters only;
    // they never influence request timing, order, the gate or the latch.
    TesterStatsTracker _tester;

    // ------------------------------------------------------------------
    // G2.2 -- Single-request-in-flight UDS state machine.
    // UDS is a strict request/response protocol: only one DID is ever awaited at a
    // time: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT -> IDLE.
    // ------------------------------------------------------------------
    enum class UdsRequestState : uint8_t { IDLE, REQUEST_SENT, WAITING, COMPLETE, TIMEOUT };

    // D-075 item 1 (amended, provisional): why a DID is stopped. Bookkeeping, no heap.
    enum class StopReason : uint8_t { NONE, NRC_31, NRC_7E_7F, TIMEOUTS, FIRST_FRAME };

    struct DidSlot {
        // Ordered to keep the slot at 24 B on the ESP32 (three words, then the halfword and
        // the single bytes); see the static_assert below.
        unsigned long cadenceMs;      // desired re-request interval for this DID
        unsigned long lastRequestMs;
        unsigned long skipUntilMs;    // temporarily skipped after repeated timeouts
        uint16_t did;
        uint8_t consecutiveTimeouts;
        // D-068 item 2, only for a DID whose support is not verified (yaml `verified: false`):
        // read once when a stop epoch starts, then not asked again until the next epoch if
        // the ECU refuses it (NRC 0x31, 0x7E/0x7F) or never answers it (D-075).
        bool verified;                // vehicle_cl250_did_t.verified
        bool answered;                // a positive answer since the stop epoch started
        bool unsupported;             // stopped until the next stop epoch (D-075)
        StopReason stopReason;        // set with `unsupported`, NONE otherwise
        uint8_t stopNrcCount;         // NRC 0x31/0x7E/0x7F reads since the last positive answer
        // D-075 item 1 (f), provisional: NRC 0x7E/0x7F refusals of an unverified DID in this
        // stop epoch, answered or not, saturating. A positive answer does not reset it; the
        // epoch start and the D-069 re-ask do. nrc31Seen: a 0x31 was counted since the last
        // positive answer (it fixes the stop reason).
        uint8_t sessionNrcCount;
        bool nrc31Seen;
    };
    static_assert(sizeof(unsigned long) != 4 || sizeof(DidSlot) <= 24,
                  "DidSlot must stay at 24 bytes on the 32-bit target (D-075 item 1 (f) review MINOR-4)");


    // Same order as the generated table (table order, not the D-043 poll priority):
    // the unverified D-068 DIDs come after every verified one (codegen checks it), so they
    // delay a verified DID by at most the one read in flight.
    // engine speed (50 ms) is checked first every cycle, so a slow DID's wait can never
    // starve it for more than one response timeout. This temporary tester deliberately
    // ignores vehicle_cl250_did_t.priority; the priority-first scheduler belongs to
    // rt-core (D-043, ISSUES E-4). Changing the order here needs safety-reviewer.
    DidSlot _dids[VEHICLE_CL250_DID_COUNT];

    UdsRequestState _udsState = UdsRequestState::IDLE;
    int8_t _pendingDidIndex = -1;
    uint16_t _pendingDid = 0;
    uint8_t _pendingSid = 0;               // SID of the request in flight (NRC matching)
    unsigned long _requestSentMs = 0;
    unsigned long _requestFirstSentMs = 0; // first send of the pending request (0x78 cap, round trip)
    bool _pendingSawNrc78 = false;         // the pending request got NRC 0x78: no round-trip sample
    bool _pendingRttSkip = false;          // sent in the quiet window after a timeout: no sample
    unsigned long _rttQuietUntilMs = 0;    // end of that window (0 = none), see recordRoundTrip()
    unsigned long _responseTimeoutMs = 0;

    // ------------------------------------------------------------------
    // G2.3 -- Diagnostic session management + ECU-presence detection.
    // The extended session request is retried until a positive 0x50 response is seen,
    // and any UDS response (0x50, 0x62 or a 0x7F NRC) marks the ECU "present" for
    // VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS, exposed via state.engine.ecuPresent.
    // ------------------------------------------------------------------
    bool _sessionConfirmed = false;
    // D-075 (provisional) item 1: the D-068 stop state and the D-069 engine-start flag belong
    // to a stop epoch. One starts at the tester's start and when the session is confirmed
    // after the ECU was lost, not after an NRC 0x7E/0x7F re-confirmation.
    bool _newStopEpoch = true;
    // D-075 item 1 (amended, (f), provisional): set when an NRC 0x7E/0x7F refusal of an
    // unverified DID was counted (answered or not); the next step then requests the session
    // without waiting for kSessionRetryMs. Every other session loss keeps the retry interval.
    bool _sessionRetryNow = false;
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
    // Copies bus/tester counters into state.can and state.tester; read-only, never transmits.
    void publishHealth(SystemState& state);

    // Round-trip sample for the pending request, unless it saw NRC 0x78 or was sent in the
    // quiet window after a timeout (D-058).
    void recordRoundTrip(unsigned long now);

    void storeValue(SystemState& state, uint8_t didIndex, float value, unsigned long now);

    // D-068 item 2 / D-075 item 1: a new stop epoch begins (first confirmation after the start
    // or after the ECU was lost). Every unverified DID is due at once and asked again,
    // whatever the last epoch found.
    void armUnverifiedReads(unsigned long now);
    // D-069 item 3: every unverified DID stopped since the stop epoch started becomes
    // pollable again, due one cadence after its last request (lastRequestMs kept).
    void rearmStoppedReads();
    // D-069 item 3: the first engine start of a stop epoch (kEngineStartSamples
    // consecutive answered RPM samples above 0 after a valid 0) re-arms once. The stop logs
    // stay bounded: each DID logs its stop at most twice per stop epoch.
    void watchEngineStart(float rpm);
    static constexpr uint8_t kEngineStartSamples = 2;
    bool _engineStartRearmed = false;
    bool _rpmZeroSeen = false;
    uint8_t _rpmRunSamples = 0;

    // Saves the latch, then stops the CAN driver and all polling (survives resets
    // other than power-on, see ITesterLatchStore).
    void latchOff(TesterLatchReason reason, const char* why);

    // ------------------------------------------------------------------
    // D-080 -- DTC freeze frames, and the segmented reception of their answers (D-059 FC).
    // ------------------------------------------------------------------
    DtcSnapshot _dtc;
    bool _pendingDtc = false;            // the request in flight is the DtcSnapshot's
    // Round-2 MAJOR-A/B: the response ID that answered the run's count; the rest of the run
    // goes to its paired request ID only and only its answers count.
    bool _dtcLocked = false;
    uint32_t _dtcResponseId = 0;
    bool _dtcResponseExtended = false;
    uint32_t _dtcRequestId = 0;
    // True if a 0x19 answer frame may resolve the request (and locks the run to its ID).
    bool acceptDtcAnswerFrom(const CanFrame& frame);
    void lockDtcTo(const CanFrame& frame, uint32_t requestId);
    IsoTpReceiver _isoRx;
    bool _fcSentForPending = false;      // one FC per request (BS 0)
    // An accepted First Frame waits here until the drain is complete (Q-018: nothing is
    // sent before the whole queue was checked for a foreign tester); then its FC goes out.
    bool _fcPending = false;
    CanFrame _fcPendingFrame;
    unsigned long _fcPendingMs = 0;
    uint32_t _requestLostBaseline = 0;   // driver lost count when the 0x19/scan request went out
    unsigned long _txQuietUntilMs = 0;   // no request at all until then (yaml flow_control rule)

    // D-080 item 1: the warm-idle trigger, the generated D-079 W3 conditions over the last
    // valid, non-stale DID values (vehicle_cl250_dtc_warm_idle[]). Bookkeeping only.
    float _didValue[VEHICLE_CL250_DID_COUNT] = {0.0f};
    unsigned long _didValueMs[VEHICLE_CL250_DID_COUNT] = {0}; // 0 = no value in this epoch
    unsigned long _warmIdleSinceMs = 0;                       // 0 = the conditions do not hold
    bool warmIdleConditionsMet(unsigned long now) const;
    void watchWarmIdle(unsigned long now);
    // Sends the DtcSnapshot's next request (one in flight, no NRC 0x78 extension).
    void sendDtcRequest(unsigned long now);
    // A Single Frame answer (positive or NRC) to the 0x19 request in flight. False if it is
    // not one (then the normal handling runs).
    bool onDtcSingleFrame(const CanFrame& frame, unsigned long now);
    // True while the request in flight may be answered with a segmented message (0x19, scan).
    bool segmentedRequestPending() const;
    // `logBudget`: the drain's per-pass log lines (a flood of First Frames, safety-reviewer
    // MINOR-1 on conn D-080).
    void onFirstFrame(const CanFrame& frame, unsigned long now, uint8_t& logBudget);
    // MAJOR-2: a 0x19 transaction (request to last frame) longer than one unverified-read slot
    // turns 0x19 off for the epoch, after its answer is logged.
    void checkDtcSlot(unsigned long now);
    void onConsecutiveFrame(const CanFrame& frame, unsigned long now);
    // The reception completed or was aborted: resolve the 0x19 or scan request.
    void endReception(unsigned long now);
    // The segmented answer of the request in flight failed before or during its reception.
    void failSegmented(const char* why, uint16_t received, uint16_t announced, unsigned long now);
    // Sends the generated FC.CTS on the request ID paired with the First Frame's response
    // ID, through the same checks as every request. False if refused or not sent.
    bool sendFlowControl(const CanFrame& firstFrame);
    // After a complete drain: sends the pending FC (or aborts the request) and starts the
    // reception. Called only when the drain returned DRAINED.
    void sendPendingFlowControl(unsigned long now);
    void holdTx(unsigned long now);

#if CONN_DISCOVERY_PROBE
    // ------------------------------------------------------------------
    // D-059 item 3 -- one-shot discovery scan (probe env only).
    // ------------------------------------------------------------------
    enum class ScanPhase : uint8_t { WAIT_SESSION, RUNNING, DONE };
    ScanPhase _scanPhase = ScanPhase::WAIT_SESSION;
    DiscoveryScan _scan;
    int8_t _pendingScanIndex = -1;       // scan entry in flight, -1 = none (or a DID)
    unsigned long _firstSessionRequestMs = 0;
    unsigned long _scanHoldUntilMs = 0;  // no scan request until then (a late answer may come)

    // Starts the scan once the session is confirmed (or one retry interval after the first
    // session request) and sends the next entry; marks the scan done at the end of the list.
    void scanStep(unsigned long now);
    // A Single Frame answer to the scan entry in flight. False if it is not one (then the
    // normal handling runs, e.g. for a session confirm or an NRC 0x78).
    bool onScanSingleFrame(const CanFrame& frame, unsigned long now);
#endif
};

#endif // HONDA_CAN_MODULE_H
