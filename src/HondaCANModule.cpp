#include "HondaCANModule.h"
#include "IsoTpCan.h"
#include "uds_iso14229.h" // generated: D-040 client subset (gen/c/conn/)

namespace {
// Typed copies of the generated timing macros, so min()/comparisons see one type.
const unsigned long kTesterPresentPeriodMs = VEHICLE_CL250_TESTER_PRESENT_PERIOD_MS;
const unsigned long kSessionRetryMs = VEHICLE_CL250_SESSION_RETRY_INTERVAL_MS;
const unsigned long kResponseTimeoutMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS;
const unsigned long kResponsePendingMaxMs = VEHICLE_CL250_RESPONSE_TIMEOUT_MAX_MS;
const uint8_t kMaxConsecutiveTimeouts = VEHICLE_CL250_MAX_CONSECUTIVE_TIMEOUTS;
const unsigned long kDidSkipCooldownMs = VEHICLE_CL250_DID_SKIP_COOLDOWN_MS;
const unsigned long kEcuAbsentTimeoutMs = VEHICLE_CL250_ECU_ABSENT_TIMEOUT_MS;
const unsigned long kBackoffMinMs = VEHICLE_CL250_BUS_OFF_BACKOFF_INITIAL_MS;
const unsigned long kBackoffMaxMs = VEHICLE_CL250_BUS_OFF_BACKOFF_MAX_MS;
// Response IDs of the ECU; `out` = the paired request ID. False for any other frame.
bool pairedRequestId(const CanFrame& f, uint32_t& out) {
    if (f.id == VEHICLE_CL250_RESPONSE_ID && f.extended == isotp::isExtendedId(VEHICLE_CL250_RESPONSE_ID)) {
        out = VEHICLE_CL250_REQUEST_ID;
        return true;
    }
    if (f.id == VEHICLE_CL250_FALLBACK_RESPONSE_ID &&
        f.extended == isotp::isExtendedId(VEHICLE_CL250_FALLBACK_RESPONSE_ID)) {
        out = VEHICLE_CL250_FALLBACK_REQUEST_ID;
        return true;
    }
    return false;
}

// D-080 item 4 / D-068 item 1: a 0x19 transaction may hold the slot as long as an unverified
// DID read (base timeout + one step); longer = the epoch's one 0x19 fault (MAJOR-2).
const unsigned long kDtcSlotMaxMs = VEHICLE_CL250_RESPONSE_TIMEOUT_BASE_MS + VEHICLE_CL250_CLIENT_STEP_MAX_MS;

// Local policy of this TEMPORARY tester, not a vehicle definition: after this many
// bus-off events the driver is stopped for good (until reboot) instead of rejoining the
// OEM bus forever and putting error frames on it. Candidate for moto-vehicle-defs.
const uint32_t kMaxBusOffEvents = 5;

// D-068 item 2 (safety-reviewer MAJOR-2): NRC 0x31 on this many separate reads, with no
// positive answer between them, stops an unverified DID. Local policy of this tester.
const uint8_t kStopNrcReads = 2;

// Local policy of this TEMPORARY tester (Q-018): listen-only for this long after begin()
// before the first request, so a tester that is already polling the ECU is seen before
// this node ever transmits (D-021). Candidate for moto-vehicle-defs, like the one above.
const unsigned long kListenOnlyWindowMs = 2000;

// Bounded RX drain: at most this many frames per pass. Twice the TWAI RX queue depth
// (hal/CanRxQueue.h), so one pass empties the queue (also a whole CF burst after the FC),
// while a flooded bus still cannot stall the loop.
static_assert(2u * kCanRxQueueLen <= 255u, "the drain bound must fit uint8_t");
const uint8_t kMaxRxFramesPerPass = (uint8_t)(2u * kCanRxQueueLen);
// At most this many per-frame log lines per pass (Serial output is slow; see TWDT).
const uint8_t kMaxRxLogLinesPerPass = 4;
// DID timeout warnings: at most one timeout happens per pass, so they are bounded per time
// window instead. A log policy of this temporary node, not a vehicle value (Serial is slow,
// see TWDT).
const unsigned long kTimeoutLogWindowMs = 1000;
const uint8_t kMaxTimeoutLogLinesPerWindow = kMaxRxLogLinesPerPass;

// Per-pass log budget: true (and one slot used) while the budget lasts.
inline bool takeLogSlot(uint8_t& budget) {
    if (budget == 0) {
        return false;
    }
    budget--;
    return true;
}

const char* latchReasonText(TesterLatchReason reason) {
    switch (reason) {
        case TesterLatchReason::FOREIGN_TESTER: return "another tester on the bus";
        case TesterLatchReason::BUS_OFF: return "too many bus-off events";
        case TesterLatchReason::UNKNOWN: return "latch record invalid after a reset";
        default: return "not latched";
    }
}

// NRC name for logs only. Only the NRCs the generated client subset names (D-040);
// the log line always carries the raw code as well.
const char* nrcName(uint8_t nrc) {
    switch (nrc) {
        case UDS_NRC_RESPONSE_PENDING: return "responsePending";
        case UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION: return "subFunctionNotSupportedInActiveSession";
        case UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION: return "serviceNotSupportedInActiveSession";
        case UDS_NRC_REQUEST_OUT_OF_RANGE: return "requestOutOfRange";
        default: return "other";
    }
}

// millis()-wrap-safe "deadline not reached yet" (deadline 0 = none).
inline bool beforeDeadline(unsigned long now, unsigned long deadline) {
    return deadline != 0 && (long)(deadline - now) > 0;
}

const uint8_t kSidSessionResponse = VEHICLE_CL250_SESSION_POSITIVE_SID;

const uint8_t kSessionRequest[] = {VEHICLE_CL250_SESSION_SID, VEHICLE_CL250_SESSION_SUBFUNCTION};
const uint8_t kTesterPresentRequest[] = {VEHICLE_CL250_TESTER_PRESENT_SID,
                                         VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION};

// D-059 scan and D-080 DTC report lines go to USB serial.
void reportSerialLine(const char* line) {
    Serial.println(line);
}
} // namespace

uint8_t HondaCANModule::maxTimeoutLogLinesPerWindow() {
    return kMaxTimeoutLogLinesPerWindow;
}

HondaCANModule::HondaCANModule(ICanBus& bus, ITesterLatchStore& latchStore)
    : _bus(bus), _latchStore(latchStore), _dtc(reportSerialLine)
#if CONN_DISCOVERY_PROBE
    , _scan(reportSerialLine)
#endif
{
    _timeoutLogBudget = kMaxTimeoutLogLinesPerWindow; // the first window is open from boot
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        _dids[i].did = vehicle_cl250_dids[i].did;
        _dids[i].cadenceMs = vehicle_cl250_dids[i].poll_period_ms;
        _dids[i].lastRequestMs = 0;
        _dids[i].consecutiveTimeouts = 0;
        _dids[i].skipUntilMs = 0;
        _dids[i].verified = vehicle_cl250_dids[i].verified;
        _dids[i].answered = false;
        _dids[i].unsupported = false;
        _dids[i].stopReason = StopReason::NONE;
        _dids[i].stopNrcCount = 0;
        _dids[i].sessionNrcCount = 0;
        _dids[i].nrc31Seen = false;
    }
}

void HondaCANModule::rearmStoppedReads() {
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        DidSlot& slot = _dids[i];
        if (slot.verified) {
            continue;
        }
        // D-069 item 3: the stop rule starts again from zero. lastRequestMs is kept, so the
        // DID is due one cadence after its last request (never above its tier rate, D-068
        // budget unchanged). The NRC counts of every unverified DID, stopped or not, are
        // cleared (rt-core#42 safety review round 3 MINOR-1): a count left from before the
        // engine start would stop it on one refusal.
        slot.stopNrcCount = 0;
        slot.sessionNrcCount = 0;
        slot.nrc31Seen = false;
        if (!slot.unsupported) {
            continue;
        }
        slot.unsupported = false;
        slot.stopReason = StopReason::NONE;
        slot.consecutiveTimeouts = 0;
    }
}

void HondaCANModule::watchEngineStart(float rpm) {
    if (rpm <= 0.0f) {
        _rpmZeroSeen = true;
        _rpmRunSamples = 0;
        return;
    }
    if (!_rpmZeroSeen || _engineStartRearmed) {
        return;
    }
    if (++_rpmRunSamples >= kEngineStartSamples) {
        _engineStartRearmed = true; // once per stop epoch, not on every stall/restart
        rearmStoppedReads();
    }
}

// D-075 (provisional) item 1: starts a stop epoch. Called only at the first session
// confirmation after the tester's start or after the ECU was lost, never for a
// re-confirmation after NRC 0x7E/0x7F.
void HondaCANModule::armUnverifiedReads(unsigned long now) {
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        DidSlot& slot = _dids[i];
        if (slot.verified) {
            continue; // the D-050..D-053 schedule of a verified DID is unchanged
        }
        slot.answered = false;
        slot.unsupported = false;
        slot.stopReason = StopReason::NONE;
        slot.stopNrcCount = 0;
        slot.sessionNrcCount = 0;
        slot.nrc31Seen = false;
        slot.consecutiveTimeouts = 0;
        slot.skipUntilMs = 0;
        slot.lastRequestMs = now - slot.cadenceMs; // due at once (unsigned, wrap-safe)
    }
    // D-069 item 3: a new stop epoch may re-arm once more at its first engine start
    // (_rpmZeroSeen and the sample run belong to the epoch too, D-075 item 1 (b)).
    _engineStartRearmed = false;
    _rpmZeroSeen = false;
    _rpmRunSamples = 0;
    // D-080 item 1: the key-on run of the epoch (also its "not served" and skip state are
    // forgotten: they hold until the next stop epoch). The warm-idle trigger starts over.
    _dtc.startEpoch();
    _warmIdleSinceMs = 0;
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        _didValueMs[i] = 0;
    }
}

bool HondaCANModule::warmIdleConditionsMet(unsigned long now) const {
    for (uint8_t k = 0; k < VEHICLE_CL250_DTC_WARM_IDLE_CONDITION_COUNT; k++) {
        const vehicle_cl250_dtc_condition_t& c = vehicle_cl250_dtc_warm_idle[k];
        if (c.did_index >= VEHICLE_CL250_DID_COUNT || _didValueMs[c.did_index] == 0 ||
            now - _didValueMs[c.did_index] > (unsigned long)vehicle_cl250_dids[c.did_index].stale_after_ms) {
            return false; // no value, or a stale one: not met
        }
        const float v = _didValue[c.did_index];
        bool met = false;
        switch (c.op) {
            case VEHICLE_CL250_DTC_OP_LT: met = v < c.value; break;
            case VEHICLE_CL250_DTC_OP_LE: met = v <= c.value; break;
            case VEHICLE_CL250_DTC_OP_GT: met = v > c.value; break;
            case VEHICLE_CL250_DTC_OP_GE: met = v >= c.value; break;
            case VEHICLE_CL250_DTC_OP_EQ: met = v == c.value; break; // NOLINT: exact, DIDs decode exact steps
            default: met = false; break;
        }
        if (!met) {
            return false;
        }
    }
    return true;
}

void HondaCANModule::watchWarmIdle(unsigned long now) {
    if (!warmIdleConditionsMet(now)) {
        _warmIdleSinceMs = 0;
        return;
    }
    if (_warmIdleSinceMs == 0) {
        _warmIdleSinceMs = (now == 0) ? 1 : now; // 0 means "not holding"
        return;
    }
    if (now - _warmIdleSinceMs >= (unsigned long)VEHICLE_CL250_DTC_WARM_IDLE_MIN_DURATION_MS) {
        _dtc.armWarmIdle(); // once per stop epoch (DtcSnapshot keeps the flag)
    }
}

bool HondaCANModule::begin() {
    // Q-018: a latch from before the last reset stays in force (an invalid record after a
    // non-power-on reset counts as one), and so does the bus-off budget. A latched node
    // never installs the driver: it neither transmits nor acknowledges on the vehicle bus.
    TesterLatchStatus restored = _latchStore.load();
    _busOffEventCount = restored.busOffCount;
    if (restored.reason == TesterLatchReason::NONE && _busOffEventCount >= kMaxBusOffEvents) {
        restored.reason = TesterLatchReason::BUS_OFF; // defensive: the budget is already spent
        _latchStore.save(restored.reason, _busOffEventCount);
    }
    if (restored.reason != TesterLatchReason::NONE) {
        _latchedOff = true;
        _latchReason = restored.reason;
        _bus.stop();
        Serial.printf("[UDS STOP] Vehicle-bus poller stays latched off from before the last reset (%s). Power-cycle the node to re-enable it.\n",
            latchReasonText(_latchReason));
        _initialized = true; // healthy: update() keeps publishing the latch in CAN health
        return true;
    }

    if (_bus.beginListenOnly()) {
        // Nothing is sent yet: the window starts at the first update() (listenStep()).
        _listening = true;
        Serial.printf("[CAN SUCCESS] CAN Bus Active (500 kbps), listen-only for %lu ms before the first request (bus-off events so far: %lu)...\n",
            kListenOnlyWindowMs, (unsigned long)_busOffEventCount);
        _initialized = true;
        return true;
    }
    _bus.stop();
    Serial.println("[CAN ERROR] Failed to initialize CAN bus driver! Check TX/RX pins.");
    _initialized = false;
    return false;
}

bool HondaCANModule::sendFrame(uint32_t id, const uint8_t* request, uint8_t len) {
    CanFrame frame;
    frame.extended = isotp::isExtendedId(id);
    frame.id = id;
    frame.dlc = VEHICLE_CL250_FRAME_DLC;
    frame.data[0] = len; // ISO-TP single-frame PCI
    for (uint8_t i = 1; i < VEHICLE_CL250_FRAME_DLC; i++) {
        frame.data[i] = (i - 1 < len) ? request[i - 1] : VEHICLE_CL250_PADDING_BYTE;
    }

    // D-021: only the ECU's own request IDs; D-020: only allow-listed services, Single
    // Frames only. The generated guard is the last word on what may reach the vehicle bus.
    bool idAllowed = (id == VEHICLE_CL250_REQUEST_ID) || (id == VEHICLE_CL250_FALLBACK_REQUEST_ID);
    if (!idAllowed || len == 0 || len > isotp::kSingleFrameMaxLen ||
        !vehicle_cl250_frame_allowed(frame.data, frame.dlc)) {
        _blockedFrames++;
        Serial.printf("[UDS BLOCKED] Frame 0x%08lX [%02X %02X %02X] refused by the D-020 guard (total=%lu).\n",
            (unsigned long)frame.id, frame.data[0], frame.data[1], frame.data[2], (unsigned long)_blockedFrames);
        return false;
    }
    return _bus.transmit(frame);
}

void HondaCANModule::sendRequest(const uint8_t* request, uint8_t len) {
    sendFrame(VEHICLE_CL250_REQUEST_ID, request, len);
    sendFrame(VEHICLE_CL250_FALLBACK_REQUEST_ID, request, len);
}

void HondaCANModule::requestDID(uint16_t did) {
    const uint8_t request[] = {UDS_SID_READ_DATA_BY_IDENTIFIER, (uint8_t)((did >> 8) & 0xFF), (uint8_t)(did & 0xFF)};
    sendRequest(request, sizeof(request));
}

void HondaCANModule::storeValue(SystemState& state, uint8_t didIndex, float value, unsigned long now) {
    switch (didIndex) {
        case VEHICLE_CL250_IDX_ENGINE_SPEED:
            state.engine.rpm = value;
            state.engine.rpmUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_VEHICLE_SPEED:
            state.engine.speed = (uint8_t)value;
            state.engine.speedUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_COOLANT_TEMP:
            state.engine.coolantTemp = (int16_t)value;
            state.engine.coolantTempUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_THROTTLE_POS:
            state.engine.throttlePos = value;
            state.engine.throttlePosUpdatedMs = now;
            break;
        case VEHICLE_CL250_IDX_BATTERY_VOLTAGE:
            state.engine.batteryVoltage = value;
            state.engine.batteryVoltageUpdatedMs = now;
            break;
        default:
            break;
    }
}

void HondaCANModule::update(SystemState& state) {
    // D-058 item 2: every entry counts, also in the listen window and when latched (the step
    // still runs). micros() wraps at 2^32 us; TesterStatsTracker subtracts wrap-safely.
    _tester.onStepEntry((uint32_t)micros());
    poll(state);
    publishHealth(state);
}

// Read-only: mirrors bus/tester counters into SystemState for BLE telemetry v3. Never
// transmits and never changes the poller state; once latched off the driver is gone,
// so the counters stay frozen at their last values.
void HondaCANModule::publishHealth(SystemState& state) {
    state.tester = _tester.stats();
    state.tester.available = true;
    CanHealth& h = state.can;
    h.flags = CAN_HEALTH_FLAG_POLLER_ENABLED;
    if (_latchReason == TesterLatchReason::FOREIGN_TESTER || _latchReason == TesterLatchReason::UNKNOWN) {
        h.flags |= CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER;
    }
    if (_latchReason == TesterLatchReason::BUS_OFF || _latchReason == TesterLatchReason::UNKNOWN) {
        h.flags |= CAN_HEALTH_FLAG_LATCHED_BUS_OFF; // both bits = latched, cause unknown
    }
    h.busOffCount = _busOffEventCount;
    h.unansweredDidCount = _unansweredDidCount;
    // D-066 item 4: which DIDs sit in their timeout cool-down (read-only mirror). Once
    // latched off nothing is read any more: no DID is skipped and no NRC is current, so the
    // values only age into STALE (safety-reviewer M1).
    const unsigned long now = millis();
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        VehicleSignalRecord& r = state.vehicleSignals.dids[i];
        // D-068: a DID stopped after timeouts shows as skipped; one stopped by NRC 0x31 keeps
        // showing that NRC (its last answer).
        r.skipped = !_latchedOff && (beforeDeadline(now, _dids[i].skipUntilMs) ||
                                     (_dids[i].unsupported && !r.lastWasNrc));
        if (_latchedOff) {
            r.lastWasNrc = false;
        }
    }
    if (_latchedOff) {
        h.busState = CanHealthState::STOPPED;
        return;
    }
    uint16_t txErr = 0, rxErr = 0;
    _bus.getErrorCounters(txErr, rxErr);
    h.txErrorCount = txErr;
    h.rxErrorCount = rxErr;
    switch (_bus.getState()) {
        case CanBusState::BUS_OFF: h.busState = CanHealthState::BUS_OFF; break;
        case CanBusState::STOPPED: h.busState = CanHealthState::STOPPED; break;
        case CanBusState::ERROR_WARNING: h.busState = CanHealthState::ERROR_WARNING; break;
        case CanBusState::RUNNING:
        default:
            h.busState = (txErr >= CAN_ERROR_WARNING_LIMIT || rxErr >= CAN_ERROR_WARNING_LIMIT)
                ? CanHealthState::ERROR_WARNING : CanHealthState::RUNNING;
            break;
    }
}

void HondaCANModule::poll(SystemState& state) {
    unsigned long now = millis();

    // Latched off (foreign tester seen or too many bus-offs): transmit nothing ever
    // again until power-on, and stop feeding SystemState so consumers show "stale".
    if (_latchedOff) {
        state.engine.ecuPresent = false;
        return;
    }

    if (_listening) {
        listenStep(state, now);
        return;
    }

    // Q-018: drain RX first -- before anything below transmits (a foreign tester in the
    // queue latches us off first) and before the response-timeout check (a response that
    // is already queued resolves its request instead of counting as a timeout).
    DrainResult rx = drainRx(state, now);
    if (rx == DrainResult::LATCHED) {
        return;
    }
    if (rx == DrainResult::BACKLOG) {
        // More queued than one pass handles: a foreign request may still be in there, so
        // transmit nothing this pass (and count no timeout).
        return;
    }
    // D-059/D-080: while a segmented answer is received nothing else is sent and no response
    // timeout runs. Lost frames and N_Cr are checked only now, after the drain, against
    // the drain time of the last CF, so CFs already queued are never "late".
    if (_isoRx.active()) {
        if (!_isoRx.checkAfterDrain(now, _bus.rxLostCount())) {
            return;
        }
        endReception(now);
    }
    // The FC of an accepted First Frame goes out only here, after the whole queue was
    // drained without a foreign tester (Q-018), never in the middle of a drain.
    if (_fcPending) {
        sendPendingFlowControl(now);
        if (_isoRx.active()) {
            return; // the reception started: nothing else this pass
        }
    }
    // D-080: at most one line of a 0x19 answer's hex dump per pass (Serial is slow).
    _dtc.flushOne();
    watchWarmIdle(now);

    if (_sessionStartPending) {
        // First pass in normal mode, after its drain: start the extended diagnostic
        // session. G2.3: not assumed to succeed -- retried below until 0x50 confirms it.
        _sessionStartPending = false;
        Serial.println("[CAN] Starting the diagnostic session.");
        sendRequest(kSessionRequest, sizeof(kSessionRequest));
        _lastSessionAttemptMs = now;
        _lastKeepAlive = now;
#if CONN_DISCOVERY_PROBE
        _firstSessionRequestMs = now;
#endif
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            _dids[i].lastRequestMs = now; // the DID schedule starts together with the session
        }
        return;
    }

    // 1. Bus-Off Auto Recovery Check (G1.3 -- exponential backoff, event logging)
    CanBusState busState = _bus.getState();
    if (busState == CanBusState::BUS_OFF) {
        if (!_busOff) {
            // Just entered bus-off: log once, reset backoff, attempt immediately.
            _busOff = true;
            _busOffEventCount++;
            if (_busOffEventCount >= kMaxBusOffEvents) {
                latchOff(TesterLatchReason::BUS_OFF, "too many bus-off events");
                state.engine.ecuPresent = false;
                return;
            }
            // Q-018: the budget counts from power-on, not from the last reset.
            _latchStore.save(TesterLatchReason::NONE, _busOffEventCount);
            _recoveryBackoffMs = kBackoffMinMs;
            _lastRecoveryAttempt = 0;
            uint16_t txErr = 0, rxErr = 0;
            _bus.getErrorCounters(txErr, rxErr);
            Serial.printf("[CAN WARNING] Bus-Off detected (event #%lu, tx_err=%u, rx_err=%u). Starting recovery...\n",
                (unsigned long)_busOffEventCount, txErr, rxErr);
        }

        if (now - _lastRecoveryAttempt >= _recoveryBackoffMs) {
            _lastRecoveryAttempt = now;
            Serial.printf("[CAN WARNING] Bus-Off recovery attempt (next retry in %lu ms if this fails)...\n",
                _recoveryBackoffMs);
            _bus.initiateRecovery();
            _recoveryBackoffMs = min(_recoveryBackoffMs * 2, kBackoffMaxMs);
        }
    } else if (busState == CanBusState::STOPPED) {
        // initiateRecovery() lands the driver here once recovery completes.
        if (_busOff) {
            Serial.println("[CAN SUCCESS] Bus-Off recovery complete, restarting driver.");
            _busOff = false;
            _recoveryBackoffMs = kBackoffMinMs;
        }
        _bus.start();
    } else if (busState == CanBusState::RUNNING && _busOff) {
        _busOff = false;
        _recoveryBackoffMs = kBackoffMinMs;
    }

    // txHold = the quiet time after a First Frame that got no FC or an aborted reception,
    // in which no request at all is sent (yaml flow_control rule). D-059 probe only (true in
    // every other build): during the scan, tester present and the session retry go out only
    // between requests, never while one is in flight.
    const bool txHold = beforeDeadline(now, _txQuietUntilMs);
    bool auxAllowed = true;
#if CONN_DISCOVERY_PROBE
    if (_scanPhase != ScanPhase::DONE) {
        auxAllowed = (_udsState == UdsRequestState::IDLE);
    }
#endif
    // D-080 (safety-reviewer MINOR-2): no tester present or session retry while a 0x19 is in
    // flight, so no other answer lands in its (segmented) reception. At most one slot plus a
    // reception (<= RESPONSE_TIMEOUT_MAX_MS), well inside S3 with the 1 s tester present.
    if (_pendingDtc && _udsState == UdsRequestState::WAITING) {
        auxAllowed = false;
    }
    bool auxSent = false; // tester present or session retry sent this pass

    // 2. Tester present -- only meaningful once a session is confirmed, but harmless to
    // send regardless (an ECU in default session simply ignores/NAKs it).
    if (!txHold && auxAllowed && now - _lastKeepAlive >= kTesterPresentPeriodMs) {
        _lastKeepAlive = now;
        sendRequest(kTesterPresentRequest, sizeof(kTesterPresentRequest));
        auxSent = true;
    }

    // 2b. G2.3 -- Retry the extended diagnostic session until a positive 0x50 confirms it.
    if (!txHold && auxAllowed && !_sessionConfirmed &&
        (_sessionRetryNow || now - _lastSessionAttemptMs >= kSessionRetryMs)) {
        _lastSessionAttemptMs = now;
        _sessionRetryNow = false;
        Serial.println("[UDS] Extended session (0x10 0x03) not yet confirmed -- retrying...");
        sendRequest(kSessionRequest, sizeof(kSessionRequest));
        auxSent = true;
    }
    (void)auxSent; // read by the 0x19 step below and the probe's scan step

    // 2c. G2.3 -- Derive ECU presence from recency of any UDS response and publish it
    // for consumers to show an explicit "ECU not found" state.
    bool ecuPresentNow = (_lastGoodResponseMs != 0) && (now - _lastGoodResponseMs < kEcuAbsentTimeoutMs);
    if (ecuPresentNow != _ecuPresent) {
        if (!ecuPresentNow) {
            _sessionConfirmed = false; // ECU lost (e.g. reset): request the session again
            _newStopEpoch = true;      // D-075 item 1: its confirmation starts a new stop epoch
        }
        _ecuPresent = ecuPresentNow;
        Serial.printf("[UDS] ECU is now %s.\n", _ecuPresent ? "PRESENT" : "NOT DETECTED");
    }
    state.engine.ecuPresent = _ecuPresent;

    // 3. G2.2 -- UDS single-request state machine: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT
    if (_udsState == UdsRequestState::IDLE && !txHold) {
#if CONN_DISCOVERY_PROBE
        // D-059: the scan runs first, once; DID polling starts when it is done. After a
        // tester present or session retry, the next scan request waits for the base
        // response timeout, so the aux request is answered (or not) before it (one request
        // in flight, safety-reviewer MAJOR-2).
        if (_scanPhase != ScanPhase::DONE) {
            if (auxSent) {
                unsigned long hold = now + kResponseTimeoutMs;
                if (!beforeDeadline(hold, _scanHoldUntilMs)) {
                    _scanHoldUntilMs = hold; // never shortens a longer hold
                }
            } else {
                scanStep(now);
            }
        } else
#endif
        {
        bool didSent = false;
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            DidSlot& slot = _dids[i];
            if (beforeDeadline(now, slot.skipUntilMs)) {
                continue; // temporarily skipped after repeated timeouts
            }
            if (slot.unsupported || (!slot.verified && !_sessionConfirmed)) {
                // D-068 item 2: an unverified DID is asked only in a confirmed session (an
                // ECU that serves it only there would refuse it before), and not at all
                // once stopped.
                continue;
            }
            if (now - slot.lastRequestMs >= slot.cadenceMs) {
                slot.lastRequestMs = now;
                _pendingDidIndex = i;
                _pendingDid = slot.did;
                _pendingSid = UDS_SID_READ_DATA_BY_IDENTIFIER;
                _udsState = UdsRequestState::REQUEST_SENT;

                requestDID(slot.did);

                // Transmission above is synchronous, so the request is immediately
                // outstanding -- move straight into WAITING for its response.
                _udsState = UdsRequestState::WAITING;
                _requestSentMs = now;
                _requestFirstSentMs = now;
                _pendingSawNrc78 = false;
                _pendingRttSkip = beforeDeadline(now, _rttQuietUntilMs);
                _responseTimeoutMs = kResponseTimeoutMs;
                didSent = true;
                break; // exactly one request in flight at a time
            }
        }
        // D-080 item 4: a 0x19 request only in a step where no DID was due (after every
        // verified and unverified DID), and only in a confirmed session, like an unverified read.
        // Not in a pass that sent tester present or the session retry: its answer could land in
        // the 0x19 reception (round-2 MINOR-3).
        if (!didSent && !auxSent && _sessionConfirmed && _dtc.due()) {
            sendDtcRequest(now);
        }
        }
    } else if (_udsState == UdsRequestState::WAITING) {
        if (now - _requestSentMs > _responseTimeoutMs) {
            _udsState = UdsRequestState::TIMEOUT;
        }
    }

#if CONN_DISCOVERY_PROBE
    if (_udsState == UdsRequestState::TIMEOUT && _pendingScanIndex >= 0) {
        // A scan entry is not a DID: no DID timeout count. Its answer may still arrive late
        // and be taken for the next entry's, so the next scan request waits.
        _scan.recordNoAnswer((uint8_t)_pendingScanIndex);
        _pendingScanIndex = -1;
        _scanHoldUntilMs = now + kResponsePendingMaxMs;
        _udsState = UdsRequestState::IDLE;
    }
#endif
    if (_udsState == UdsRequestState::TIMEOUT && _pendingDtc) {
        // Not a DID: no DID timeout count. A late answer no longer matches (the run ended).
        _pendingDtc = false;
        _dtc.onTimeout();
        _udsState = UdsRequestState::IDLE;
    }
    bool timeoutThisPass = false;
    if (_udsState == UdsRequestState::TIMEOUT) {
        DidSlot& slot = _dids[_pendingDidIndex];
        slot.consecutiveTimeouts++;
        _unansweredDidCount++;
        // D-066 item 4: silence is now this DID's latest answer, so an earlier NRC is no
        // longer shown (safety-reviewer M1); the record falls back to its value's age.
        state.vehicleSignals.dids[_pendingDidIndex].lastWasNrc = false;
        // D-058: the timed-out request's answer may still come and resolve a later request
        // early (see recordRoundTrip()); statistics only, the schedule is unchanged.
        _rttQuietUntilMs = now + kResponsePendingMaxMs;
        // Log only (the counters and the stop rule here do not depend on it): a bounded
        // number of lines per window. What was left out is folded into the first line of
        // the next window, so a timeout never prints more than this one line.
        timeoutThisPass = true;
        uint16_t foldedSuppressed = 0;
        if (now - _timeoutLogWindowStartMs >= kTimeoutLogWindowMs) {
            foldedSuppressed = _timeoutLogsSuppressed;
            _timeoutLogWindowStartMs = now;
            _timeoutLogBudget = kMaxTimeoutLogLinesPerWindow;
            _timeoutLogsSuppressed = 0;
        }
        if (takeLogSlot(_timeoutLogBudget)) {
            if (foldedSuppressed > 0) {
                Serial.printf("[UDS WARNING] Timeout waiting for DID 0x%04X (consecutive=%u/%u) (+%u suppressed in the last window)\n",
                    _pendingDid, slot.consecutiveTimeouts, kMaxConsecutiveTimeouts, (unsigned)foldedSuppressed);
            } else {
                Serial.printf("[UDS WARNING] Timeout waiting for DID 0x%04X (consecutive=%u/%u)\n",
                    _pendingDid, slot.consecutiveTimeouts, kMaxConsecutiveTimeouts);
            }
        } else if (_timeoutLogsSuppressed < 0xFFFFu) {
            _timeoutLogsSuppressed++; // saturating, never wraps
        }

        if (_pendingDidIndex == VEHICLE_CL250_IDX_ENGINE_SPEED) {
            _rpmRunSamples = 0; // D-069 item 3: the RPM samples must be consecutive
        }
        if (slot.consecutiveTimeouts >= kMaxConsecutiveTimeouts) {
            slot.consecutiveTimeouts = 0;
            if (!slot.verified && !slot.answered) {
                // D-068 item 2: never answered in this stop epoch: probably not served at all.
                slot.unsupported = true;
                slot.stopReason = StopReason::TIMEOUTS;
                Serial.printf("[UDS] DID 0x%04X never answered -- not asked again until the next stop epoch (D-068, D-075).\n",
                    _pendingDid);
            } else {
                slot.skipUntilMs = now + kDidSkipCooldownMs;
                Serial.printf("[UDS WARNING] DID 0x%04X unresponsive -- skipping requests for %lu ms.\n",
                    _pendingDid, kDidSkipCooldownMs);
            }
        }
        _udsState = UdsRequestState::IDLE;
    }

    if (_udsState == UdsRequestState::COMPLETE) {
        if (_pendingDidIndex >= 0) {
            _dids[_pendingDidIndex].consecutiveTimeouts = 0;
        }
        _pendingDtc = false;
#if CONN_DISCOVERY_PROBE
        _pendingScanIndex = -1;
#endif
        _udsState = UdsRequestState::IDLE;
    }

    // Log only: when timeouts stop, the last window's suppressed count is reported once on a
    // later pass (never in a pass that already printed a timeout line).
    if (!timeoutThisPass && _timeoutLogsSuppressed > 0 &&
        now - _timeoutLogWindowStartMs >= kTimeoutLogWindowMs) {
        Serial.printf("[UDS WARNING] %u DID timeout warnings suppressed in the last window\n",
            (unsigned)_timeoutLogsSuppressed);
        _timeoutLogsSuppressed = 0;
    }
}

void HondaCANModule::listenStep(SystemState& state, unsigned long now) {
    state.engine.ecuPresent = false;
    if (!_listenStarted) {
        // The window starts here, not in begin(): setup() still runs the other modules'
        // begin() in between, and the queue may overflow meanwhile (caught below).
        _listenStarted = true;
        _listenStartMs = now;
    }
    DrainResult rx = drainRx(state, now);
    if (rx == DrainResult::LATCHED) {
        return; // latched off before this node ever transmitted
    }
    uint32_t lost = _bus.rxLostCount();
    if (rx == DrainResult::BACKLOG || lost != _rxLostBaseline) {
        // Frames went unseen (queue full) or are still queued: this window proves nothing.
        _rxLostBaseline = lost;
        _listenStartMs = now;
        if (_listenRestarts++ == 0) {
            Serial.println("[CAN] Listen-only window restarted: frames were lost or still queued.");
        }
        return;
    }
    if (now - _listenStartMs < kListenOnlyWindowMs) {
        return;
    }
    _listening = false;
    if (!_bus.enterNormalMode()) {
        Serial.println("[CAN ERROR] Could not switch the CAN driver to normal mode -- poller disabled.");
        _bus.stop();
        _initialized = false;
        return;
    }
    Serial.printf("[CAN] No other tester seen for %lu ms (window restarts: %lu). Normal mode.\n",
        kListenOnlyWindowMs, (unsigned long)_listenRestarts);
    // The session request goes out on the next pass, after its RX drain, so frames that
    // arrived while the driver was reinstalled are checked first.
    _sessionStartPending = true;
}

bool HondaCANModule::isForeignTesterFrame(const CanFrame& f) {
    // Our ECU's physical request ID from any source address (29-bit normal fixed
    // addressing 0x18DA<TA><SA>), or exactly the ID when it is an 11-bit one.
    const bool primaryExtended = isotp::isExtendedId(VEHICLE_CL250_REQUEST_ID);
    const uint32_t primaryMask = primaryExtended ? ~isotp::kNormalFixedSourceAddressMask : 0xFFFFFFFFu;
    if (f.extended == primaryExtended && (f.id & primaryMask) == (VEHICLE_CL250_REQUEST_ID & primaryMask)) {
        return true;
    }
    if (f.extended == isotp::isExtendedId(VEHICLE_CL250_FALLBACK_REQUEST_ID) && f.id == VEHICLE_CL250_FALLBACK_REQUEST_ID) {
        return true;
    }
    // Q-021/D-040: the watch-only OBD functional (broadcast) request IDs of a generic OBD
    // tester. A 29-bit entry (0x18DB<TA><SA>) matches from any source address, like the
    // physical ID above; an 11-bit entry matches exactly.
    for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        const vehicle_cl250_watch_id_t& w = vehicle_cl250_functional_watch[i];
        const uint32_t mask = w.extended ? ~isotp::kNormalFixedSourceAddressMask : 0xFFFFFFFFu;
        if (f.extended == w.extended && (f.id & mask) == (w.id & mask)) {
            return true;
        }
    }
    return false;
}

// Reads incoming CAN frames. Bounded: at most kMaxRxFramesPerPass per call.
HondaCANModule::DrainResult HondaCANModule::drainRx(SystemState& state, unsigned long now) {
    uint8_t logBudget = kMaxRxLogLinesPerPass;
    CanFrame rxMsg;
    for (uint8_t n = 0; n < kMaxRxFramesPerPass; n++) {
        if (!_bus.receive(rxMsg)) {
            return DrainResult::DRAINED;
        }
        // TWAI does not receive its own frames, so a diagnostic request on the bus is
        // another tester (e.g. rt-core already polls). Two testers are never allowed
        // (D-021): latch this poller off, across resets until power-on (Q-018).
        if (isForeignTesterFrame(rxMsg)) {
            latchOff(TesterLatchReason::FOREIGN_TESTER, "another tester is on the vehicle bus (D-021)");
            state.engine.ecuPresent = false;
            return DrainResult::LATCHED;
        }
        bool isUDSResponse =
            (rxMsg.id == VEHICLE_CL250_RESPONSE_ID && rxMsg.extended == isotp::isExtendedId(VEHICLE_CL250_RESPONSE_ID)) ||
            (rxMsg.id == VEHICLE_CL250_FALLBACK_RESPONSE_ID &&
             rxMsg.extended == isotp::isExtendedId(VEHICLE_CL250_FALLBACK_RESPONSE_ID));
        if (!isUDSResponse || rxMsg.dlc < 2) {
            continue;
        }
        if (_listening) {
            // Nothing has been sent yet, so an ECU answer means someone else asked (their
            // request may have been lost or sent before the window began). Known false
            // positive, on the safe side: after a non-power-on reset the ECU may still
            // answer (or 0x78) a request this node sent before the reset; a power cycle
            // clears that latch.
            latchOff(TesterLatchReason::FOREIGN_TESTER, "the ECU answered a request this node did not send (D-021)");
            state.engine.ecuPresent = false;
            return DrainResult::LATCHED;
        }

        // G2.4 -- ISO-TP (ISO 15765-2) PCI byte check. data[0] high nibble is the frame
        // type: 0x0X = Single Frame (X = payload length), 0x1X = First Frame, 0x2X =
        // Consecutive Frame, 0x3X = Flow Control. A First Frame's data[1] is part of the
        // multi-frame length, not a SID, so parsing it as one would silently corrupt
        // SystemState. Multi-frame reassembly is only for the 0x19 answers (D-080) and the
        // D-059 probe's scan (below); moto-rt-core owns the long-term client (D-023), and
        // every DID used today fits in a Single Frame.
        uint8_t isoTpFrameType = (rxMsg.data[0] >> 4) & 0x0F;
        // D-059/D-080: segmented answers (an FC only for the 0x19 or scan request in flight).
        if (isoTpFrameType == IsoTpReceiver::kPciFirstFrame) {
            onFirstFrame(rxMsg, now, logBudget);
            continue;
        }
        if (isoTpFrameType == IsoTpReceiver::kPciConsecutiveFrame) {
            onConsecutiveFrame(rxMsg, now);
            continue;
        }
        if (isoTpFrameType == IsoTpReceiver::kPciSingleFrame &&
            (_isoRx.fromReceptionId(rxMsg) ||
             (_fcPending && rxMsg.id == _fcPendingFrame.id && rxMsg.extended == _fcPendingFrame.extended))) {
            _isoRx.abort(IsoTpReceiver::AbortReason::UNEXPECTED_FRAME); // then handled as usual
            endReception(now);
        }
        if (isoTpFrameType == IsoTpReceiver::kPciSingleFrame && onDtcSingleFrame(rxMsg, now)) {
            continue;
        }
#if CONN_DISCOVERY_PROBE
        if (isoTpFrameType == IsoTpReceiver::kPciSingleFrame && onScanSingleFrame(rxMsg, now)) {
            continue;
        }
#endif
        if (isoTpFrameType != 0x0) {
            if (takeLogSlot(logBudget)) {
                Serial.printf("[UDS WARNING] Unexpected ISO-TP frame type 0x%X (PCI=0x%02X) -- dropping frame.\n",
                    isoTpFrameType, rxMsg.data[0]);
            }
            continue;
        }
        // A ReadDataByIdentifier answer: the generated parser checks the Single Frame
        // PCI, the positive SID, the DID echo, the length and the physical range.
        uint16_t echoedDid = (uint16_t)((rxMsg.data[2] << 8) | rxMsg.data[3]);
        const vehicle_cl250_did_t* entry = vehicle_cl250_find(echoedDid);
        float value = 0.0f;
        if (entry != nullptr && vehicle_cl250_parse_response(echoedDid, rxMsg.data, rxMsg.dlc, &value)) {
            const uint8_t didIndex = (uint8_t)(entry - vehicle_cl250_dids);
            storeValue(state, didIndex, value, now);
            _didValue[didIndex] = value; // D-080 warm-idle trigger (bookkeeping only)
            _didValueMs[didIndex] = (now == 0) ? 1 : now;
            // D-066 item 4: the same validated answer, as raw data bytes for the BLE
            // vehicle-signal block. Bookkeeping only; nothing here changes what is sent.
            vsigRecordPositive(state.vehicleSignals.dids[didIndex], &rxMsg.data[4], entry->length, (uint32_t)now);
            // D-068: the DID echo proves the ECU serves this DID, also for a late answer, so
            // an earlier stop (e.g. an NRC 0x31 from the other response ID) is undone.
            _dids[didIndex].answered = true;
            // D-075 item 1 (f), review MINOR-1: a late answer (the other response ID) does not
            // undo a stop by the (f) counter; it undoes a 0x31 or timeout stop as before.
            if (_dids[didIndex].sessionNrcCount < kStopNrcReads) {
                _dids[didIndex].unsupported = false;
                _dids[didIndex].stopReason = StopReason::NONE;
            }
            _dids[didIndex].stopNrcCount = 0;
            _dids[didIndex].nrc31Seen = false; // review MINOR-2: with stopNrcCount, as in rt-core
            // G2.2 -- resolve the state machine only if this is the DID we're waiting on.
            if (_udsState == UdsRequestState::WAITING && echoedDid == _pendingDid) {
                if (didIndex == VEHICLE_CL250_IDX_ENGINE_SPEED) {
                    // D-069 item 3: only the answer to the current request is a fresh sample.
                    // In the quiet window after a timeout it may be the timed-out read's late
                    // answer (same DID echo), so it breaks the run instead (safety-reviewer
                    // MINOR-1).
                    if (_pendingRttSkip) {
                        _rpmRunSamples = 0;
                    } else {
                        watchEngineStart(value);
                    }
                }
                recordRoundTrip(now);
                _udsState = UdsRequestState::COMPLETE;
            }
            _lastGoodResponseMs = now; // G2.3: any DID response proves the ECU is present
        } else if ((rxMsg.data[0] & 0xF0) == 0 && (rxMsg.data[0] & 0x0F) >= 2 && rxMsg.dlc >= 3 &&
                   rxMsg.data[1] == kSidSessionResponse &&
                   (rxMsg.data[2] & 0x7F) == VEHICLE_CL250_SESSION_SUBFUNCTION) {
            // G2.3 -- Positive response to DiagnosticSessionControl (0x10).
            if (!_sessionConfirmed) {
                _sessionConfirmed = true;
                _sessionRetryNow = false;
                // D-075 (provisional) item 1: only the first confirmation of a stop epoch
                // (tester start, ECU lost) arms everything at once. A re-confirmation after
                // NRC 0x7E/0x7F stays in the epoch: stops, answered flags, the engine-start
                // flag and every lastRequestMs are kept.
                if (_newStopEpoch) {
                    _newStopEpoch = false;
                    armUnverifiedReads(now); // D-068 item 2: read every unverified DID once now
                }
                Serial.println("[UDS SUCCESS] Extended diagnostic session (0x10 0x03) confirmed by ECU.");
            }
            _lastGoodResponseMs = now;
        } else if (rxMsg.data[1] == UDS_SID_NEGATIVE_RESPONSE && rxMsg.dlc >= 4) {
            // G2.1 -- Negative response: [PCI][0x7F][echoed SID][NRC].
            uint8_t echoedSid = rxMsg.data[2];
            uint8_t nrc = rxMsg.data[3];
            _nrcCount++;
            _lastGoodResponseMs = now; // G2.3: a NRC still proves the ECU is alive and answering
            // Only an NRC for ReadDataByIdentifier can resolve or extend the pending DID
            // request; NRCs for 0x10/0x3E are logged and otherwise ignored.
            // A 0x19 is resolved only by onDtcSingleFrame() (its own response-ID rule, round-2
            // MINOR-4): a malformed or other-ID 7F 19 never resolves it here.
            bool forPendingRead = (_udsState == UdsRequestState::WAITING) &&
                                  (echoedSid == _pendingSid) && !_pendingDtc;

            if (nrc == UDS_NRC_RESPONSE_PENDING) {
                if (forPendingRead && !_pendingSawNrc78 && _pendingDidIndex >= 0) {
                    _pendingSawNrc78 = true; // D-058: once per request, and no round-trip sample
                    if (!_pendingRttSkip) {
                        // In the quiet window after a timeout this 0x78 (no DID) may be the
                        // timed-out read's: not counted against the pending DID.
                        _tester.recordNrc78((uint8_t)_pendingDidIndex);
                    }
                }
                // responsePending: the ECU is still working on the DID we're WAITING on.
                // Reset and double the timeout, but never beyond the generated maximum
                // total wait since the request was sent, so a 0x78 storm still times out.
                // D-068 (safety-reviewer MAJOR-1): never for an unverified DID, like a D-050
                // faulty read: its read holds the slot for at most the base timeout + one
                // step, the premise of the verified DIDs' gap bounds. Scan entries (-1) keep it.
                unsigned long waited = now - _requestFirstSentMs;
                const bool extendable = _pendingDidIndex < 0 || _dids[_pendingDidIndex].verified;
                if (forPendingRead && extendable && waited < kResponsePendingMaxMs) {
                    unsigned long left = kResponsePendingMaxMs - waited;
                    unsigned long doubled = min(_responseTimeoutMs * 2, kResponsePendingMaxMs);
                    _requestSentMs = now;
                    _responseTimeoutMs = min(doubled, left);
                }
                if (takeLogSlot(logBudget)) {
                    Serial.printf("[UDS] NRC 0x78 responsePending for SID 0x%02X -- extending wait to %lu ms.\n",
                        echoedSid, _responseTimeoutMs);
                }
            } else {
                if (takeLogSlot(logBudget)) {
                    Serial.printf("[UDS WARNING] Negative response: SID=0x%02X NRC=0x%02X (%s) [total NRCs=%lu]\n",
                        echoedSid, nrc, nrcName(nrc), (unsigned long)_nrcCount);
                }
                // D-068 (safety-reviewer MINOR-1): a read or tester present refused as not
                // supported in the active session means the ECU fell back to the default
                // session (a short reset, S3 expiry) without 3 s of silence. Ask for the
                // session again (retried below, D-020). D-075 item 1: its confirmation stays
                // in the same stop epoch and re-arms nothing; the refusal of a never-answered
                // unverified DID counts toward its stop (below).
                if (_sessionConfirmed &&
                    (nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION ||
                     nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION) &&
                    (echoedSid == UDS_SID_READ_DATA_BY_IDENTIFIER || echoedSid == VEHICLE_CL250_TESTER_PRESENT_SID)) {
                    _sessionConfirmed = false; // same stop epoch (D-075 item 1): no re-arm on confirm
                    Serial.println("[UDS] Extended session lost (NRC 0x7E/0x7F) -- requesting it again.");
                }
                // Any other NRC definitively resolves this request (not a timeout, not
                // success) -- don't leave the state machine WAITING on a rejected DID.
                if (forPendingRead) {
                    // D-066 item 4: shown as the pending DID's answer, except in the quiet
                    // window after a timeout, where the NRC (it carries no DID) may belong to
                    // the timed-out read (see recordRoundTrip()). A scan entry is no DID
                    // (_pendingDidIndex -1). Bookkeeping only, like the round-trip record.
                    if (_pendingDidIndex >= 0 && !_pendingRttSkip) {
                        vsigRecordNrc(state.vehicleSignals.dids[_pendingDidIndex], nrc);
                        // D-068 item 2: requestOutOfRange = the ECU does not serve this DID.
                        // Same quiet-window rule: an NRC carries no DID.
                        // Two separate reads (safety-reviewer MAJOR-2): requests go to both
                        // IDs, so one 0x31 may be the other ID's, or trail an earlier answer.
                        DidSlot& slot = _dids[_pendingDidIndex];
                        // D-075 (provisional) item 1: NRC 0x7E/0x7F to a read of an unverified
                        // DID without a positive answer in this epoch counts like 0x31.
                        const bool sessionNrc = nrc == UDS_NRC_SERVICE_NOT_SUPPORTED_IN_ACTIVE_SESSION ||
                            nrc == UDS_NRC_SUBFUNCTION_NOT_SUPPORTED_IN_ACTIVE_SESSION;
                        const bool nrc31 = nrc == UDS_NRC_REQUEST_OUT_OF_RANGE;
                        const bool counted = (nrc31 || sessionNrc) && !slot.verified && !slot.answered;
                        // D-075 item 1 (f), provisional: a session NRC is also counted, in its
                        // own counter, for an unverified DID that answered in this epoch
                        // (a positive answer does not reset it). Verified DIDs: never.
                        const bool countedF = sessionNrc && !slot.verified;
                        if (countedF) {
                            // D-075 item 1 (amended, provisional) (a), (f): only a counted
                            // refusal asks for the session at the next step instead of after
                            // kSessionRetryMs.
                            _sessionRetryNow = true;
                        }
                        if (counted && nrc31) {
                            slot.nrc31Seen = true;
                        }
                        bool stop = false;
                        if (counted && ++slot.stopNrcCount >= kStopNrcReads) {
                            stop = true;
                        }
                        if (countedF) {
                            if (slot.sessionNrcCount < UINT8_MAX) {
                                slot.sessionNrcCount++; // saturates, never wraps (review MINOR-3)
                            }
                        }
                        if (countedF && slot.sessionNrcCount >= kStopNrcReads) {
                            stop = true;
                        }
                        if (stop) {
                            slot.unsupported = true;
                            // Precedence: once a 0x31 was counted, the reason stays NRC_31.
                            slot.stopReason = slot.nrc31Seen ? StopReason::NRC_31 : StopReason::NRC_7E_7F;
                            Serial.printf("[UDS] DID 0x%04X not supported (NRC 0x%02X) -- not asked again in this stop epoch (D-075).\n",
                                _pendingDid, nrc);
                        }
                    }
                    if (_pendingDidIndex == VEHICLE_CL250_IDX_ENGINE_SPEED) {
                        _rpmRunSamples = 0; // D-069 item 3: an NRC breaks the run (MINOR-2)
                    }
                    recordRoundTrip(now);
                    _udsState = UdsRequestState::COMPLETE;
                }
            }
        }
    }
    return DrainResult::BACKLOG;
}

// D-058 item 1: the pending request was resolved at `now` by a positive response or an NRC
// other than 0x78. Time from its (first) send to this drain; none after a 0x78.
// None either for a request sent within kResponsePendingMaxMs after a timeout: the answer
// to the timed-out request may arrive late and resolve it early, which would record a
// too-short sample (a positive answer echoes its DID, so for the next request of the same
// DID; an NRC carries no DID, so for any DID read). An ECU answer later than the generated
// maximum wait is outside the ECU's own P2* limit and not covered.
void HondaCANModule::recordRoundTrip(unsigned long now) {
    if (_pendingSawNrc78 || _pendingRttSkip || _pendingDidIndex < 0) {
        return;
    }
    _tester.recordRoundTrip((uint8_t)_pendingDidIndex, (uint32_t)(now - _requestFirstSentMs));
}

void HondaCANModule::latchOff(TesterLatchReason reason, const char* why) {
    if (_latchedOff) {
        return;
    }
    _latchedOff = true;
    _latchReason = reason;
    _latchStore.save(reason, _busOffEventCount); // Q-018: first, so it holds even if a reset follows
    _bus.stop(); // no more transmission, no ACKs, no error frames from this node
    Serial.printf("[UDS STOP] Vehicle-bus poller latched off until power-on: %s.\n", why);
}

// ---------------------------------------------------------------------------
// D-080 -- DTC freeze frames; segmented reception of a 0x19 (or D-059 scan) answer.
// ---------------------------------------------------------------------------

void HondaCANModule::holdTx(unsigned long now) {
    // yaml flow_control: after a First Frame that got no FC (or an aborted reception) no
    // request for response_timeout_max_ms; the ECU's remaining frames drain meanwhile.
    _txQuietUntilMs = now + kResponsePendingMaxMs;
    if (_txQuietUntilMs == 0) {
        _txQuietUntilMs = 1; // 0 means "none"
    }
}

void HondaCANModule::sendDtcRequest(unsigned long now) {
    uint8_t request[DtcSnapshot::kRequestMax];
    const uint8_t len = _dtc.buildRequest(request);
    if (len == 0) {
        return;
    }
    _pendingDtc = true;
    _pendingDidIndex = -1; // not a DID: no round-trip sample, no DID timeout count
    _pendingDid = 0;
    _pendingSid = UDS_SID_READ_DTC_INFORMATION;
    _fcSentForPending = false;
    _requestLostBaseline = _bus.rxLostCount(); // a frame lost after this may be part of the answer
    // A padded Single Frame like every request; sendFrame() passes it through the D-020 gate.
    // Round-2 MAJOR-A/B: the run's first request (the count) goes to both request IDs; the
    // first response ID that answers it locks the run, and the rest goes to its pair only,
    // so the other addressing never answers a segmented 0x19 in between.
    if (_dtc.step() == DtcSnapshot::Step::COUNT) {
        _dtcLocked = false;
    }
    if (_dtcLocked) {
        sendFrame(_dtcRequestId, request, len);
    } else {
        sendRequest(request, len);
    }
    _udsState = UdsRequestState::WAITING;
    _requestSentMs = now;
    _requestFirstSentMs = now;
    _pendingSawNrc78 = false;
    _pendingRttSkip = true;
    _responseTimeoutMs = kResponseTimeoutMs; // D-080 item 4: never extended by NRC 0x78
}

bool HondaCANModule::onDtcSingleFrame(const CanFrame& f, unsigned long now) {
    if (!_pendingDtc || _udsState != UdsRequestState::WAITING) {
        return false;
    }
    const uint8_t len = (uint8_t)(f.data[0] & 0x0F);
    if (len == 0 || len > isotp::kSingleFrameMaxLen || len + 1u > f.dlc) {
        return false;
    }
    const uint8_t* msg = &f.data[1];
    const bool positive = _dtc.matchesPositive(msg, len);
    if (!positive && !_dtc.matchesNegative(msg, len)) {
        return false;
    }
    if (!acceptDtcAnswerFrom(f)) {
        return true; // the other addressing (or after a First Frame of ours): ignored
    }
    if (positive) {
        checkDtcSlot(now);
        _dtc.onPositive(msg, len, f.id);
        _lastGoodResponseMs = now;
        _udsState = UdsRequestState::COMPLETE;
        return true;
    }
    _nrcCount++;
    _lastGoodResponseMs = now; // G2.3: an NRC still proves the ECU is alive and answering
    switch (_dtc.onNrc(msg[2])) {
        case DtcSnapshot::NrcAction::WAIT:
            return true; // 0x78: no extension (D-080 item 4), it times out at the base timeout
        case DtcSnapshot::NrcAction::SESSION_LOST:
            // D-080 item 2 / D-075 item 1: the ECU fell back to the default session; ask for
            // it again at the next step. Its confirmation stays in the same stop epoch.
            if (_sessionConfirmed) {
                _sessionConfirmed = false;
                Serial.println("[UDS] Extended session lost (NRC 0x7E/0x7F to 0x19) -- requesting it again.");
            }
            _sessionRetryNow = true;
            break;
        default:
            break;
    }
    _udsState = UdsRequestState::COMPLETE;
    return true;
}

bool HondaCANModule::segmentedRequestPending() const {
    if (_udsState != UdsRequestState::WAITING) {
        return false;
    }
#if CONN_DISCOVERY_PROBE
    if (_pendingScanIndex >= 0) {
        return true;
    }
#endif
    return _pendingDtc;
}

void HondaCANModule::failSegmented(const char* why, uint16_t received, uint16_t announced, unsigned long now) {
    holdTx(now);
    if (_pendingDtc) {
        _dtc.onSegmentedFailure(why, received, announced); // D-080 item 3: 0x19 off for the epoch
    }
#if CONN_DISCOVERY_PROBE
    if (_pendingScanIndex >= 0) {
        _scan.recordAborted((uint8_t)_pendingScanIndex, why, received, announced);
    }
#endif
    _udsState = UdsRequestState::COMPLETE;
}

bool HondaCANModule::acceptDtcAnswerFrom(const CanFrame& f) {
    uint32_t requestId = 0;
    if (!pairedRequestId(f, requestId)) {
        return false;
    }
    if (_fcPending || _isoRx.active()) {
        // Our First Frame is being answered: a Single Frame now (from either ID) must not
        // resolve the 0x19 under the reception (round-2 MAJOR-B). If it is the other ID,
        // 0x19 is off for the epoch; the reception finishes and is checked and logged.
        if (!(_fcPending ? (f.id == _fcPendingFrame.id && f.extended == _fcPendingFrame.extended)
                         : _isoRx.fromReceptionId(f))) {
            _dtc.haltAfterAnswer(DtcSnapshot::Halt::BOTH_IDS);
        }
        return false;
    }
    if (_dtcLocked) {
        return f.id == _dtcResponseId && f.extended == _dtcResponseExtended;
    }
    lockDtcTo(f, requestId);
    return true;
}

void HondaCANModule::lockDtcTo(const CanFrame& f, uint32_t requestId) {
    _dtcLocked = true;
    _dtcResponseId = f.id;
    _dtcResponseExtended = f.extended;
    _dtcRequestId = requestId;
}

void HondaCANModule::checkDtcSlot(unsigned long now) {
    if (_pendingDtc && now - _requestFirstSentMs > kDtcSlotMaxMs) {
        _dtc.haltAfterAnswer(DtcSnapshot::Halt::SLOW);
    }
}

void HondaCANModule::onFirstFrame(const CanFrame& f, unsigned long now, uint8_t& logBudget) {
    if (_isoRx.active() || _fcPending) {
        const CanFrame& current = _isoRx.active() ? f : _fcPendingFrame;
        bool sameId = _isoRx.active() ? _isoRx.fromReceptionId(f)
                                      : (current.id == f.id && current.extended == f.extended);
        if (sameId) {
            // A second First Frame on the reception's ID: no new FC, the reception ends.
            _fcPending = false;
            _isoRx.abort(IsoTpReceiver::AbortReason::UNEXPECTED_FRAME);
            endReception(now);
        } else {
            // The other response ID answered too (requests go to both IDs): that one gets
            // no FC, and its ECU times out on N_Bs (safety-reviewer MINOR-2).
            holdTx(now);
            if (_pendingDtc) {
                // D-080 MAJOR-1 (a): this would repeat for every segmented 0x19 answer; once
                // per epoch, then 0x19 is off (the reception in progress still completes).
                _dtc.haltAfterAnswer(DtcSnapshot::Halt::BOTH_IDS);
            }
        }
        return;
    }
    const uint16_t announced = IsoTpReceiver::firstFrameLength(f);
    // The data after the 2-byte PCI, only from an 8-byte frame (a shorter FF is invalid).
    const uint16_t inFrame = (f.dlc != VEHICLE_CL250_FRAME_DLC) ? 0u
        : (announced < IsoTpReceiver::kFirstFramePayload) ? announced : (uint16_t)IsoTpReceiver::kFirstFramePayload;
    // The FF must carry the in-flight request's positive SID and echo (for 0x19 0x04/0x06
    // also the DTC), so an FC only ever goes to the answer of our own request; never within
    // a quiet time (safety-reviewer MINOR-1).
    bool ours = false;
    if (segmentedRequestPending() && inFrame > 0) {
        uint32_t requestId = 0;
        ours = _pendingDtc && _dtc.matchesPositive(&f.data[2], inFrame) && pairedRequestId(f, requestId) &&
               (!_dtcLocked || (f.id == _dtcResponseId && f.extended == _dtcResponseExtended));
        if (ours && !_dtcLocked) {
            lockDtcTo(f, requestId); // the count's answer, segmented
        }
#if CONN_DISCOVERY_PROBE
        ours = ours || (_pendingScanIndex >= 0 &&
                        DiscoveryScan::matchesPositive((uint8_t)_pendingScanIndex, &f.data[2], inFrame));
#endif
    }
    if (ours && !_fcSentForPending && !beforeDeadline(now, _txQuietUntilMs) && IsoTpReceiver::acceptableLength(f) != 0) {
        _fcSentForPending = true; // one FC per request (BS 0), also if it is never sent
        _fcPending = true;        // sent by poll() once the drain is complete
        _fcPendingFrame = f;
        _fcPendingMs = now;
        return;
    }
    // No FC: too long, not an answer to the request in flight, in a quiet time, or a second FF.
    holdTx(now);
    if (takeLogSlot(logBudget)) {
        Serial.printf("[UDS WARNING] First Frame (FF_DL=%u) not answered with an FC; no request for %lu ms.\n",
            announced, kResponsePendingMaxMs);
    }
    if (!ours && inFrame > 0 && f.data[2] == (uint8_t)(UDS_SID_READ_DTC_INFORMATION + UDS_POSITIVE_RESPONSE_OFFSET)) {
        // Round-2 MAJOR-A: any quiet time a 0x19 answer causes (a late copy, the other
        // addressing, another DTC's echo) is the epoch's one; then 0x19 is off. The request in
        // flight, if any, still resolves (or times out).
        _dtc.haltAfterAnswer(DtcSnapshot::Halt::BOTH_IDS);
    }
    if (ours) {
#if CONN_DISCOVERY_PROBE
        if (_pendingScanIndex >= 0) {
            _scan.recordFfRefused((uint8_t)_pendingScanIndex, announced);
        }
#endif
        if (_pendingDtc) {
            // D-080 item 3: above max_ff_dl (or no FC possible) = refused, 0x19 off for the epoch.
            _dtc.onSegmentedFailure("First Frame refused", 0, announced);
        }
        _udsState = UdsRequestState::COMPLETE;
    } else if (_udsState == UdsRequestState::WAITING && _pendingDidIndex >= 0 && inFrame >= 3 &&
               f.data[2] == (uint8_t)(UDS_SID_READ_DATA_BY_IDENTIFIER + UDS_POSITIVE_RESPONSE_OFFSET) &&
               (uint16_t)((f.data[3] << 8) | f.data[4]) == _pendingDid) {
        // yaml: a segmented DID answer that got no FC is not a DID timeout (no sample either).
        // D-068 (safety-reviewer MINOR-2): every table DID fits a Single Frame, so an
        // unverified DID answered this way is never decoded; stop it, or its read would hold
        // every request for the quiet time each period.
        DidSlot& slot = _dids[_pendingDidIndex];
        if (!slot.verified && !slot.answered) {
            slot.unsupported = true;
            slot.stopReason = StopReason::FIRST_FRAME;
        } else {
            // A verified or answered DID is never stopped: the D-050..D-053 skip, so the
            // quiet time does not repeat every period (mirrors rt-core's stop_or_skip).
            slot.skipUntilMs = now + kDidSkipCooldownMs;
        }
        _udsState = UdsRequestState::COMPLETE;
    }
}

// A quiet time started meanwhile by an FF on the other response ID does not stop this FC: it
// answers our own request, and the yaml quiet-time rule covers requests only (intended).
void HondaCANModule::sendPendingFlowControl(unsigned long now) {
    _fcPending = false;
    if (!segmentedRequestPending()) {
        // The request was resolved meanwhile: no FC, so the yaml quiet time (round-2 MAJOR-B).
        holdTx(now);
        return;
    }
    const uint16_t announced = IsoTpReceiver::firstFrameLength(_fcPendingFrame);
    const char* why = nullptr;
    const uint32_t lost = _bus.rxLostCount();
    if (lost != _requestLostBaseline) {
        why = "lost frame before the FC"; // part of the answer may be missing
    } else if (now - _fcPendingMs > kResponseTimeoutMs) {
        why = "FC too late"; // the drain stayed backlogged; well inside the ECU's N_Bs
    } else if (!sendFlowControl(_fcPendingFrame)) {
        why = "FC not sent";
    }
    if (why != nullptr) {
        failSegmented(why, 0, announced, now);
        return;
    }
    // Times count from this drain; the baseline is the count the request went out with.
    _isoRx.start(_fcPendingFrame, now, lost);
}

void HondaCANModule::onConsecutiveFrame(const CanFrame& f, unsigned long now) {
    if (!_isoRx.fromReceptionId(f)) {
        return; // no reception (or after an abort): a stray CF is dropped, never printed
    }
    if (_isoRx.onConsecutive(f, now) == IsoTpReceiver::Result::IN_PROGRESS) {
        return;
    }
    endReception(now);
}

void HondaCANModule::endReception(unsigned long now) {
    _fcPending = false;
    if (segmentedRequestPending()) {
        if (_isoRx.complete()) {
            // The FF matched the request's SID and echo; the payload starts at the SID.
            if (_pendingDtc) {
                checkDtcSlot(now);
                _dtc.onPositive(_isoRx.payload(), _isoRx.length(), _isoRx.receptionId());
            }
#if CONN_DISCOVERY_PROBE
            if (_pendingScanIndex >= 0) {
                _scan.recordPositive((uint8_t)_pendingScanIndex, _isoRx.payload(), _isoRx.length());
            }
#endif
            _lastGoodResponseMs = now;
            _udsState = UdsRequestState::COMPLETE;
        } else {
            // Nothing of a partial answer is printed (in the probe it may be the VIN).
            failSegmented(IsoTpReceiver::reasonText(_isoRx.abortReason()), _isoRx.received(), _isoRx.length(), now);
        }
    } else if (!_isoRx.complete()) {
        holdTx(now);
    }
    _isoRx.reset();
}

bool HondaCANModule::sendFlowControl(const CanFrame& firstFrame) {
    // The request ID paired with the response ID the First Frame came on (gen/ IDs only).
    uint32_t id = 0;
    const bool paired = pairedRequestId(firstFrame, id);
    CanFrame fc;
    fc.id = id;
    fc.extended = isotp::isExtendedId(id);
    fc.dlc = VEHICLE_CL250_FRAME_DLC;
    for (uint8_t i = 0; i < VEHICLE_CL250_FRAME_DLC; i++) {
        fc.data[i] = vehicle_cl250_fc_cts[i]; // byte for byte from gen/ (D-059 item 1)
    }
    // Same last word as every request: the latches, the listen window, and the D-020 gate.
    if (!paired || _latchedOff || _listening || !vehicle_cl250_frame_allowed(fc.data, fc.dlc)) {
        _blockedFrames++;
        Serial.printf("[UDS BLOCKED] Flow Control to 0x%08lX refused (total=%lu).\n",
            (unsigned long)fc.id, (unsigned long)_blockedFrames);
        return false;
    }
    if (!_bus.transmit(fc)) {
        return false;
    }
    Serial.printf("[ISOTP] FC.CTS sent to 0x%08lX for a %u-byte answer.\n",
        (unsigned long)fc.id, IsoTpReceiver::firstFrameLength(firstFrame));
    return true;
}
#if CONN_DISCOVERY_PROBE
// ---------------------------------------------------------------------------
// D-059 item 3 -- one-shot discovery scan (probe env only).
// ---------------------------------------------------------------------------

void HondaCANModule::scanStep(unsigned long now) {
    if (_scanPhase == ScanPhase::WAIT_SESSION) {
        // After the listen window and the session start: when 0x50 confirms the session, or
        // one retry interval after the first session request (then NRCs and timeouts are
        // the findings). A late 0x50 during the scan does not restart it.
        if (!_sessionConfirmed && now - _firstSessionRequestMs < kSessionRetryMs) {
            return;
        }
        _scanPhase = ScanPhase::RUNNING;
        _scan.begin(_sessionConfirmed);
    }
    if (beforeDeadline(now, _scanHoldUntilMs)) {
        return;
    }
    int next = _scan.next();
    if (next < 0) {
        _scanPhase = ScanPhase::DONE;
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            _dids[i].lastRequestMs = now; // the DID schedule starts after the scan
        }
        // D-075 item 1: the scan-end arm is the epoch's deferred first arm (the scan ran
        // before the DID polling). Without a confirmed session the next confirmation is
        // that first arm, also if the session was lost meanwhile.
        if (_sessionConfirmed) {
            _newStopEpoch = false;
            armUnverifiedReads(now); // D-068: their first read is due at once, as without a scan
        } else {
            _newStopEpoch = true;
        }
        return;
    }
    const vehicle_cl250_scan_t& e = vehicle_cl250_discovery_scan[next];
    _pendingScanIndex = (int8_t)next;
    _pendingDidIndex = -1; // not a DID: no round-trip sample, no DID timeout count
    _pendingDid = 0;
    _pendingSid = e.request[0];
    _fcSentForPending = false;
    _requestLostBaseline = _bus.rxLostCount(); // a frame lost after this may be part of the answer
    // A Single Frame like every request; sendFrame() passes it through the D-020 gate.
    sendRequest(e.request, e.size);
    _udsState = UdsRequestState::WAITING;
    _requestSentMs = now;
    _requestFirstSentMs = now;
    _pendingSawNrc78 = false;
    _pendingRttSkip = true;
    _responseTimeoutMs = kResponseTimeoutMs;
}

bool HondaCANModule::onScanSingleFrame(const CanFrame& f, unsigned long now) {
    if (_pendingScanIndex < 0 || _udsState != UdsRequestState::WAITING) {
        return false;
    }
    uint8_t len = (uint8_t)(f.data[0] & 0x0F);
    if (len == 0 || len > isotp::kSingleFrameMaxLen || len + 1u > f.dlc) {
        return false;
    }
    uint8_t i = (uint8_t)_pendingScanIndex;
    const uint8_t* msg = &f.data[1];
    if (DiscoveryScan::matchesPositive(i, msg, len)) {
        _scan.recordPositive(i, msg, len);
        _lastGoodResponseMs = now;
        _udsState = UdsRequestState::COMPLETE;
        return true;
    }
    if (DiscoveryScan::matchesNegative(i, msg, len) && msg[2] != UDS_NRC_RESPONSE_PENDING) {
        // 0x78 is left to the normal NRC handling, which extends the wait (capped).
        _nrcCount++;
        _scan.recordNrc(i, msg[2]);
        _lastGoodResponseMs = now;
        _udsState = UdsRequestState::COMPLETE;
        return true;
    }
    return false;
}
#endif // CONN_DISCOVERY_PROBE
