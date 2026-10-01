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

// Local policy of this TEMPORARY tester, not a vehicle definition: after this many
// bus-off events the driver is stopped for good (until reboot) instead of rejoining the
// OEM bus forever and putting error frames on it. Candidate for moto-vehicle-defs.
const uint32_t kMaxBusOffEvents = 5;

// Local policy of this TEMPORARY tester (Q-018): listen-only for this long after begin()
// before the first request, so a tester that is already polling the ECU is seen before
// this node ever transmits (D-021). Candidate for moto-vehicle-defs, like the one above.
const unsigned long kListenOnlyWindowMs = 2000;

// Bounded RX drain: at most this many frames per pass. Twice the TWAI RX queue depth,
// so one pass empties the queue, while a flooded bus still cannot stall the loop.
const uint8_t kMaxRxFramesPerPass = 64;
// At most this many per-frame log lines per pass (Serial output is slow; see TWDT).
const uint8_t kMaxRxLogLinesPerPass = 4;

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
} // namespace

HondaCANModule::HondaCANModule(ICanBus& bus, ITesterLatchStore& latchStore)
    : _bus(bus), _latchStore(latchStore) {
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        _dids[i].did = vehicle_cl250_dids[i].did;
        _dids[i].cadenceMs = vehicle_cl250_dids[i].poll_period_ms;
        _dids[i].lastRequestMs = 0;
        _dids[i].consecutiveTimeouts = 0;
        _dids[i].skipUntilMs = 0;
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
    poll(state);
    publishHealth(state);
}

// Read-only: mirrors bus/tester counters into SystemState for BLE telemetry v3. Never
// transmits and never changes the poller state; once latched off the driver is gone,
// so the counters stay frozen at their last values.
void HondaCANModule::publishHealth(SystemState& state) {
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

    if (_sessionStartPending) {
        // First pass in normal mode, after its drain: start the extended diagnostic
        // session. G2.3: not assumed to succeed -- retried below until 0x50 confirms it.
        _sessionStartPending = false;
        Serial.println("[CAN] Starting the diagnostic session.");
        sendRequest(kSessionRequest, sizeof(kSessionRequest));
        _lastSessionAttemptMs = now;
        _lastKeepAlive = now;
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

    // 2. Tester present -- only meaningful once a session is confirmed, but harmless to
    // send regardless (an ECU in default session simply ignores/NAKs it).
    if (now - _lastKeepAlive >= kTesterPresentPeriodMs) {
        _lastKeepAlive = now;
        sendRequest(kTesterPresentRequest, sizeof(kTesterPresentRequest));
    }

    // 2b. G2.3 -- Retry the extended diagnostic session until a positive 0x50 confirms it.
    if (!_sessionConfirmed && (now - _lastSessionAttemptMs >= kSessionRetryMs)) {
        _lastSessionAttemptMs = now;
        Serial.println("[UDS] Extended session (0x10 0x03) not yet confirmed -- retrying...");
        sendRequest(kSessionRequest, sizeof(kSessionRequest));
    }

    // 2c. G2.3 -- Derive ECU presence from recency of any UDS response and publish it
    // for consumers to show an explicit "ECU not found" state.
    bool ecuPresentNow = (_lastGoodResponseMs != 0) && (now - _lastGoodResponseMs < kEcuAbsentTimeoutMs);
    if (ecuPresentNow != _ecuPresent) {
        if (!ecuPresentNow) {
            _sessionConfirmed = false; // ECU lost (e.g. reset): request the session again
        }
        _ecuPresent = ecuPresentNow;
        Serial.printf("[UDS] ECU is now %s.\n", _ecuPresent ? "PRESENT" : "NOT DETECTED");
    }
    state.engine.ecuPresent = _ecuPresent;

    // 3. G2.2 -- UDS single-request state machine: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT
    if (_udsState == UdsRequestState::IDLE) {
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            DidSlot& slot = _dids[i];
            if (beforeDeadline(now, slot.skipUntilMs)) {
                continue; // temporarily skipped after repeated timeouts
            }
            if (now - slot.lastRequestMs >= slot.cadenceMs) {
                slot.lastRequestMs = now;
                _pendingDidIndex = i;
                _pendingDid = slot.did;
                _udsState = UdsRequestState::REQUEST_SENT;

                requestDID(slot.did);

                // Transmission above is synchronous, so the request is immediately
                // outstanding -- move straight into WAITING for its response.
                _udsState = UdsRequestState::WAITING;
                _requestSentMs = now;
                _requestFirstSentMs = now;
                _responseTimeoutMs = kResponseTimeoutMs;
                break; // exactly one request in flight at a time
            }
        }
    } else if (_udsState == UdsRequestState::WAITING) {
        if (now - _requestSentMs > _responseTimeoutMs) {
            _udsState = UdsRequestState::TIMEOUT;
        }
    }

    if (_udsState == UdsRequestState::TIMEOUT) {
        DidSlot& slot = _dids[_pendingDidIndex];
        slot.consecutiveTimeouts++;
        _unansweredDidCount++;
        Serial.printf("[UDS WARNING] Timeout waiting for DID 0x%04X (consecutive=%u/%u)\n",
            _pendingDid, slot.consecutiveTimeouts, kMaxConsecutiveTimeouts);

        if (slot.consecutiveTimeouts >= kMaxConsecutiveTimeouts) {
            slot.skipUntilMs = now + kDidSkipCooldownMs;
            slot.consecutiveTimeouts = 0;
            Serial.printf("[UDS WARNING] DID 0x%04X unresponsive -- skipping requests for %lu ms.\n",
                _pendingDid, kDidSkipCooldownMs);
        }
        _udsState = UdsRequestState::IDLE;
    }

    if (_udsState == UdsRequestState::COMPLETE) {
        _dids[_pendingDidIndex].consecutiveTimeouts = 0;
        _udsState = UdsRequestState::IDLE;
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
        // SystemState. Multi-frame reassembly is not implemented (moto-rt-core will do
        // it, D-023); every DID used today fits in a Single Frame.
        uint8_t isoTpFrameType = (rxMsg.data[0] >> 4) & 0x0F;
        if (isoTpFrameType != 0x0) {
            if (takeLogSlot(logBudget)) {
                Serial.printf("[UDS WARNING] Unsupported ISO-TP frame type 0x%X (PCI=0x%02X) -- multi-frame responses are not handled, dropping frame.\n",
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
            storeValue(state, (uint8_t)(entry - vehicle_cl250_dids), value, now);

            // G2.2 -- resolve the state machine only if this is the DID we're waiting on.
            if (_udsState == UdsRequestState::WAITING && echoedDid == _pendingDid) {
                _udsState = UdsRequestState::COMPLETE;
            }
            _lastGoodResponseMs = now; // G2.3: any DID response proves the ECU is present
        } else if ((rxMsg.data[0] & 0xF0) == 0 && (rxMsg.data[0] & 0x0F) >= 2 && rxMsg.dlc >= 3 &&
                   rxMsg.data[1] == kSidSessionResponse &&
                   (rxMsg.data[2] & 0x7F) == VEHICLE_CL250_SESSION_SUBFUNCTION) {
            // G2.3 -- Positive response to DiagnosticSessionControl (0x10).
            if (!_sessionConfirmed) {
                _sessionConfirmed = true;
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
            bool forPendingRead = (_udsState == UdsRequestState::WAITING) &&
                                  (echoedSid == UDS_SID_READ_DATA_BY_IDENTIFIER);

            if (nrc == UDS_NRC_RESPONSE_PENDING) {
                // responsePending: the ECU is still working on the DID we're WAITING on.
                // Reset and double the timeout, but never beyond the generated maximum
                // total wait since the request was sent, so a 0x78 storm still times out.
                unsigned long waited = now - _requestFirstSentMs;
                if (forPendingRead && waited < kResponsePendingMaxMs) {
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
                // Any other NRC definitively resolves this request (not a timeout, not
                // success) -- don't leave the state machine WAITING on a rejected DID.
                if (forPendingRead) {
                    _udsState = UdsRequestState::COMPLETE;
                }
            }
        }
    }
    return DrainResult::BACKLOG;
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
