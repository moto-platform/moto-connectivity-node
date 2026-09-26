#include "HondaCANModule.h"
#include "uds_iso14229.h"

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

const uint8_t kSidSessionResponse = VEHICLE_CL250_SESSION_POSITIVE_SID;

const uint8_t kSessionRequest[] = {VEHICLE_CL250_SESSION_SID, VEHICLE_CL250_SESSION_SUBFUNCTION};
const uint8_t kTesterPresentRequest[] = {VEHICLE_CL250_TESTER_PRESENT_SID,
                                         VEHICLE_CL250_TESTER_PRESENT_SUBFUNCTION};
} // namespace

HondaCANModule::HondaCANModule(ICanBus& bus)
    : _bus(bus) {
    for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
        _dids[i].did = vehicle_cl250_dids[i].did;
        _dids[i].cadenceMs = vehicle_cl250_dids[i].poll_period_ms;
        _dids[i].lastRequestMs = 0;
        _dids[i].consecutiveTimeouts = 0;
        _dids[i].skipUntilMs = 0;
    }
}

bool HondaCANModule::begin() {
    if (_bus.begin()) {
        Serial.println("[CAN SUCCESS] CAN Bus Active (500 kbps). Listening for Honda ECU...");
        delay(200);

        // Start the extended diagnostic session. G2.3: this is not assumed to succeed --
        // update() retries it until a positive 0x50 response confirms it.
        sendRequest(kSessionRequest, sizeof(kSessionRequest));
        _lastSessionAttemptMs = millis();
        delay(50);
        _initialized = true;
        return true;
    }
    Serial.println("[CAN ERROR] Failed to initialize CAN bus driver! Check TX/RX pins.");
    _initialized = false;
    return false;
}

bool HondaCANModule::sendFrame(uint32_t id, const uint8_t* request, uint8_t len) {
    CanFrame frame;
    frame.extended = uds::isExtendedId(id);
    frame.id = id;
    frame.dlc = VEHICLE_CL250_FRAME_DLC;
    frame.data[0] = len; // ISO-TP single-frame PCI
    for (uint8_t i = 1; i < VEHICLE_CL250_FRAME_DLC; i++) {
        frame.data[i] = (i - 1 < len) ? request[i - 1] : VEHICLE_CL250_PADDING_BYTE;
    }

    // D-021: only the ECU's own request IDs; D-020: only allow-listed services, Single
    // Frames only. The generated guard is the last word on what may reach the vehicle bus.
    bool idAllowed = (id == VEHICLE_CL250_REQUEST_ID) || (id == VEHICLE_CL250_FALLBACK_REQUEST_ID);
    if (!idAllowed || len == 0 || len > uds::kIsoTpSingleFrameMaxLen ||
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
    const uint8_t request[] = {uds::kSidReadDataByIdentifier, (uint8_t)((did >> 8) & 0xFF), (uint8_t)(did & 0xFF)};
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
    unsigned long now = millis();

    // 1. Bus-Off Auto Recovery Check (G1.3 -- exponential backoff, event logging)
    CanBusState busState = _bus.getState();
    if (busState == CanBusState::BUS_OFF) {
        if (!_busOff) {
            // Just entered bus-off: log once, reset backoff, attempt immediately.
            _busOff = true;
            _busOffEventCount++;
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
        _ecuPresent = ecuPresentNow;
        Serial.printf("[UDS] ECU is now %s.\n", _ecuPresent ? "PRESENT" : "NOT DETECTED");
    }
    state.engine.ecuPresent = _ecuPresent;

    // 3. G2.2 -- UDS single-request state machine: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT
    if (_udsState == UdsRequestState::IDLE) {
        for (uint8_t i = 0; i < VEHICLE_CL250_DID_COUNT; i++) {
            DidSlot& slot = _dids[i];
            if (now < slot.skipUntilMs) {
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

    // 4. Read incoming CAN frames from the ECU
    CanFrame rxMsg;
    while (_bus.receive(rxMsg)) {
        bool isUDSResponse =
            (rxMsg.id == VEHICLE_CL250_RESPONSE_ID && rxMsg.extended == uds::isExtendedId(VEHICLE_CL250_RESPONSE_ID)) ||
            (rxMsg.id == VEHICLE_CL250_FALLBACK_RESPONSE_ID &&
             rxMsg.extended == uds::isExtendedId(VEHICLE_CL250_FALLBACK_RESPONSE_ID));
        if (!isUDSResponse || rxMsg.dlc < 2) {
            continue;
        }

        // G2.4 -- ISO-TP (ISO 15765-2) PCI byte check. data[0] high nibble is the frame
        // type: 0x0X = Single Frame (X = payload length), 0x1X = First Frame, 0x2X =
        // Consecutive Frame, 0x3X = Flow Control. A First Frame's data[1] is part of the
        // multi-frame length, not a SID, so parsing it as one would silently corrupt
        // SystemState. Multi-frame reassembly is not implemented (moto-rt-core will do
        // it, D-023); every DID used today fits in a Single Frame.
        uint8_t isoTpFrameType = (rxMsg.data[0] >> 4) & 0x0F;
        if (isoTpFrameType != 0x0) {
            Serial.printf("[UDS WARNING] Unsupported ISO-TP frame type 0x%X (PCI=0x%02X) -- multi-frame responses are not handled, dropping frame.\n",
                isoTpFrameType, rxMsg.data[0]);
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
        } else if (rxMsg.data[1] == kSidSessionResponse) {
            // G2.3 -- Positive response to DiagnosticSessionControl (0x10).
            if (!_sessionConfirmed) {
                _sessionConfirmed = true;
                Serial.println("[UDS SUCCESS] Extended diagnostic session (0x10 0x03) confirmed by ECU.");
            }
            _lastGoodResponseMs = now;
        } else if (rxMsg.data[1] == uds::kSidNegativeResponse && rxMsg.dlc >= 4) {
            // G2.1 -- Negative response: [PCI][0x7F][echoed SID][NRC].
            uint8_t echoedSid = rxMsg.data[2];
            uint8_t nrc = rxMsg.data[3];
            _nrcCount++;
            _lastGoodResponseMs = now; // G2.3: a NRC still proves the ECU is alive and answering

            if (nrc == uds::kNrcResponsePending) {
                // responsePending: the ECU is still working on the DID we're WAITING on.
                // Reset and double the timeout (capped) instead of declaring a timeout.
                if (_udsState == UdsRequestState::WAITING) {
                    _requestSentMs = now;
                    _responseTimeoutMs = min(_responseTimeoutMs * 2, kResponsePendingMaxMs);
                }
                Serial.printf("[UDS] NRC 0x78 responsePending for SID 0x%02X -- extending wait to %lu ms.\n",
                    echoedSid, _responseTimeoutMs);
            } else {
                Serial.printf("[UDS WARNING] Negative response: SID=0x%02X NRC=0x%02X (%s) [total NRCs=%lu]\n",
                    echoedSid, nrc, uds::nrcName(nrc), (unsigned long)_nrcCount);
                // Any other NRC definitively resolves this request (not a timeout, not
                // success) -- don't leave the state machine WAITING on a rejected DID.
                if (_udsState == UdsRequestState::WAITING) {
                    _udsState = UdsRequestState::COMPLETE;
                }
            }
        }
    }

    if (_udsState == UdsRequestState::COMPLETE) {
        _dids[_pendingDidIndex].consecutiveTimeouts = 0;
        _udsState = UdsRequestState::IDLE;
    }
}
