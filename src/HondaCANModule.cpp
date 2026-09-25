#include "HondaCANModule.h"

// 29-bit Extended Honda UDS request ID (Target ECU 0x10, Source Tool 0xF1).
// The response IDs (UDS_RESP_29BIT/11BIT) are public static constants on the
// class now, exposed for test/test_can_protocol to construct fake ECU frames.
#define HONDA_UDS_REQ_29BIT   0x18DA10F1

// 11-bit Standard Honda/OBD2 UDS request ID (Target ECU 0x7E0, Source Tool 0x7E8)
#define HONDA_UDS_REQ_11BIT   0x7E0

// Out-of-class definitions for the static const members declared in the header.
// Not needed for their use as plain values (array sizes, direct comparisons), but
// required by the standard the moment anything takes their address/reference --
// e.g. min(_recoveryBackoffMs * 2, RECOVERY_BACKOFF_MAX_MS) with a real std::min
// template. This previously only "worked" because Arduino's min()/max() are
// reference-free macros; a strict linker (as native/test builds use) will reject
// the ODR-use otherwise. Harmless, always-correct C++ to add regardless.
const uint32_t HondaCANModule::UDS_RESP_29BIT;
const uint32_t HondaCANModule::UDS_RESP_11BIT;
const unsigned long HondaCANModule::RECOVERY_BACKOFF_MAX_MS;
const uint8_t HondaCANModule::DID_SLOT_COUNT;
const unsigned long HondaCANModule::UDS_BASE_TIMEOUT_MS;
const unsigned long HondaCANModule::UDS_MAX_TIMEOUT_MS;
const uint8_t HondaCANModule::UDS_MAX_CONSECUTIVE_TIMEOUTS;
const unsigned long HondaCANModule::UDS_DID_SKIP_COOLDOWN_MS;
const unsigned long HondaCANModule::SESSION_RETRY_INTERVAL_MS;
const unsigned long HondaCANModule::ECU_ABSENT_TIMEOUT_MS;

HondaCANModule::HondaCANModule(ICanBus& bus)
    : _bus(bus) {}

bool HondaCANModule::begin() {
    if (_bus.begin()) {
        Serial.println("[CAN SUCCESS] CAN Bus Active (500 kbps). Listening for Honda ECU...");
        delay(200);

        // Start UDS Extended Session ($10 $03) on both 29-bit and 11-bit IDs.
        // G2.3: this is not assumed to succeed -- update() retries it every
        // SESSION_RETRY_INTERVAL_MS until a positive 0x50 response confirms it.
        sendFrame29(0x02, 0x10, 0x03);
        sendFrame11(0x02, 0x10, 0x03);
        _lastSessionAttemptMs = millis();
        delay(50);
        _initialized = true;
        return true;
    }
    Serial.println("[CAN ERROR] Failed to initialize CAN bus driver! Check TX/RX pins.");
    _initialized = false;
    return false;
}

void HondaCANModule::sendFrame29(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
    CanFrame frame;
    frame.extended = true; // 29-bit Extended Frame
    frame.id = HONDA_UDS_REQ_29BIT;
    frame.dlc = 8;
    frame.data[0] = d0;
    frame.data[1] = d1;
    frame.data[2] = d2;
    frame.data[3] = d3;
    for (int i = 4; i < 8; i++) frame.data[i] = 0xAA;

    _bus.transmit(frame);
}

void HondaCANModule::sendFrame11(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3) {
    CanFrame frame;
    frame.extended = false; // 11-bit Standard Frame
    frame.id = HONDA_UDS_REQ_11BIT;
    frame.dlc = 8;
    frame.data[0] = d0;
    frame.data[1] = d1;
    frame.data[2] = d2;
    frame.data[3] = d3;
    for (int i = 4; i < 8; i++) frame.data[i] = 0xAA;

    _bus.transmit(frame);
}

const char* HondaCANModule::nrcName(uint8_t nrc) {
    switch (nrc) {
        case 0x10: return "generalReject";
        case 0x11: return "serviceNotSupported";
        case 0x12: return "subFunctionNotSupported";
        case 0x13: return "incorrectMessageLengthOrInvalidFormat";
        case 0x21: return "busyRepeatRequest";
        case 0x22: return "conditionsNotCorrect";
        case 0x24: return "requestSequenceError";
        case 0x31: return "requestOutOfRange";
        case 0x33: return "securityAccessDenied";
        case 0x35: return "invalidKey";
        case 0x78: return "responsePending";
        case 0x7E: return "subFunctionNotSupportedInActiveSession";
        case 0x7F: return "serviceNotSupportedInActiveSession";
        default:   return "unknownNRC";
    }
}

void HondaCANModule::requestDID(uint16_t did) {
    // Send request using both 29-bit extended and 11-bit standard CAN headers
    sendFrame29(0x03, 0x22, (did >> 8) & 0xFF, did & 0xFF);
    sendFrame11(0x03, 0x22, (did >> 8) & 0xFF, did & 0xFF);
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
            _recoveryBackoffMs = 1000;
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
            _recoveryBackoffMs = min(_recoveryBackoffMs * 2, RECOVERY_BACKOFF_MAX_MS);
        }
    } else if (busState == CanBusState::STOPPED) {
        // initiateRecovery() lands the driver here once recovery completes.
        if (_busOff) {
            Serial.println("[CAN SUCCESS] Bus-Off recovery complete, restarting driver.");
            _busOff = false;
            _recoveryBackoffMs = 1000;
        }
        _bus.start();
    } else if (busState == CanBusState::RUNNING && _busOff) {
        _busOff = false;
        _recoveryBackoffMs = 1000;
    }

    // 2. UDS Session Keep-Alive (1000ms) -- only meaningful once a session is confirmed,
    // but harmless to send regardless (an ECU in default session simply ignores/NAKs it).
    if (now - _lastKeepAlive >= 1000) {
        _lastKeepAlive = now;
        sendFrame29(0x02, 0x3E, 0x80);
        sendFrame11(0x02, 0x3E, 0x80);
    }

    // 2b. G2.3 -- Retry the extended diagnostic session until a positive 0x50 confirms
    // it, instead of assuming the single begin()-time request landed.
    if (!_sessionConfirmed && (now - _lastSessionAttemptMs >= SESSION_RETRY_INTERVAL_MS)) {
        _lastSessionAttemptMs = now;
        Serial.println("[UDS] Extended session (0x10 0x03) not yet confirmed -- retrying...");
        sendFrame29(0x02, 0x10, 0x03);
        sendFrame11(0x02, 0x10, 0x03);
    }

    // 2c. G2.3 -- Derive ECU presence from recency of any positive UDS response and
    // publish it for consumers (Nextion etc.) to show an explicit "ECU not found"
    // state rather than a frozen last-good value.
    bool ecuPresentNow = (_lastGoodResponseMs != 0) && (now - _lastGoodResponseMs < ECU_ABSENT_TIMEOUT_MS);
    if (ecuPresentNow != _ecuPresent) {
        _ecuPresent = ecuPresentNow;
        Serial.printf("[UDS] ECU is now %s.\n", _ecuPresent ? "PRESENT" : "NOT DETECTED");
    }
    state.engine.ecuPresent = _ecuPresent;

    // 3. G2.2 -- UDS single-request state machine: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT
    if (_udsState == UdsRequestState::IDLE) {
        // _dids is priority-ordered (RPM first) so a slow DID's wait never starves it
        // for longer than one UDS_RESPONSE_TIMEOUT_MS.
        for (uint8_t i = 0; i < DID_SLOT_COUNT; i++) {
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

                // Transmission above is synchronous (twai_transmit), so the request is
                // immediately outstanding -- move straight into WAITING for its response.
                _udsState = UdsRequestState::WAITING;
                _requestSentMs = now;
                _responseTimeoutMs = UDS_BASE_TIMEOUT_MS;
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
            _pendingDid, slot.consecutiveTimeouts, UDS_MAX_CONSECUTIVE_TIMEOUTS);

        if (slot.consecutiveTimeouts >= UDS_MAX_CONSECUTIVE_TIMEOUTS) {
            slot.skipUntilMs = now + UDS_DID_SKIP_COOLDOWN_MS;
            slot.consecutiveTimeouts = 0;
            Serial.printf("[UDS WARNING] DID 0x%04X unresponsive -- skipping requests for %lu ms.\n",
                _pendingDid, UDS_DID_SKIP_COOLDOWN_MS);
        }
        _udsState = UdsRequestState::IDLE;
    }

    // 4. Read incoming CAN frames from Honda ECU
    CanFrame rxMsg;
    while (_bus.receive(rxMsg)) {
        // Check if response comes from 29-bit or 11-bit UDS frame
        bool isUDSResponse = (rxMsg.id == UDS_RESP_29BIT || rxMsg.id == UDS_RESP_11BIT);

        // G2.4 -- ISO-TP (ISO 15765-2) PCI byte check. rxMsg.data[0] high nibble is the
        // frame type: 0x0X = Single Frame (X = payload length, what every DID response
        // here has always been), 0x1X = First Frame of a multi-frame response, 0x2X =
        // Consecutive Frame, 0x3X = Flow Control. The code below has always assumed
        // Single Frame and read data[1] straight as the SID; a First Frame's data[1] is
        // actually part of the multi-frame *length* field, not a SID, so parsing it as
        // one would silently corrupt SystemState. Multi-frame reassembly (First Frame +
        // Flow Control + Consecutive Frames) is not implemented -- every DID used today
        // fits in a Single Frame (<=7 bytes), so this only guards against silently
        // misreading a response that grows past that in the future.
        if (isUDSResponse) {
            uint8_t isoTpFrameType = (rxMsg.data[0] >> 4) & 0x0F;
            if (isoTpFrameType != 0x0) {
                Serial.printf("[UDS WARNING] Unsupported ISO-TP frame type 0x%X (PCI=0x%02X) -- multi-frame responses are not handled, dropping frame.\n",
                    isoTpFrameType, rxMsg.data[0]);
                continue;
            }
        }

        if (isUDSResponse && rxMsg.data[1] == 0x62) {
            uint16_t did = (rxMsg.data[2] << 8) | rxMsg.data[3];
            switch (did) {
                case 0xF40C: // Engine RPM
                    state.engine.rpm = ((rxMsg.data[4] << 8) | rxMsg.data[5]) / 4.0f;
                    state.engine.rpmUpdatedMs = now;
                    break;
                case 0xF40D: // Vehicle Speed (km/h)
                    state.engine.speed = rxMsg.data[4];
                    state.engine.speedUpdatedMs = now;
                    break;
                case 0xF405: // Coolant Temperature (°C)
                    state.engine.coolantTemp = rxMsg.data[4] - 40;
                    state.engine.coolantTempUpdatedMs = now;
                    break;
                case 0xF411: // Throttle Position (%)
                    state.engine.throttlePos = (rxMsg.data[4] * 100.0f) / 255.0f;
                    state.engine.throttlePosUpdatedMs = now;
                    break;
                case 0xF442: // Battery Voltage (mV / V)
                    if (rxMsg.dlc >= 6) {
                        uint16_t rawVolt = (rxMsg.data[4] << 8) | rxMsg.data[5];
                        state.engine.batteryVoltage = rawVolt > 500 ? (rawVolt / 1000.0f) : (rxMsg.data[4] / 10.0f);
                    } else {
                        state.engine.batteryVoltage = rxMsg.data[4] / 10.0f;
                    }
                    state.engine.batteryVoltageUpdatedMs = now;
                    break;
            }

            // G2.2 -- resolve the state machine only if this is the DID we're waiting on.
            if (_udsState == UdsRequestState::WAITING && did == _pendingDid) {
                _udsState = UdsRequestState::COMPLETE;
            }
            _lastGoodResponseMs = now; // G2.3: any successful DID read proves the ECU is present
        } else if (isUDSResponse && rxMsg.data[1] == 0x50) {
            // G2.3 -- Positive response to DiagnosticSessionControl (0x10).
            if (!_sessionConfirmed) {
                _sessionConfirmed = true;
                Serial.println("[UDS SUCCESS] Extended diagnostic session (0x10 0x03) confirmed by ECU.");
            }
            _lastGoodResponseMs = now;
        } else if (isUDSResponse && rxMsg.data[1] == 0x7F && rxMsg.dlc >= 4) {
            // G2.1 -- Negative response: [PCI][0x7F][echoed SID][NRC].
            uint8_t echoedSid = rxMsg.data[2];
            uint8_t nrc = rxMsg.data[3];
            _nrcCount++;
            _lastGoodResponseMs = now; // G2.3: a NRC still proves the ECU is alive and answering

            if (nrc == 0x78) {
                // responsePending: the ECU is still working on the DID we're WAITING on.
                // Reset and double the timeout (capped) instead of declaring a timeout.
                if (_udsState == UdsRequestState::WAITING) {
                    _requestSentMs = now;
                    _responseTimeoutMs = min(_responseTimeoutMs * 2, UDS_MAX_TIMEOUT_MS);
                }
                Serial.printf("[UDS] NRC 0x78 responsePending for SID 0x%02X -- extending wait to %lu ms.\n",
                    echoedSid, _responseTimeoutMs);
            } else {
                Serial.printf("[UDS WARNING] Negative response: SID=0x%02X NRC=0x%02X (%s) [total NRCs=%lu]\n",
                    echoedSid, nrc, nrcName(nrc), (unsigned long)_nrcCount);
                // Any other NRC definitively resolves this request (not a timeout, not
                // success) -- don't leave the state machine WAITING on a DID that was
                // just explicitly rejected.
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