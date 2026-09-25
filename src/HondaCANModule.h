#ifndef HONDA_CAN_MODULE_H
#define HONDA_CAN_MODULE_H

#include "IModule.h"
#include "hal/ICanBus.h"

/**
 * @brief Module handling CAN bus communications with Honda ECU via a generic ICanBus.
 * Supports 29-bit Extended Honda UDS and 11-bit Standard OBD2 fallback queries.
 *
 * G3.2 -- depends only on ICanBus, never on driver/twai.h: main.cpp injects a
 * TwaiCanBus on real hardware and test/test_can_protocol injects a MockCanBus, so
 * this exact protocol logic (request state machine, NRC handling, ISO-TP frame
 * checking, bus-off recovery) runs identically either way and is unit-testable
 * with no ESP32 device attached.
 */
class HondaCANModule : public IProducerModule {
public:
    // Exposed so tests can construct realistic fake ECU responses without
    // duplicating these as separately-maintained magic numbers.
    static const uint32_t UDS_RESP_29BIT = 0x18DAF110;
    static const uint32_t UDS_RESP_11BIT = 0x7E8;

    explicit HondaCANModule(ICanBus& bus);
    bool begin() override;
    void update(SystemState& state) override;
    bool isHealthy() const override { return _initialized; }

private:
    ICanBus& _bus;
    bool _initialized = false;
    unsigned long _lastKeepAlive = 0;

    // G1.3 -- Bus-off recovery state. Recovery attempts back off exponentially
    // (1s, 2s, 4s ... capped at 30s) instead of hammering twai_initiate_recovery()
    // every single loop pass while the bus stays off.
    bool _busOff = false;
    unsigned long _lastRecoveryAttempt = 0;
    unsigned long _recoveryBackoffMs = 1000;
    uint32_t _busOffEventCount = 0;
    static const unsigned long RECOVERY_BACKOFF_MAX_MS = 30000;

    // G2.1 -- Negative response (NRC, 0x7F) bookkeeping.
    uint32_t _nrcCount = 0;

    /**
     * @brief Human-readable name for a UDS Negative Response Code (ISO 14229-1 Annex A).
     */
    static const char* nrcName(uint8_t nrc);

    // ------------------------------------------------------------------
    // G2.2 -- Single-request-in-flight UDS state machine.
    // UDS is a strict request/response protocol; the old code fired the RPM
    // request every 50ms and a rotating slow-DID request every 200ms
    // completely independently, so two requests could be in flight and their
    // responses could not be told apart. Now only one DID is ever awaited at
    // a time: IDLE -> REQUEST_SENT -> WAITING -> COMPLETE/TIMEOUT -> IDLE.
    // ------------------------------------------------------------------
    enum class UdsRequestState : uint8_t { IDLE, REQUEST_SENT, WAITING, COMPLETE, TIMEOUT };

    struct DidSlot {
        uint16_t did;
        unsigned long cadenceMs;      // desired re-request interval for this DID
        unsigned long lastRequestMs;
        uint8_t consecutiveTimeouts;
        unsigned long skipUntilMs;    // temporarily skipped after repeated timeouts

        DidSlot(uint16_t d, unsigned long cadence)
            : did(d), cadenceMs(cadence), lastRequestMs(0), consecutiveTimeouts(0), skipUntilMs(0) {}
    };

    // Ordered by priority: RPM (50ms/20Hz) is checked first every cycle so a slow
    // DID's wait can never starve it for more than one UDS_RESPONSE_TIMEOUT_MS.
    static const uint8_t DID_SLOT_COUNT = 5;
    DidSlot _dids[DID_SLOT_COUNT] = {
        {0xF40C, 50},   // Engine RPM
        {0xF40D, 800},  // Vehicle Speed
        {0xF411, 800},  // Throttle Position
        {0xF405, 800},  // Coolant Temperature
        {0xF442, 800},  // Control Module Battery Voltage
    };

    UdsRequestState _udsState = UdsRequestState::IDLE;
    int8_t _pendingDidIndex = -1;
    uint16_t _pendingDid = 0;
    unsigned long _requestSentMs = 0;
    unsigned long _responseTimeoutMs = 0;

    static const unsigned long UDS_BASE_TIMEOUT_MS = 100;
    static const unsigned long UDS_MAX_TIMEOUT_MS = 2000;   // ceiling after repeated 0x78 extensions
    static const uint8_t UDS_MAX_CONSECUTIVE_TIMEOUTS = 5;
    static const unsigned long UDS_DID_SKIP_COOLDOWN_MS = 5000;

    // ------------------------------------------------------------------
    // G2.3 -- Diagnostic session management + ECU-presence detection.
    // begin() fires the extended session request once but never confirmed it; if the
    // ECU is absent or rejects it, the module just kept requesting DIDs into the void.
    // Now the request is retried until a positive 0x50 response is seen, and any
    // positive UDS response (0x50 or 0x62) marks the ECU "present" for
    // ECU_ABSENT_TIMEOUT_MS, exposed via state.engine.ecuPresent for consumers
    // (Nextion) to show an explicit "ECU not found" state instead of frozen numbers.
    // ------------------------------------------------------------------
    bool _sessionConfirmed = false;
    unsigned long _lastSessionAttemptMs = 0;
    static const unsigned long SESSION_RETRY_INTERVAL_MS = 2000;

    unsigned long _lastGoodResponseMs = 0;
    bool _ecuPresent = false; // mirrors state.engine.ecuPresent; logged only on change
    static const unsigned long ECU_ABSENT_TIMEOUT_MS = 3000;

    /**
     * @brief Transmits a 29-bit Extended CAN frame for Honda UDS queries ($18DA10F1).
     */
    void sendFrame29(uint8_t d0, uint8_t d1, uint8_t d2 = 0xAA, uint8_t d3 = 0xAA);

    /**
     * @brief Transmits an 11-bit Standard CAN frame for OBD-II queries ($7DF).
     */
    void sendFrame11(uint8_t d0, uint8_t d1, uint8_t d2 = 0x55, uint8_t d3 = 0x55);

    /**
     * @brief Transmits a ReadDataByIdentifier ($22) request for a specific DID.
     */
    void requestDID(uint16_t did);
};

#endif // HONDA_CAN_MODULE_H