#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>

#include "vehicle_cl250.h" // generated: external/moto-vehicle-defs/gen/c/conn/

// G0.3 -- Data staleness tracking.
// Each CAN signal carries the millis() timestamp of its last producer write,
// so consumers (Nextion, BLE, WiFi, logger) can tell a frozen last-good-value
// apart from a currently-live one instead of silently displaying stale data.
constexpr uint32_t STALE_THRESHOLD_MS = 500; // e.g. RPM must refresh at least every 500ms

inline bool isStale(uint32_t lastUpdateMs, uint32_t thresholdMs = STALE_THRESHOLD_MS) {
    // lastUpdateMs == 0 means "never written since boot" -- always stale.
    return lastUpdateMs == 0 || (millis() - lastUpdateMs) > thresholdMs;
}

// Staleness threshold for a polled DID: the generated stale_after_ms of that DID
// (moto-vehicle-defs, 3 x poll period, D-025/D-029). The legacy code used one 500 ms
// value for everything (docs/legacy-telemetry-notes.md).
inline uint32_t didStaleThresholdMs(uint8_t didIndex) {
    return vehicle_cl250_dids[didIndex].stale_after_ms;
}

/**
 * @brief Engine and ECU telemetry metrics retrieved over CAN bus.
 */
struct EngineData {
    // G2.3 -- true once the ECU has answered any UDS request within the last
    // ECU_ABSENT_TIMEOUT_MS (see HondaCANModule); consumers use this to show an
    // explicit "ECU not present" state instead of just frozen/stale numbers.
    bool ecuPresent = false;

    float rpm = 0.0f;
    uint32_t rpmUpdatedMs = 0;
    uint8_t speed = 0;
    uint32_t speedUpdatedMs = 0;
    int16_t coolantTemp = 0;
    uint32_t coolantTempUpdatedMs = 0;
    float throttlePos = 0.0f;
    uint32_t throttlePosUpdatedMs = 0;
    float batteryVoltage = 0.0f;
    uint32_t batteryVoltageUpdatedMs = 0;
};

/**
 * @brief Smartphone / BLE telematics data (Music & Navigation).
 */
struct TelematicsData {
    char songTitle[32] = "Not Connected";
    char artistName[32] = "N/A";
    uint16_t navDistance = 0;
    uint8_t navIconID = 0;
    bool phoneConnected = false;
};

// Vehicle-bus CAN/tester health, written by the CAN producer (HondaCANModule) and sent
// in BLE telemetry v4 (moto-vehicle-defs ble/ble_schema.json, D-061). The numeric values of
// CanHealthState and the CAN_HEALTH_FLAG_* bits are part of that schema (static_assert in
// BLETelemetryPacket.h).
enum class CanHealthState : uint8_t {
    NOT_INSTALLED = 0, // no TWAI driver (poller-off build) or synthetic data (mock build)
    RUNNING = 1,
    ERROR_WARNING = 2, // TEC or REC at or above CAN_ERROR_WARNING_LIMIT
    BUS_OFF = 3,
    STOPPED = 4,       // recovering from bus-off, or latched off (see flags)
};

// ISO 11898-1 error-warning limit for TEC/REC.
constexpr uint16_t CAN_ERROR_WARNING_LIMIT = 96;

constexpr uint8_t CAN_HEALTH_FLAG_POLLER_ENABLED          = 1u << 0;
constexpr uint8_t CAN_HEALTH_FLAG_LATCHED_FOREIGN_TESTER  = 1u << 1;
constexpr uint8_t CAN_HEALTH_FLAG_LATCHED_BUS_OFF         = 1u << 2;
constexpr uint8_t CAN_HEALTH_FLAG_SYNTHETIC_DATA          = 1u << 3;

struct CanHealth {
    CanHealthState busState = CanHealthState::NOT_INSTALLED;
    uint16_t txErrorCount = 0;
    uint16_t rxErrorCount = 0;
    uint32_t busOffCount = 0;
    uint32_t unansweredDidCount = 0; // DID requests that timed out with no response at all
    uint8_t flags = 0;               // CAN_HEALTH_FLAG_* bits
};

// D-058: statistics about the tester itself, kept by HondaCANModule and sent in BLE telemetry
// v4. TEMPORARY like the lean fields (rt-core's health DID 0xFD02 replaces them, D-055).
// A build without a tester (mock, poller off, listen-only) leaves `available` false.
constexpr uint16_t TESTER_RTT_NONE_MS = 65535; // min round trip while there is no sample yet

// Round trip of one DID as the tester sees it: from the request's send to the step that
// drains its answer (positive response or an NRC other than 0x78), so it includes one step
// and loop latency. A request that got 0x78 counts in nrc78Count and gives no sample.
struct DidRoundTripStats {
    uint16_t minMs = TESTER_RTT_NONE_MS; // TESTER_RTT_NONE_MS = no sample yet
    uint16_t maxMs = 0;
    uint32_t sumMs = 0;                  // saturating
    uint32_t count = 0;                  // samples, saturating
    uint16_t nrc78Count = 0;             // requests that saw NRC 0x78, saturating
};

struct TesterStats {
    bool available = false;
    uint16_t stepGapMaxMs = 0;     // max gap between two poller step entries since boot, ms rounded up
    uint16_t stepGapOverCount = 0; // gaps above VEHICLE_CL250_CLIENT_STEP_MAX_MS
    DidRoundTripStats rtt[VEHICLE_CL250_DID_COUNT]; // generated DID table order
};

/**
 * @brief State of the on-board IMU sampler (ImuModule), mirrored here by its update()
 * so consumers can see it without touching the sampler task's data.
 */
struct ImuStatus {
    bool active = false;         // sampler running and its last read succeeded
    uint32_t samplesTaken = 0;
    uint32_t samplesDropped = 0; // ring buffer full, sample discarded
    uint32_t readErrors = 0;
};

/**
 * @brief Global System State container shared across all modules.
 */
struct SystemState {
    EngineData engine;
    CanHealth can;
    TesterStats tester;
    ImuStatus imu;
    // No lean angle here: the legacy complementary filter was dropped (D-023). Lean comes
    // from the rt-core EKF (platform.dbc LeanEstimate) once this node reads the platform bus.
    TelematicsData telematics;
};

#endif // SYSTEM_STATE_H
