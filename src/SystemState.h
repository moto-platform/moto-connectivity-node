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

// Staleness threshold for a polled DID: twice its poll period from the generated table,
// never below STALE_THRESHOLD_MS. The legacy code used 500 ms for every value, so the
// 800 ms DIDs flickered to "stale" between two polls (docs/legacy-telemetry-notes.md).
inline uint32_t didStaleThresholdMs(uint8_t didIndex) {
    uint32_t twice = 2u * (uint32_t)vehicle_cl250_dids[didIndex].poll_period_ms;
    return twice > STALE_THRESHOLD_MS ? twice : STALE_THRESHOLD_MS;
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

/**
 * @brief Global System State container shared across all modules.
 */
struct SystemState {
    EngineData engine;
    // No lean angle here: the legacy complementary filter was dropped (D-023). Lean comes
    // from the rt-core EKF (platform.dbc LeanEstimate) once this node reads the platform bus.
    TelematicsData telematics;
};

#endif // SYSTEM_STATE_H
