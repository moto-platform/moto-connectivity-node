#include "MockCANModule.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

bool MockCANModule::begin() {
    Serial.println("[MOCK CAN] MockCANModule active -- synthetic telemetry, no real ECU/CAN hardware in use.");
    return true;
}

void MockCANModule::update(SystemState& state) {
    unsigned long now = millis();
    if (now - _lastUpdateMs < UPDATE_INTERVAL_MS) {
        return;
    }
    _lastUpdateMs = now;

    // Periodically simulate an ECU dropout so staleness/ecuPresent-absent paths get
    // exercised on real hardware without touching real ECU/CAN code at all.
    unsigned long cyclePos = now % (DROPOUT_EVERY_MS + DROPOUT_DURATION_MS);
    bool shouldDropout = cyclePos >= DROPOUT_EVERY_MS;

    if (shouldDropout) {
        if (!_inDropout) {
            _inDropout = true;
            Serial.println("[MOCK CAN] Simulating ECU dropout...");
        }
        state.engine.ecuPresent = false;
        return; // leave *UpdatedMs timestamps frozen so staleness kicks in downstream
    }
    if (_inDropout) {
        _inDropout = false;
        Serial.println("[MOCK CAN] Simulated ECU dropout ended, resuming telemetry.");
    }

    float ridePhase = (float)(now % RIDE_CYCLE_MS) / (float)RIDE_CYCLE_MS; // 0..1
    float rpmWave = sinf(ridePhase * 2.0f * PI);           // -1..1
    float throttleWave = sinf(ridePhase * 2.0f * PI + 0.6f);
    float leanWave = sinf(ridePhase * 4.0f * PI);          // faster oscillation, simulates S-curves

    float rpm = 3500.0f + rpmWave * 2500.0f;               // 1000-6000 RPM
    uint8_t speed = (uint8_t)constrain((rpm - 1000.0f) / (6000.0f - 1000.0f) * 140.0f, 0.0f, 255.0f);
    float throttlePos = (throttleWave * 0.5f + 0.5f) * 100.0f; // 0-100%
    float leanAngle = leanWave * 35.0f;                    // -35..+35 degrees

    // Coolant ramps from ambient to operating temperature over the first minute of
    // runtime, then holds with small noise -- mirrors a real cold-start warmup.
    float warmupSec = now / 1000.0f;
    int16_t coolantTemp = (int16_t)fminf(20.0f + warmupSec * (70.0f / 60.0f), 90.0f);

    float batteryVoltage = 12.6f + sinf(now / 5000.0f) * 0.15f;

    state.engine.rpm = rpm;
    state.engine.rpmUpdatedMs = now;
    state.engine.speed = speed;
    state.engine.speedUpdatedMs = now;
    state.engine.coolantTemp = coolantTemp;
    state.engine.coolantTempUpdatedMs = now;
    state.engine.throttlePos = throttlePos;
    state.engine.throttlePosUpdatedMs = now;
    state.engine.batteryVoltage = batteryVoltage;
    state.engine.batteryVoltageUpdatedMs = now;
    state.engine.ecuPresent = true;

    state.dynamics.leanAngle = leanAngle;
    state.dynamics.leanAngleUpdatedMs = now;
    if (leanAngle > state.dynamics.maxLeanRight) {
        state.dynamics.maxLeanRight = leanAngle;
    }
    if (leanAngle < state.dynamics.maxLeanLeft) {
        state.dynamics.maxLeanLeft = leanAngle;
    }
}
