#include "SerialLoggerModule.h"

SerialLoggerModule::SerialLoggerModule(unsigned long printIntervalMs)
    : _printIntervalMs(printIntervalMs) {}

bool SerialLoggerModule::begin() {
    // USB Serial terminal initialized in main setup()
    return true;
}

void SerialLoggerModule::update(const SystemState& state) {
    // G3.4 -- main.cpp now only calls this every getPeriodMs(), so every call here
    // does real work; the old internal _lastPrint millis() gate is gone.

    // G0.3: RPM staleness surfaced here as the reference signal (500ms threshold).
    // Other per-signal *UpdatedMs timestamps in SystemState are available the same way.
    bool rpmStale = isStale(state.engine.rpmUpdatedMs);

    Serial.printf("[TELEMETRY] RPM: %6.1f%s | SPEED: %3d km/h | TPS: %5.1f%% | ECT: %3d°C | BATT: %4.1fV | LEAN: %5.1f° (L:%.1f°/R:%.1f°) | BLE: %s\n",
                  state.engine.rpm,
                  rpmStale ? " [STALE]" : "",
                  state.engine.speed,
                  state.engine.throttlePos,
                  state.engine.coolantTemp,
                  state.engine.batteryVoltage,
                  state.dynamics.leanAngle,
                  state.dynamics.maxLeanLeft,
                  state.dynamics.maxLeanRight,
                  state.telematics.phoneConnected ? "CONNECTED" : "DISCONNECTED");
}
