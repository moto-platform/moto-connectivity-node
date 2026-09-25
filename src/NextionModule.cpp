#include "NextionModule.h"

NextionModule::NextionModule(HardwareSerial& serial, int8_t rxPin, int8_t txPin) 
    : _serial(serial), _rxPin(rxPin), _txPin(txPin) {}

bool NextionModule::begin() {
    // HardwareSerial::begin(baud, config, rxPin, txPin)
    _serial.begin(115200, SERIAL_8N1, _rxPin, _txPin);
    _initialized = true;
    return true;
}

void NextionModule::sendEndCmd() {
    _serial.write(0xFF);
    _serial.write(0xFF);
    _serial.write(0xFF);
}

void NextionModule::setVal(const char* name, int32_t val) {
    _serial.printf("%s.val=%d", name, val);
    sendEndCmd();
}

void NextionModule::setTxt(const char* name, const char* text) {
    // G4.3 -- text may come straight from a phone-controlled BLE write (songTitle/
    // artistName, see BLEServerModule::onWrite). It's embedded in a "..."-quoted
    // Nextion command below, and every Nextion instruction is terminated by any
    // 0xFF 0xFF 0xFF byte sequence regardless of context. An unfiltered '"' would
    // let the string break out of its quotes early; an unfiltered 0xFF could
    // terminate this command and start injecting a new, attacker-chosen one. Strip
    // anything outside printable 7-bit ASCII (and '"'/'\\') and hard-cap the length.
    char safe[NEXTION_MAX_TEXT_LEN + 1];
    size_t i = 0;
    for (; text[i] != '\0' && i < NEXTION_MAX_TEXT_LEN; i++) {
        unsigned char c = (unsigned char)text[i];
        safe[i] = (c == '"' || c == '\\' || c < 0x20 || c >= 0x7F) ? '_' : (char)c;
    }
    safe[i] = '\0';

    _serial.printf("%s.txt=\"%s\"", name, safe);
    sendEndCmd();
}

void NextionModule::update(const SystemState& state) {
    // G3.4 -- main.cpp now only calls this every getPeriodMs() (100ms), so every call
    // here does real work; the old internal _lastRender millis() gate is gone.

    // G1.4 safe state: a signal past its staleness threshold (G0.3) is sent as
    // NEXTION_STALE_SENTINEL instead of its last-known value, so a severed CAN
    // line stops rendering as if the engine were still reporting live data.
    // NOTE: -999 is out of range for every field below (RPM/speed/TPS/volt are
    // never negative; lean angle stays within +-90 deg in practice), so it is
    // safe to use as a sentinel across all of them.
    // Open follow-up (needs the Nextion Editor .HMI project, not present in this
    // repo): configure each numeric component (or an overlay text component) to
    // render -999 as "--" instead of the literal number.
    setVal("n_rpm",   isStale(state.engine.rpmUpdatedMs)         ? NEXTION_STALE_SENTINEL : (int32_t)state.engine.rpm);
    setVal("n_speed", isStale(state.engine.speedUpdatedMs)       ? NEXTION_STALE_SENTINEL : (int32_t)state.engine.speed);
    setVal("n_temp",  isStale(state.engine.coolantTempUpdatedMs) ? NEXTION_STALE_SENTINEL : (int32_t)state.engine.coolantTemp);
    setVal("n_tps",   isStale(state.engine.throttlePosUpdatedMs) ? NEXTION_STALE_SENTINEL : (int32_t)state.engine.throttlePos);
    setVal("n_volt",  isStale(state.engine.batteryVoltageUpdatedMs) ? NEXTION_STALE_SENTINEL : (int32_t)(state.engine.batteryVoltage * 10.0f)); // e.g. 12.4V -> 124

    // Vehicle dynamics telemetry
    setVal("n_lean", isStale(state.dynamics.leanAngleUpdatedMs) ? NEXTION_STALE_SENTINEL : (int32_t)state.dynamics.leanAngle);

    // G2.3 -- explicit "ECU not found" indicator, distinct from a merely-stale value
    // (e.g. right after boot before any UDS response has ever arrived).
    // Open follow-up, same as G1.4's sentinel: needs the Nextion Editor .HMI project
    // (not present in this repo) to have a "t_ecu" text component to receive this.
    setTxt("t_ecu", state.engine.ecuPresent ? "ECU OK" : "ECU YOK");

    // Telematics / Smartphone integration (optional Nextion UI widgets)
    if (state.telematics.phoneConnected) {
        setTxt("t_song", state.telematics.songTitle);
        setVal("n_navdist", state.telematics.navDistance);
    }
}