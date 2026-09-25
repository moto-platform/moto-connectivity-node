#include "WiFiServerModule.h"

// Access Point Credentials
#define AP_SSID "Honda-CL250-AP"

#ifndef AP_PASSWORD
#error "AP_PASSWORD not defined. Copy platformio_local.ini.example to platformio_local.ini (gitignored) and set your own AP password there."
#endif

WiFiServerModule::WiFiServerModule(uint16_t port)
    : _server(port) {}

bool WiFiServerModule::begin() {
    // G1.5: AP_SSID/AP_PASS are compile-time constants -- nothing to persist across
    // reboots. Without this, WiFi.mode()/softAP() write the config to NVS flash on
    // every single boot (Arduino-ESP32 default), which is unnecessary flash wear and
    // a needless window for a sudden power-cut mid-write to corrupt the NVS partition.
    WiFi.persistent(false);

    // G4.1: Do NOT start the SoftAP here. It stays off until a trigger fires
    // in update(): BOOT button held at startup, or BLE fallback timeout.
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    _bootTimeMs = millis();

    _initialized = true;
    return true;
}

void WiFiServerModule::enableAccessPoint(const char* reason) {
    if (_apEnabled) {
        return;
    }

    // Configure ESP32 as Wi-Fi Access Point (SoftAP)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    // Configure HTTP route endpoints
    _server.on("/", std::bind(&WiFiServerModule::handleRoot, this));
    _server.on("/api/telemetry", std::bind(&WiFiServerModule::handleTelemetryJson, this));

    _server.begin();
    _apEnabled = true;

    Serial.printf("[WIFI] Access Point enabled: %s\n", reason);
}

void WiFiServerModule::handleRoot() {
    String html = "<html><head><title>Honda CL250 Telemetry AP</title></head>";
    html += "<body style='background:#0B0E14; color:#00F0FF; font-family:sans-serif; text-align:center; padding:50px;'>";
    html += "<h1>Honda CL250 Telemetry Backend Server</h1>";
    html += "<p style='color:#FFF;'>Access Point Active. Endpoint: <a href='/api/telemetry' style='color:#FFB800;'>/api/telemetry</a></p>";
    html += "</body></html>";
    _server.send(200, "text/html", html);
}

// G4.3 -- songTitle/artistName come straight from a phone-controlled BLE write
// (see BLEServerModule::onWrite) and used to be interpolated into this endpoint's
// JSON with no escaping: a '"' or backslash in either field would break the JSON
// syntax for every client polling /api/telemetry. Escapes '"'/'\\' and drops any
// control/non-ASCII byte instead of passing it through.
static String jsonEscape(const char* s) {
    String out;
    for (size_t i = 0; s[i] != '\0'; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '"' || c == '\\') {
            out += '\\';
            out += (char)c;
        } else if (c >= 0x20 && c < 0x7F) {
            out += (char)c;
        }
        // else: drop control/non-ASCII bytes entirely
    }
    return out;
}

void WiFiServerModule::handleTelemetryJson() {
    if (!_pSystemState) {
        _server.send(500, "application/json", "{\"error\":\"State not bound\"}");
        return;
    }

    const EngineData& e = _pSystemState->engine;
    const DynamicsData& d = _pSystemState->dynamics;
    const TelematicsData& t = _pSystemState->telematics;

    String json = "{";
    json += "\"rpm\":" + String(e.rpm, 1) + ",";
    json += "\"speed\":" + String(e.speed) + ",";
    json += "\"coolantTemp\":" + String(e.coolantTemp) + ",";
    json += "\"throttlePos\":" + String(e.throttlePos, 1) + ",";
    json += "\"batteryVoltage\":" + String(e.batteryVoltage, 2) + ",";
    json += "\"leanAngle\":" + String(d.leanAngle, 1) + ",";
    json += "\"maxLeanLeft\":" + String(d.maxLeanLeft, 1) + ",";
    json += "\"maxLeanRight\":" + String(d.maxLeanRight, 1) + ",";
    json += "\"phoneConnected\":" + String(t.phoneConnected ? "true" : "false") + ",";
    json += "\"songTitle\":\"" + jsonEscape(t.songTitle) + "\",";
    json += "\"artistName\":\"" + jsonEscape(t.artistName) + "\"";
    json += "}";

    // Set CORS headers for Web dashboards
    _server.sendHeader("Access-Control-Allow-Origin", "*");
    _server.send(200, "application/json", json);
}

void WiFiServerModule::update(const SystemState& state) {
    _pSystemState = &state;

    if (!_apEnabled) {
        unsigned long now = millis();

        // Trigger 1: BOOT button (GPIO0, LOW = pressed) held within the first
        // ~1 second after startup.
        if (!_bootButtonHeld && (now - _bootTimeMs) < 1000 && digitalRead(BOOT_BUTTON_PIN) == LOW) {
            _bootButtonHeld = true;
            enableAccessPoint("BOOT button held at startup");
        }
        // Trigger 2: BLE hasn't connected within 15s -> auto fallback (keeps the
        // existing app.js/Dart "BLE failed -> WiFi fallback" flow unchanged).
        else if (!state.telematics.phoneConnected && (now - _bootTimeMs) >= BLE_FALLBACK_TIMEOUT_MS) {
            enableAccessPoint("BLE not connected within 15s (auto fallback)");
        }

        if (!_apEnabled) {
            return; // AP not enabled yet, don't call handleClient()
        }
    }

    _server.handleClient();
}
