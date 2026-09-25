#include "WiFiServerModule.h"

// Access Point Credentials
#define AP_SSID "Honda-CL250-AP"

#ifndef AP_PASSWORD
#error "AP_PASSWORD not defined. Copy platformio_local.ini.example to platformio_local.ini (gitignored) and set your own AP password there."
#endif
// An empty or short password would silently start an open or failing AP.
static_assert(sizeof(AP_PASSWORD) - 1 >= 8 && sizeof(AP_PASSWORD) - 1 <= 63,
              "AP_PASSWORD must be 8-63 characters (WPA2)");

namespace {
const char kRootPage[] =
    "<html><head><title>Honda CL250 Telemetry AP</title></head>"
    "<body style='background:#0B0E14; color:#00F0FF; font-family:sans-serif; text-align:center; padding:50px;'>"
    "<h1>Honda CL250 Telemetry Backend Server</h1>"
    "<p style='color:#FFF;'>Access Point Active. Endpoint: "
    "<a href='/api/telemetry' style='color:#FFB800;'>/api/telemetry</a></p>"
    "</body></html>";
const char kStateNotBound[] = "{\"error\":\"State not bound\"}";
const char kOverflow[] = "{\"error\":\"Response too large\"}";
} // namespace

WiFiServerModule::WiFiServerModule(uint16_t port)
    : _server(port) {
    _jsonBuf[0] = '\0';
}

bool WiFiServerModule::begin() {
    // G1.5: AP_SSID/AP_PASSWORD are compile-time constants -- nothing to persist across
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
    _server.on("/", [this]() { handleRoot(); });
    _server.on("/api/telemetry", [this]() { handleTelemetryJson(); });

    _server.begin();
    _apEnabled = true;

    Serial.printf("[WIFI] Access Point enabled: %s\n", reason);
}

void WiFiServerModule::handleRoot() {
    _server.send_P(200, "text/html", kRootPage, sizeof(kRootPage) - 1);
}

void WiFiServerModule::handleTelemetryJson() {
    if (!_pSystemState) {
        _server.send_P(500, "application/json", kStateNotBound, sizeof(kStateNotBound) - 1);
        return;
    }

    size_t len = buildTelemetryJson(*_pSystemState, _jsonBuf, sizeof(_jsonBuf));
    if (len == 0) {
        _server.send_P(500, "application/json", kOverflow, sizeof(kOverflow) - 1);
        return;
    }
    // The legacy CORS header served the dropped web PWA (D-023) and is not sent any more.
    _server.send_P(200, "application/json", _jsonBuf, len);
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
        // phone app's "BLE failed -> Wi-Fi fallback" flow).
        else if (!state.telematics.phoneConnected && (now - _bootTimeMs) >= BLE_FALLBACK_TIMEOUT_MS) {
            enableAccessPoint("BLE not connected within 15s (auto fallback)");
        }

        if (!_apEnabled) {
            return; // AP not enabled yet, don't call handleClient()
        }
    }

    _server.handleClient();
}
