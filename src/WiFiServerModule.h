#ifndef WIFI_SERVER_MODULE_H
#define WIFI_SERVER_MODULE_H

#include "IModule.h"
#include <WiFi.h>
#include <WebServer.h>

/**
 * @brief Wi-Fi Access Point and HTTP REST/JSON Telemetry Backend Server module.
 * Creates a wireless Access Point ("Honda-CL250-AP") and serves live telemetry JSON
 * endpoints for smartphone mobile apps and Web browser dashboards.
 */
class WiFiServerModule : public IConsumerModule {
private:
    WebServer _server;
    bool _initialized = false;
    const SystemState* _pSystemState = nullptr;

    // G4.1: AP is off by default and only enabled on-demand (BOOT button at
    // startup, or BLE fallback timeout) to avoid broadcasting an open wireless
    // access point continuously.
    bool _apEnabled = false;
    bool _bootButtonHeld = false;
    unsigned long _bootTimeMs = 0;
    static const unsigned long BLE_FALLBACK_TIMEOUT_MS = 15000; // 15s
    static const gpio_num_t BOOT_BUTTON_PIN = GPIO_NUM_0; // ESP32-S3 DevKitC-1 onboard BOOT button, no extra hardware needed

    /**
     * @brief Handles HTTP GET request to /api/telemetry returning JSON payload.
     */
    void handleTelemetryJson();

    /**
     * @brief Handles HTTP GET request to root endpoint serving status page.
     */
    void handleRoot();

    /**
     * @brief Actually starts the SoftAP and HTTP server. Called on-demand from
     * update() once a trigger condition (BOOT button / BLE fallback) fires.
     */
    void enableAccessPoint(const char* reason);

public:
    WiFiServerModule(uint16_t port = 80);
    virtual ~WiFiServerModule() {}

    bool begin() override;
    void update(const SystemState& state) override;
    bool isHealthy() const override { return _initialized; }
};

#endif // WIFI_SERVER_MODULE_H
