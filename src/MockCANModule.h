#ifndef MOCK_CAN_MODULE_H
#define MOCK_CAN_MODULE_H

#include "IModule.h"

/**
 * @brief Synthetic telemetry source used in place of HondaCANModule when no ECU/CAN
 * hardware is available. Writes the exact same SystemState fields (values,
 * *UpdatedMs timestamps, ecuPresent) that HondaCANModule would, using a plausible
 * simulated ride pattern instead of real UDS responses.
 *
 * This is a completely separate module -- HondaCANModule.cpp/.h are never touched or
 * imported here. Selected at build time only (see platformio.ini env
 * esp32-s3-devkitc-1-mock, MOCK_CAN_DATA build flag / main.cpp's #ifdef), so the real,
 * hardware-verified CAN/UDS path is unaffected regardless of which env is built.
 *
 * Also periodically simulates an "ECU dropout" (stops updating for a few seconds) so
 * the staleness (G0.3), Nextion safe-state (G1.4/G2.3) and ecuPresent logic can be
 * exercised on real ESP32/Nextion/BLE/WiFi hardware without a real Honda ECU.
 */
class MockCANModule : public IProducerModule {
private:
    unsigned long _lastUpdateMs = 0;
    bool _inDropout = false;

    static const unsigned long UPDATE_INTERVAL_MS = 50;     // matches HondaCANModule's RPM cadence
    static const unsigned long RIDE_CYCLE_MS = 20000;        // one simulated ride "lap"
    static const unsigned long DROPOUT_EVERY_MS = 30000;     // simulate ECU dropout this often
    static const unsigned long DROPOUT_DURATION_MS = 4000;   // long enough to trigger staleness/ecuPresent-absent

public:
    MockCANModule() {}

    bool begin() override;
    void update(SystemState& state) override;
    bool isHealthy() const override { return true; }
};

#endif // MOCK_CAN_MODULE_H
