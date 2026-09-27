#ifndef SERIAL_LOGGER_MODULE_H
#define SERIAL_LOGGER_MODULE_H

#include "IModule.h"

/**
 * @brief Serial Logger debug module.
 * Formats and prints real-time telemetry metrics to USB Serial terminal for monitoring.
 */
class SerialLoggerModule : public IConsumerModule {
private:
    unsigned long _printIntervalMs;

public:
    SerialLoggerModule(unsigned long printIntervalMs = 1000);
    bool begin() override;
    void update(const SystemState& state) override;
    bool isHealthy() const override { return true; }
    // G3.4 -- scheduling now lives in main.cpp instead of an internal _lastPrint
    // millis() gate; the configured interval is just reported here.
    uint32_t getPeriodMs() const override { return _printIntervalMs; }
};

#endif // SERIAL_LOGGER_MODULE_H
