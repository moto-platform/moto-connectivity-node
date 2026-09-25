#ifndef NEXTION_MODULE_H
#define NEXTION_MODULE_H

#include "IModule.h"

// G1.4 -- Out-of-range sentinel sent to a Nextion numeric component when its backing
// SystemState signal is stale (see isStale() in SystemState.h). See NextionModule.cpp
// for why -999 is safe across every field currently rendered.
constexpr int32_t NEXTION_STALE_SENTINEL = -999;

// G4.3 -- Max characters sanitized/embedded into a single Nextion .txt="..." command
// by setTxt(). Independent of SystemState's songTitle/artistName[32] bound -- this is
// a hard cap on what setTxt() will ever emit, regardless of caller.
constexpr size_t NEXTION_MAX_TEXT_LEN = 31;

/**
 * @brief Nextion HMI display driver module.
 * Sends updated telemetry values over HardwareSerial (UART) using Nextion instructions.
 */
class NextionModule : public IConsumerModule {
private:
    HardwareSerial& _serial;
    int8_t _rxPin;
    int8_t _txPin;
    bool _initialized = false;

    /**
     * @brief Transmits Nextion instruction terminator sequence (0xFF 0xFF 0xFF).
     */
    void sendEndCmd();

    /**
     * @brief Sets numerical component property on Nextion screen.
     */
    void setVal(const char* name, int32_t val);

    /**
     * @brief Sets string component property on Nextion screen.
     */
    void setTxt(const char* name, const char* text);

public:
    NextionModule(HardwareSerial& serial, int8_t rxPin, int8_t txPin);
    bool begin() override;
    void update(const SystemState& state) override;
    bool isHealthy() const override { return _initialized; }
    // G3.4 -- throttled from 10Hz loop rate to avoid over-saturating UART bandwidth;
    // scheduling now lives in main.cpp instead of an internal _lastRender millis() gate.
    uint32_t getPeriodMs() const override { return 100; }
};

#endif // NEXTION_MODULE_H