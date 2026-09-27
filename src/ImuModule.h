#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <atomic>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "IModule.h"
#include "ImuRingBuffer.h"

/**
 * @brief 100 Hz raw accel + gyro sampler for data collection (D-032, Claude proposal).
 *
 * Sensor: MPU-6050 register-compatible 6-axis IMU on I2C (legacy wiring: SDA GPIO1,
 * SCL GPIO2, 400 kHz, address 0x68), configured to +-8 g / +-500 deg/s so the raw counts
 * match the schema scale (4096 LSB/g, 65.5 LSB/(deg/s)).
 *
 * Timing: an esp_timer fires every 10 ms and only wakes a dedicated sampler task pinned to
 * core 0. That task installs the I2C driver (so its interrupt also lands on core 0), does
 * every I2C transfer and pushes into the static SPSC ring that BLEServerModule drains. The
 * Arduino loop (CAN poller, BLE, consumers) runs on core 1 and never waits for the IMU:
 * update() below only reads counters.
 *
 * Not a safety function and not a vehicle signal: raw data for offline analysis only.
 * No lean estimation here (dropped with the legacy filter, D-023).
 */
class ImuModule : public IProducerModule {
public:
    explicit ImuModule(ImuRing& ring) : _ring(ring) {}

    // Starts the sampler task (which probes and configures the sensor on core 0) and,
    // when a sensor answered, the 100 Hz timer. Returns false (module excluded, no IMU
    // blocks sent) when no sensor answers.
    bool begin() override;
    // Mirrors the sampler's counters into state.imu (cheap, never touches I2C).
    void update(SystemState& state) override;
    bool isHealthy() const override { return _initialized; }

private:
    enum InitState : uint8_t { INIT_PENDING = 0, INIT_OK = 1, INIT_FAILED = 2 };

    static void timerCallback(void* arg);
    static void taskEntry(void* arg);
    void taskLoop();

    // Sampler task only.
    bool initSensor();
    bool configureSensor();
    bool configurationIntact();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* buf, size_t len);
    bool readSample(ImuSample& sample);

    ImuRing& _ring;
    bool _initialized = false;
    esp_timer_handle_t _timer = nullptr;
    TaskHandle_t _task = nullptr;
    int64_t _timerStartUs = 0;
    uint8_t _whoAmI = 0;

    std::atomic<uint8_t> _initState{INIT_PENDING};
    std::atomic<uint32_t> _readsOk{0};
    std::atomic<uint32_t> _samplesTaken{0};
    std::atomic<uint32_t> _samplesDropped{0};
    std::atomic<uint32_t> _readErrors{0};
    std::atomic<uint32_t> _reconfigurations{0};
    std::atomic<bool> _lastReadOk{false};

    // Main loop only (update()).
    uint32_t _lastReadsOk = 0;
    unsigned long _lastProgressMs = 0;
};

#endif // IMU_MODULE_H
