#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "GpsCore.h"
#include "IModule.h"

/**
 * @brief u-blox NEO-M8N on a conn UART (D-060). TEMPORARY like conn's tester: rt-core owns
 * the long-term GPS (`hal/gps`, EKF input) and takes it over.
 *
 * A static task pinned to core 0 (static stack/TCB, no heap; priority as the IMU task,
 * below the BT/Wi-Fi stacks) installs the UART driver, configures the receiver (GpsCore)
 * and parses byte-wise. The Arduino loop on core 1 only copies the latest fix and the
 * counters into SystemState in update(), every kUpdatePeriodMs, so no parsing runs in the
 * tester's loop step (D-053); D-058's step gap is measured with the GPS running.
 *
 * Only ground speed, heading of motion, their accuracies, fix type, satellite count and
 * the node time ever leave GpsCore; latitude, longitude and height are never extracted
 * (invariant 7). Raw GPS bytes are never logged. update()'s copy is what BLEServerModule
 * sends as the defs GPS block (ble_schema.json `gpsBlock`, D-061); BLE holds no pointer here.
 */
class GpsModule : public IProducerModule {
public:
    static constexpr uint32_t kUpdatePeriodMs = 50u;

    // Starts the GPS task and returns at once (receiver set-up takes up to ~3 s and runs in
    // the task). Returns false if the build has no confirmed pins or the task failed.
    bool begin() override;
    // Copies the latest fix and counters into state.gps; never touches the UART.
    void update(SystemState& state) override;
    bool isHealthy() const override { return _started; }
    uint32_t getPeriodMs() const override { return kUpdatePeriodMs; }

private:
    static void taskEntry(void* arg);
    void taskLoop();

    GpsCore _core;
    bool _started = false;
    TaskHandle_t _task = nullptr;
    std::atomic<bool> _uartFailed{false};
    unsigned long _lastReportMs = 0;
    uint32_t _lastReportPvt = 0;
};

#endif // GPS_MODULE_H
