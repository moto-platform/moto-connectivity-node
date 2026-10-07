#ifndef CAN_CAPTURE_MODULE_H
#define CAN_CAPTURE_MODULE_H

#include "CanCaptureCore.h"
#include "IModule.h"
#include "hal/ICanRx.h" // the only CAN interface this module sees: receive-only (D-058 item 4)

/**
 * @brief D-058 item 4 -- producer of the listen-only capture build (CONN_CAN_LISTEN_ONLY).
 * Each loop pass it drains the receive-only ICanRx and streams the frames and per-ID
 * summaries to USB serial (line format in CanCaptureCore.h). It depends on ICanRx only: there
 * is no transmit path to call, and it never blocks on the serial port. In that build it is
 * the only module and the only writer to Serial. The core logic (CanCaptureCore) is pure and
 * tested natively; this class adds the Serial sink and a 64-bit microsecond clock.
 */
class CanCaptureModule : public IProducerModule {
public:
    explicit CanCaptureModule(ICanRx& rx);
    bool begin() override;
    void update(SystemState& state) override;
    bool isHealthy() const override { return _started; }

    const CanCaptureCore& core() const { return _core; }

private:
    // Serial output that reports its free space, so the core can drop a line instead of
    // letting Serial.write() block.
    class SerialSink : public ICaptureSink {
    public:
        size_t availableForWrite() override;
        void write(const char* data, size_t len) override;
    };

    ICanRx& _rx;
    SerialSink _sink;
    CanCaptureCore _core;
    bool _started = false;
    uint32_t _lastMicros32 = 0;
    uint64_t _timeUs = 0; // micros() widened to 64 bits (it wraps every 71.6 minutes)
};

#endif // CAN_CAPTURE_MODULE_H
