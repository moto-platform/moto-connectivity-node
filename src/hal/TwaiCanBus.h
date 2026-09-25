#ifndef TWAI_CAN_BUS_H
#define TWAI_CAN_BUS_H

#include "ICanBus.h"
#include "driver/twai.h"

/**
 * @brief G3.2 -- Real ESP32-S3 TWAI-backed ICanBus implementation.
 * Wraps the exact twai_* calls HondaCANModule used to call directly (500 kbps,
 * accept-all filter, bus-off/error-warning alerts). No behavior change versus the
 * pre-HAL code -- this only relocates the hardware I/O behind ICanBus.
 */
class TwaiCanBus : public ICanBus {
private:
    gpio_num_t _txPin;
    gpio_num_t _rxPin;

public:
    TwaiCanBus(gpio_num_t txPin, gpio_num_t rxPin);

    bool begin() override;
    bool transmit(const CanFrame& frame) override;
    bool receive(CanFrame& frame) override;
    CanBusState getState() override;
    void getErrorCounters(uint16_t& txErrorCount, uint16_t& rxErrorCount) override;
    bool initiateRecovery() override;
    bool start() override;
};

#endif // TWAI_CAN_BUS_H
