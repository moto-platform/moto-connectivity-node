#ifndef TWAI_CAN_BUS_H
#define TWAI_CAN_BUS_H

#include "ICanBus.h"
#include "driver/twai.h"

/**
 * @brief G3.2 -- Real ESP32-S3 TWAI-backed ICanBus implementation.
 * Wraps the twai_* calls HondaCANModule used to call directly (500 kbps, accept-all
 * filter, bus-off/error-warning alerts). Q-018: the driver starts in listen-only mode
 * and is reinstalled in normal mode only when HondaCANModule's observation window ends;
 * stop() leaves TX held recessive.
 */
class TwaiCanBus : public ICanBus {
private:
    gpio_num_t _txPin;
    gpio_num_t _rxPin;
    bool _installed = false;

    bool install(twai_mode_t mode);
    void uninstall();

public:
    TwaiCanBus(gpio_num_t txPin, gpio_num_t rxPin);

    bool beginListenOnly() override;
    bool enterNormalMode() override;
    bool transmit(const CanFrame& frame) override;
    bool receive(CanFrame& frame) override;
    CanBusState getState() override;
    uint32_t rxLostCount() override;
    void getErrorCounters(uint16_t& txErrorCount, uint16_t& rxErrorCount) override;
    bool initiateRecovery() override;
    bool start() override;
    void stop() override;
};

#endif // TWAI_CAN_BUS_H
