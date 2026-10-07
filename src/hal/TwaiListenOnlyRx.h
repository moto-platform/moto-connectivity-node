#ifndef TWAI_LISTEN_ONLY_RX_H
#define TWAI_LISTEN_ONLY_RX_H

// Only the listen-only capture build (CONN_CAN_LISTEN_ONLY) uses the TWAI driver through
// this class; no other env compiles it.
#if defined(CONN_CAN_LISTEN_ONLY) && CONN_CAN_LISTEN_ONLY

#include "ICanRx.h"
#include "driver/twai.h"

/**
 * @brief D-058 item 4 -- ESP32-S3 TWAI as a bus monitor: TWAI_MODE_LISTEN_ONLY (ISO 11898-1
 * bus monitoring), 500 kbps, accept-all filter. Only twai_driver_install/start/receive/
 * get_status_info/stop/uninstall are used; twai_transmit is never called, so the linker
 * drops it from the listen-only image (checked in CI).
 */
class TwaiListenOnlyRx : public ICanRx {
public:
    TwaiListenOnlyRx(gpio_num_t txPin, gpio_num_t rxPin);

    bool begin() override;
    bool receive(CanFrame& frame) override;
    uint32_t rxLostCount() override;

private:
    gpio_num_t _txPin; // TWAI needs a TX pin in its config; in listen-only mode it stays recessive
    gpio_num_t _rxPin;
    bool _installed = false;
};

#endif // CONN_CAN_LISTEN_ONLY

#endif // TWAI_LISTEN_ONLY_RX_H
