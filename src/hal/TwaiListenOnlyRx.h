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

    /**
     * @brief Blocks the calling task for at most `timeoutMs` until a frame is queued; that
     * frame is kept and returned first by the next receive(), so no frame is lost or
     * reordered. While blocked, the core runs its idle task (the task watchdog needs it).
     * @return true if the call blocked (the queue was empty on entry), false if a frame was
     * already waiting and it returned at once.
     */
    bool waitForFrame(uint32_t timeoutMs);

private:
    static void copyFrame(const twai_message_t& msg, CanFrame& frame);

    gpio_num_t _txPin; // TWAI needs a TX pin in its config; in listen-only mode it stays recessive
    gpio_num_t _rxPin;
    bool _installed = false;
    bool _held = false;      // a frame taken by waitForFrame(), not yet handed out
    twai_message_t _heldMsg; // that frame
};

#endif // CONN_CAN_LISTEN_ONLY

#endif // TWAI_LISTEN_ONLY_RX_H
