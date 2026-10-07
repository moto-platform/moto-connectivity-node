#ifndef I_CAN_RX_H
#define I_CAN_RX_H

#include <stdint.h>

#include "CanFrame.h"

/**
 * @brief D-058 item 4 -- receive-only CAN interface for the listen-only capture build
 * (CONN_CAN_LISTEN_ONLY). It has deliberately no transmit(): code that is handed an
 * ICanRx (CanCaptureModule) cannot send anything on the bus, and the listen-only env links
 * no twai_transmit at all (scripts/check_no_twai_tx.sh in CI).
 */
class ICanRx {
public:
    virtual ~ICanRx() {}

    /**
     * @brief Installs and starts the peripheral in listen-only (bus monitoring) mode with
     * an accept-all filter: no ACK, no error frames, no data frames.
     * @return true on success.
     */
    virtual bool begin() = 0;

    /**
     * @brief Non-blocking receive of one pending frame, if any.
     * @return true if a frame was available and written into `frame`.
     */
    virtual bool receive(CanFrame& frame) = 0;

    /**
     * @brief Frames the driver lost since begin() because its RX queue or FIFO was full.
     */
    virtual uint32_t rxLostCount() = 0;
};

#endif // I_CAN_RX_H
