#ifndef I_CAN_BUS_H
#define I_CAN_BUS_H

#include <stdint.h>

/**
 * @brief Hardware-independent CAN frame, decoupled from ESP-IDF's twai_message_t so
 * protocol code (HondaCANModule) never needs to know it's talking to a TWAI peripheral.
 */
struct CanFrame {
    uint32_t id = 0;
    bool extended = false;
    uint8_t dlc = 0;
    uint8_t data[8] = {0};
};

enum class CanBusState : uint8_t {
    RUNNING,
    BUS_OFF,
    STOPPED,
    ERROR_WARNING
};

/**
 * @brief G3.2 -- Minimal CAN bus abstraction. HondaCANModule depends on this interface
 * only, never on driver/twai.h directly, so its UDS/protocol logic (request state
 * machine, NRC handling, ISO-TP frame checking, bus-off recovery) can be exercised on
 * a host machine (native unit tests, MockCanBus) with no ESP32 hardware attached.
 *
 * TwaiCanBus is the real ESP32-S3 TWAI-backed implementation used in production; it
 * wraps the exact same twai_* calls HondaCANModule used to call directly -- no runtime
 * behavior change on real hardware, only where the calls live. Both implementations
 * coexist; main.cpp decides which one HondaCANModule is constructed with.
 */
class ICanBus {
public:
    virtual ~ICanBus() {}

    /**
     * @brief Installs and starts the underlying CAN peripheral/driver.
     * @return true on success.
     */
    virtual bool begin() = 0;

    /**
     * @brief Transmits a single CAN frame. Non-blocking (or a very short internal
     * timeout on real hardware); never waits for a response.
     * @return true if the frame was queued/sent successfully.
     */
    virtual bool transmit(const CanFrame& frame) = 0;

    /**
     * @brief Non-blocking receive of one pending frame, if any.
     * @return true if a frame was available and written into `frame`.
     */
    virtual bool receive(CanFrame& frame) = 0;

    /**
     * @brief Current bus state (bus-off, stopped, running, error-warning).
     */
    virtual CanBusState getState() = 0;

    /**
     * @brief Current TWAI error counters, for bus-off diagnostics/logging.
     */
    virtual void getErrorCounters(uint16_t& txErrorCount, uint16_t& rxErrorCount) = 0;

    /**
     * @brief Requests bus-off recovery (equivalent to twai_initiate_recovery()).
     * @return true if the recovery request was accepted.
     */
    virtual bool initiateRecovery() = 0;

    /**
     * @brief (Re)starts the driver, e.g. after a completed bus-off recovery leaves it
     * in the STOPPED state (equivalent to twai_start()).
     * @return true on success.
     */
    virtual bool start() = 0;
};

#endif // I_CAN_BUS_H
