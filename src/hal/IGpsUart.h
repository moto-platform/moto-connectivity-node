#ifndef I_GPS_UART_H
#define I_GPS_UART_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief The serial link to the GPS receiver (D-060), as GpsCore needs it. EspGpsUart
 * implements it on the ESP-IDF UART driver; the native tests use a scripted mock.
 * Only the GPS UART: nothing here can reach a CAN bus.
 */
class IGpsUart {
public:
    virtual ~IGpsUart() {}
    // Sets the line rate (8N1). Returns false if the driver refused it.
    virtual bool setBaud(uint32_t baud) = 0;
    // Queues bytes for transmission; returns false if not all were accepted.
    virtual bool write(const uint8_t* data, size_t len) = 0;
    // Blocks until the queued bytes have left the wire (or timeoutMs passed).
    virtual void waitTxDone(uint32_t timeoutMs) = 0;
    // Reads up to `cap` bytes, waiting at most timeoutMs; returns the count (0 on timeout).
    virtual size_t read(uint8_t* out, size_t cap, uint32_t timeoutMs) = 0;
    // True once if the receive buffer overflowed since the previous call (bytes lost).
    virtual bool takeOverflow() = 0;
    // Node clock, ms (millis() on the target).
    virtual uint32_t nowMs() = 0;
};

#endif // I_GPS_UART_H
