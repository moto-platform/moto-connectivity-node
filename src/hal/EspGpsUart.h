#ifndef ESP_GPS_UART_H
#define ESP_GPS_UART_H

#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "IGpsUart.h"

/**
 * @brief IGpsUart on the ESP-IDF UART driver (D-060). begin() must run in the GPS task on
 * core 0 so the UART interrupt lands there too (the ImuModule precedent). The driver
 * allocates its RX ring buffer and event queue once in begin(); nothing is allocated after
 * that. Raw GPS bytes stay in that ring buffer and the caller's read chunk only: they are
 * never logged or mirrored (D-060 item 3, invariant 7).
 */
class EspGpsUart : public IGpsUart {
public:
    EspGpsUart(uart_port_t port, int rxPin, int txPin) : _port(port), _rxPin(rxPin), _txPin(txPin) {}

    bool begin(uint32_t baud);
    bool setBaud(uint32_t baud) override;
    bool write(const uint8_t* data, size_t len) override;
    void waitTxDone(uint32_t timeoutMs) override;
    size_t read(uint8_t* out, size_t cap, uint32_t timeoutMs) override;
    bool takeOverflow() override;
    uint32_t nowMs() override;

private:
    uart_port_t _port;
    int _rxPin;
    int _txPin;
    bool _installed = false;
    QueueHandle_t _events = nullptr;
};

#endif // ESP_GPS_UART_H
