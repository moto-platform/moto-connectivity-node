// Only the GPS build (CONN_GPS=1) has a GPS UART; nothing of this file is compiled otherwise.
#if defined(CONN_GPS) && CONN_GPS

#include "EspGpsUart.h"

#include <esp_timer.h>

namespace {
// 10 Hz NAV-PVT = 1000 B/s at 38400 baud (~3840 B/s line): 1 KiB holds ~1 s of data if
// the task stalls. ESP-IDF requires an RX buffer larger than the hardware FIFO (128 B).
const int kRxBufferBytes = 1024;
const int kEventQueueLen = 8;
} // namespace

bool EspGpsUart::begin(uint32_t baud) {
    uart_config_t cfg = {};
    cfg.baud_rate = (int)baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;
    // TX buffer 0: uart_write_bytes() blocks until the bytes are in the FIFO (config
    // frames only, a few dozen bytes at start-up).
    if (uart_driver_install(_port, kRxBufferBytes, 0, kEventQueueLen, &_events, 0) != ESP_OK) {
        return false;
    }
    _installed = true;
    if (uart_param_config(_port, &cfg) != ESP_OK ||
        uart_set_pin(_port, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        uart_driver_delete(_port);
        _installed = false;
        return false;
    }
    return true;
}

bool EspGpsUart::setBaud(uint32_t baud) {
    return _installed && uart_set_baudrate(_port, baud) == ESP_OK;
}

bool EspGpsUart::write(const uint8_t* data, size_t len) {
    return _installed && uart_write_bytes(_port, data, len) == (int)len;
}

void EspGpsUart::waitTxDone(uint32_t timeoutMs) {
    if (_installed) {
        uart_wait_tx_done(_port, pdMS_TO_TICKS(timeoutMs));
    }
}

size_t EspGpsUart::read(uint8_t* out, size_t cap, uint32_t timeoutMs) {
    if (!_installed) {
        vTaskDelay(pdMS_TO_TICKS(timeoutMs));
        return 0u;
    }
    int n = uart_read_bytes(_port, out, (uint32_t)cap, pdMS_TO_TICKS(timeoutMs));
    return n > 0 ? (size_t)n : 0u;
}

bool EspGpsUart::takeOverflow() {
    bool overflow = false;
    uart_event_t event;
    while (_events != nullptr && xQueueReceive(_events, &event, 0) == pdTRUE) {
        if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
            overflow = true;
        }
    }
    if (overflow) {
        uart_flush_input(_port); // drop the partial data; the parser resyncs
    }
    return overflow;
}

uint32_t EspGpsUart::nowMs() {
    return (uint32_t)(esp_timer_get_time() / 1000); // same clock as millis()
}

#endif // CONN_GPS
