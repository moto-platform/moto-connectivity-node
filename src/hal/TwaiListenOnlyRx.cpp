// Only the listen-only capture build (CONN_CAN_LISTEN_ONLY) uses the TWAI driver through
// this class; no other env compiles it.
#if defined(CONN_CAN_LISTEN_ONLY) && CONN_CAN_LISTEN_ONLY

#include "TwaiListenOnlyRx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
// Deeper than the IDF default (5): a loop pass with a serial write must not lose frames.
// 64 x ~16 B.
const uint32_t kRxQueueLen = 64;
} // namespace

TwaiListenOnlyRx::TwaiListenOnlyRx(gpio_num_t txPin, gpio_num_t rxPin)
    : _txPin(txPin), _rxPin(rxPin) {}

bool TwaiListenOnlyRx::begin() {
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_LISTEN_ONLY);
    g_config.rx_queue_len = kRxQueueLen;
    g_config.alerts_enabled = TWAI_ALERT_NONE;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    _installed = true;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall(); // never leave a half-started driver behind
        _installed = false;
        return false;
    }
    return true;
}

void TwaiListenOnlyRx::copyFrame(const twai_message_t& msg, CanFrame& frame) {
    frame.id = msg.identifier;
    frame.extended = msg.extd != 0;
    frame.dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = msg.data[i];
    }
}

bool TwaiListenOnlyRx::receive(CanFrame& frame) {
    if (_held) {
        _held = false;
        copyFrame(_heldMsg, frame);
        return true;
    }
    twai_message_t rxMsg;
    if (!_installed || twai_receive(&rxMsg, 0) != ESP_OK) {
        return false;
    }
    copyFrame(rxMsg, frame);
    return true;
}

bool TwaiListenOnlyRx::waitForFrame(uint32_t timeoutMs) {
    if (_held) {
        return false;
    }
    if (!_installed) {
        vTaskDelay(pdMS_TO_TICKS(timeoutMs));
        return true;
    }
    if (twai_receive(&_heldMsg, 0) == ESP_OK) {
        _held = true;
        return false; // a frame was already queued: no wait
    }
    TickType_t ticks = pdMS_TO_TICKS(timeoutMs);
    if (ticks == 0) {
        ticks = 1;
    }
    _held = twai_receive(&_heldMsg, ticks) == ESP_OK;
    return true;
}

uint32_t TwaiListenOnlyRx::rxLostCount() {
    twai_status_info_t status;
    if (!_installed || twai_get_status_info(&status) != ESP_OK) {
        return 0;
    }
    return status.rx_missed_count + status.rx_overrun_count;
}

#endif // CONN_CAN_LISTEN_ONLY
