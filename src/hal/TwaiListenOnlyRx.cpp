// Only the listen-only capture build (CONN_CAN_LISTEN_ONLY) uses the TWAI driver through
// this class; no other env compiles it.
#if defined(CONN_CAN_LISTEN_ONLY) && CONN_CAN_LISTEN_ONLY

#include "TwaiListenOnlyRx.h"

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

bool TwaiListenOnlyRx::receive(CanFrame& frame) {
    twai_message_t rxMsg;
    if (!_installed || twai_receive(&rxMsg, 0) != ESP_OK) {
        return false;
    }
    frame.id = rxMsg.identifier;
    frame.extended = rxMsg.extd != 0;
    frame.dlc = rxMsg.data_length_code > 8 ? 8 : rxMsg.data_length_code;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = rxMsg.data[i];
    }
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
