// The listen-only capture build (D-058 item 4, CONN_CAN_LISTEN_ONLY) must not contain
// code that can transmit: nothing of this file is compiled there.
#if !defined(CONN_CAN_LISTEN_ONLY) || !CONN_CAN_LISTEN_ONLY

#include "TwaiCanBus.h"
#include "CanRxQueue.h"
#include "driver/gpio.h"

TwaiCanBus::TwaiCanBus(gpio_num_t txPin, gpio_num_t rxPin)
    : _txPin(txPin), _rxPin(rxPin) {}

bool TwaiCanBus::install(twai_mode_t mode) {
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, mode);
    g_config.rx_queue_len = kCanRxQueueLen; // CanRxQueue.h: 64 in the D-059 probe, else 32
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    _installed = true;
    if (twai_start() != ESP_OK) {
        uninstall(); // never leave a half-started driver behind
        return false;
    }
    return true;
}

void TwaiCanBus::uninstall() {
    if (_installed) {
        twai_stop(); // fails harmlessly if already stopped (e.g. bus-off)
        twai_driver_uninstall();
        _installed = false;
    }
}

bool TwaiCanBus::beginListenOnly() {
    return install(TWAI_MODE_LISTEN_ONLY);
}

bool TwaiCanBus::enterNormalMode() {
    uninstall();
    return install(TWAI_MODE_NORMAL);
}

bool TwaiCanBus::transmit(const CanFrame& frame) {
    twai_message_t txMsg;
    txMsg.extd = frame.extended ? 1 : 0;
    txMsg.rtr = 0;
    txMsg.identifier = frame.id;
    txMsg.data_length_code = frame.dlc;
    for (int i = 0; i < 8; i++) {
        txMsg.data[i] = frame.data[i];
    }

    return twai_transmit(&txMsg, pdMS_TO_TICKS(5)) == ESP_OK;
}

bool TwaiCanBus::receive(CanFrame& frame) {
    twai_message_t rxMsg;
    if (twai_receive(&rxMsg, 0) != ESP_OK) {
        return false;
    }

    frame.id = rxMsg.identifier;
    frame.extended = rxMsg.extd;
    frame.dlc = rxMsg.data_length_code;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = rxMsg.data[i];
    }
    return true;
}

CanBusState TwaiCanBus::getState() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) {
        // Driver not installed/queryable -- treat as stopped so callers attempt start().
        return CanBusState::STOPPED;
    }

    switch (status.state) {
        case TWAI_STATE_BUS_OFF: return CanBusState::BUS_OFF;
        case TWAI_STATE_STOPPED: return CanBusState::STOPPED;
        default:                 return CanBusState::RUNNING;
    }
}

uint32_t TwaiCanBus::rxLostCount() {
    twai_status_info_t status;
    if (twai_get_status_info(&status) != ESP_OK) {
        return 0;
    }
    return status.rx_missed_count + status.rx_overrun_count;
}

void TwaiCanBus::getErrorCounters(uint16_t& txErrorCount, uint16_t& rxErrorCount) {
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        txErrorCount = status.tx_error_counter;
        rxErrorCount = status.rx_error_counter;
    } else {
        txErrorCount = 0;
        rxErrorCount = 0;
    }
}

bool TwaiCanBus::initiateRecovery() {
    return twai_initiate_recovery() == ESP_OK;
}

bool TwaiCanBus::start() {
    return twai_start() == ESP_OK;
}

void TwaiCanBus::stop() {
    uninstall();
    // Take the pin back from the TWAI peripheral and hold the transceiver's TXD
    // recessive (high). Level first, so switching to output never drives it low.
    gpio_set_level(_txPin, 1);
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << _txPin;
    io.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io);
    gpio_set_level(_txPin, 1);
}

#endif // !CONN_CAN_LISTEN_ONLY
