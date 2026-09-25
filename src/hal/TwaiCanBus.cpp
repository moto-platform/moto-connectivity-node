#include "TwaiCanBus.h"

TwaiCanBus::TwaiCanBus(gpio_num_t txPin, gpio_num_t rxPin)
    : _txPin(txPin), _rxPin(rxPin) {}

bool TwaiCanBus::begin() {
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ABOVE_ERR_WARN;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    return twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK && twai_start() == ESP_OK;
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
