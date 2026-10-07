// Only the GPS build (CONN_GPS=1, env esp32-s3-devkitc-1-gps) has the GPS; nothing of this
// file is compiled otherwise.
#if defined(CONN_GPS) && CONN_GPS

#if !defined(CONN_GPS_RX_PIN) || !defined(CONN_GPS_TX_PIN)
#error "CONN_GPS=1 needs CONN_GPS_RX_PIN and CONN_GPS_TX_PIN (hardware-integration.md, pins CONFIRM)."
#endif
#ifndef CONN_GPS_PINS_CONFIRMED
#define CONN_GPS_PINS_CONFIRMED 0
#endif

#include "GpsModule.h"

#include <Arduino.h>

#include "hal/EspGpsUart.h"

namespace {
// GPS task: static stack/TCB (no heap), pinned to core 0 like the IMU sampler so it never
// preempts the Arduino loop (core 1, CAN poller); priority above idle, below the BT/Wi-Fi
// stacks. Not subscribed to the task watchdog: it blocks in uart_read_bytes by design.
const uint32_t kTaskStackBytes = 3072;
const UBaseType_t kTaskPriority = 5;
const BaseType_t kTaskCore = 0;
const uint32_t kRetryMs = 10000; // no receiver answered: try again after this
const unsigned long kReportPeriodMs = 10000;
// Pins come from the build env and stay CONFIRM items (hardware-integration.md §10 item 5)
// until CONN_GPS_PINS_CONFIRMED=1: before that the UART is never installed.
const bool kPinsConfirmed = CONN_GPS_PINS_CONFIRMED != 0;
StackType_t s_taskStack[kTaskStackBytes];
StaticTask_t s_taskTcb;
// UART1: UART0 is the USB console, UART2 the Nextion (HardwareSerial(2)).
EspGpsUart s_uart(UART_NUM_1, CONN_GPS_RX_PIN, CONN_GPS_TX_PIN);

const char* linkName(uint8_t link) {
    switch (link) {
    case GpsCore::LINK_OK: return "OK";
    case GpsCore::LINK_NO_RECEIVER: return "NO_RECEIVER";
    default: return "PENDING";
    }
}
} // namespace

bool GpsModule::begin() {
    if (!kPinsConfirmed) {
        Serial.printf("[GPS] Pins GPIO%d/GPIO%d not confirmed (CONN_GPS_PINS_CONFIRMED=0) -- "
                      "GPS disabled, UART untouched.\n", CONN_GPS_RX_PIN, CONN_GPS_TX_PIN);
        return false;
    }
    _task = xTaskCreateStaticPinnedToCore(taskEntry, "gps", kTaskStackBytes, this, kTaskPriority,
                                          s_taskStack, &s_taskTcb, kTaskCore);
    if (_task == nullptr) {
        Serial.println("[GPS] Task creation failed -- GPS disabled.");
        return false;
    }
    Serial.printf("[GPS] Task started: UART1 RX GPIO%d, TX GPIO%d, UBX NAV-PVT at %lu baud.\n",
                  CONN_GPS_RX_PIN, CONN_GPS_TX_PIN, (unsigned long)ubx::kGpsBaud);
    _started = true;
    return true;
}

void GpsModule::taskEntry(void* arg) {
    static_cast<GpsModule*>(arg)->taskLoop();
}

// GPS task, core 0: the UART driver (and its interrupt) is installed from here.
void GpsModule::taskLoop() {
    if (!s_uart.begin(ubx::kModuleDefaultBaud)) {
        _uartFailed.store(true);
        vTaskDelete(nullptr);
        return;
    }
    for (;;) {
        if (!_core.step(s_uart)) {
            vTaskDelay(pdMS_TO_TICKS(kRetryMs));
        }
    }
}

void GpsModule::update(SystemState& state) {
    GpsStatus& gps = state.gps;
    gps.link = (uint8_t)_core.link();
    GpsFix fix;
    if (_core.latest(fix)) {
        gps.fix = fix;
        gps.hasFix = true;
    }
    gps.navPvtCount = _core.navPvtCount();
    gps.checksumErrors = _core.checksumErrors();
    gps.lengthErrors = _core.lengthErrors();
    gps.uartOverflows = _core.uartOverflows();
    gps.linkLosses = _core.linkLosses();

    // Bring-up report: rate, fix quality and error counters only, never speed, heading or
    // raw bytes. Measures whether 10 Hz holds with the receiver's GNSS set (D-060 item 2).
    unsigned long now = millis();
    if (now - _lastReportMs >= kReportPeriodMs) {
        uint32_t pvts = gps.navPvtCount - _lastReportPvt;
        Serial.printf("[GPS] link=%s%s pvt/s=%lu.%lu fix=%u sv=%u ck_err=%lu len_err=%lu ovf=%lu lost=%lu\n",
                      linkName(gps.link), _uartFailed.load() ? " (UART install failed)" : "",
                      (unsigned long)(pvts * 1000u / kReportPeriodMs),
                      (unsigned long)((pvts * 10000u / kReportPeriodMs) % 10u),
                      gps.fix.fixType, gps.fix.numSv, (unsigned long)gps.checksumErrors,
                      (unsigned long)gps.lengthErrors, (unsigned long)gps.uartOverflows,
                      (unsigned long)gps.linkLosses);
        _lastReportMs = now;
        _lastReportPvt = gps.navPvtCount;
    }
}

#endif // CONN_GPS
