#include <Arduino.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

#include "SystemState.h"
#include "IModule.h"
#ifdef MOCK_CAN_DATA
#include "MockCANModule.h"
#else
#include "HondaCANModule.h"
#include "hal/TwaiCanBus.h"
#endif
#include "IMUModule.h"
#include "NextionModule.h"
#include "BLEServerModule.h"
#include "WiFiServerModule.h"
#include "SerialLoggerModule.h"

// ============================================================================
// HARDWARE PIN DEFINITIONS (ESP32-S3 DevKit C1)
// ============================================================================
#define CAN_TX_PIN      GPIO_NUM_4
#define CAN_RX_PIN      GPIO_NUM_5

#define I2C_SDA_PIN     GPIO_NUM_1
#define I2C_SCL_PIN     GPIO_NUM_2

#define UART2_TX_PIN    GPIO_NUM_17
#define UART2_RX_PIN    GPIO_NUM_18

// Free GPIO used purely for loop-timing observation (oscilloscope / logic analyzer probe).
// Not wired to any peripheral above (4,5,1,2,17,18 are taken).
#define DEBUG_LOOP_PIN  GPIO_NUM_8

// G1.4 -- Task Watchdog Timer. If loop() ever stalls (a module hangs) for longer than
// this, the TWDT panics and resets the board rather than leaving a dead unit riding.
#define TWDT_TIMEOUT_S  5

// Hardware Serial 2 Instance for Nextion HMI Display
HardwareSerial NextionSerial(2);

// ============================================================================
// CENTRAL TELEMETRY STATE STORE
// ============================================================================
SystemState globalState;

// ============================================================================
// SYSTEM MODULE INSTANTIATIONS
// ============================================================================
// G3.2-alt -- MOCK_CAN_DATA build flag (see platformio.ini env:esp32-s3-devkitc-1-mock)
// swaps in MockCANModule (synthetic telemetry, no ECU/CAN hardware) so the rest of the
// system -- Nextion, BLE, WiFi, staleness, watchdog -- can be exercised on real ESP32
// hardware without a Honda ECU. HondaCANModule's own protocol logic is never touched
// by this; it's a separate module entirely.
//
// G3.2 -- On real hardware, HondaCANModule is injected with a TwaiCanBus (the actual
// TWAI-backed ICanBus implementation). Its UDS/protocol logic never calls driver/twai.h
// directly, so the exact same HondaCANModule.cpp also runs unit-tested against a
// MockCanBus in test/test_can_protocol, with no ESP32 device attached.
#ifdef MOCK_CAN_DATA
MockCANModule      canModule;
#else
TwaiCanBus         canBus(CAN_TX_PIN, CAN_RX_PIN);
HondaCANModule     canModule(canBus);
#endif
IMUModule          imuModule(I2C_SDA_PIN, I2C_SCL_PIN);
NextionModule      displayModule(NextionSerial, UART2_RX_PIN, UART2_TX_PIN);
BLEServerModule    bleModule;
WiFiServerModule   wifiModule(80);      // SoftAP HTTP JSON Backend Server on port 80
SerialLoggerModule loggerModule(1000); // Prints serial log every 1000ms

// G3.1 -- Producer/consumer module arrays, not one polymorphic IModule[]. Producers
// (CAN, IMU, BLE) are the only modules that ever write into SystemState; consumers
// (Nextion, WiFi, Logger) get a const SystemState& so the compiler rejects any
// accidental write. Producers run first each pass so every consumer in that same
// pass sees that pass's freshest data (see PR discussion: this is not an added-latency
// change, it removes a pre-existing one-loop staleness gap for BLE-writen telematics
// reaching Nextion, since BLE used to run after Nextion in the old single array).
IProducerModule* producers[] = {
    &canModule,
    &imuModule,
    &bleModule,
};
IConsumerModule* consumers[] = {
    &displayModule,
    &wifiModule,
    &loggerModule,
};

const uint8_t PRODUCER_COUNT = sizeof(producers) / sizeof(producers[0]);
const uint8_t CONSUMER_COUNT = sizeof(consumers) / sizeof(consumers[0]);
const char* PRODUCER_NAMES[PRODUCER_COUNT] = {"CAN", "IMU", "BLE"};
const char* CONSUMER_NAMES[CONSUMER_COUNT] = {"Nextion", "WiFi", "Logger"};

// ============================================================================
// G0.1 -- LOOP TIMING INSTRUMENTATION
// Tracks per-module update() duration (esp_timer_get_time, microsecond resolution)
// and prints min/avg/max to Serial every 10s. DEBUG_LOOP_PIN pulses HIGH for the
// duration of one full loop() pass so it can be probed with a scope/logic analyzer.
// ============================================================================
struct TimingStats {
    uint32_t minUs = UINT32_MAX;
    uint32_t maxUs = 0;
    uint64_t sumUs = 0;
    uint32_t samples = 0;

    void record(uint32_t us) {
        if (us < minUs) minUs = us;
        if (us > maxUs) maxUs = us;
        sumUs += us;
        samples++;
    }

    void reset() { *this = TimingStats(); }
};

TimingStats producerTiming[PRODUCER_COUNT];
TimingStats consumerTiming[CONSUMER_COUNT];
TimingStats loopTiming;
unsigned long lastTimingReport = 0;
const unsigned long TIMING_REPORT_INTERVAL_MS = 10000;

// ============================================================================
// G0.2 -- MODULE HEALTH TRACKING
// A module whose begin() fails (or that later reports unhealthy) is excluded
// from update() so one broken peripheral (e.g. IMU not wired) cannot stall
// or crash the modules that are working.
// ============================================================================
bool producerActive[PRODUCER_COUNT];
bool consumerActive[CONSUMER_COUNT];

// ============================================================================
// G3.4 -- CENTRALIZED SCHEDULING
// Each module declares its own required cadence via getPeriodMs() (IModule.h);
// main.cpp is the single place that decides, from that declaration, whether this
// pass is due to call update() -- replacing the old pattern of every module doing
// its own internal millis() comparison. Modules that need every-pass execution
// (CAN/UDS timing, IMU sample rate, WiFi HTTP responsiveness, BLE queue draining)
// simply report period 0 (IModule's default) and are unaffected.
// ============================================================================
unsigned long producerLastRunMs[PRODUCER_COUNT] = {0};
unsigned long consumerLastRunMs[CONSUMER_COUNT] = {0};

// ============================================================================
// Small shared helpers -- refactored out to deduplicate the producer/consumer
// loops below (begin+log, health-check+log, timing-row printing were each
// repeated once per group with identical logic).
// ============================================================================
bool beginAndLog(IModule* mod, const char* roleLabel, uint8_t index, const char* name) {
    bool ok = mod->begin();
    if (ok) {
        Serial.printf(" [OK] %s [%d] %s successfully initialized.\n", roleLabel, index, name);
    } else {
        Serial.printf(" [WARNING] %s [%d] %s failed to initialize -- %s unavailable, excluded from update loop.\n",
            roleLabel, index, name, name);
    }
    return ok;
}

bool checkHealthTransition(IModule* mod, bool& activeFlag, const char* roleLabel, uint8_t index, const char* name) {
    bool healthyNow = mod->isHealthy();
    if (healthyNow != activeFlag) {
        activeFlag = healthyNow;
        Serial.printf(" [HEALTH] %s [%d] %s is now %s.\n",
            roleLabel, index, name, healthyNow ? "healthy" : "unhealthy -- excluded from update loop");
    }
    return activeFlag;
}

// G3.4 -- centralized scheduling check: true (and advances lastRunMs) if `periodMs`
// has elapsed since this module's last run, or if periodMs is 0 (every pass).
bool isDue(unsigned long now, unsigned long& lastRunMs, uint32_t periodMs) {
    if (periodMs > 0 && (now - lastRunMs) < periodMs) {
        return false;
    }
    lastRunMs = now;
    return true;
}

void printTimingRow(const char* name, TimingStats& t) {
    if (t.samples > 0) {
        Serial.printf("  [%-8s] min=%6lu  avg=%6lu  max=%6lu  (n=%lu)\n",
            name,
            (unsigned long)t.minUs,
            (unsigned long)(t.sumUs / t.samples),
            (unsigned long)t.maxUs,
            (unsigned long)t.samples);
    }
    t.reset();
}

// ============================================================================
// G1.4 -- helper to name a reset reason for the boot log
// ============================================================================
const char* resetReasonName(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXTERNAL_PIN";
        case ESP_RST_SW:        return "SOFTWARE";
        case ESP_RST_PANIC:     return "PANIC (exception)";
        case ESP_RST_INT_WDT:   return "INTERRUPT_WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG (loop stalled)";
        case ESP_RST_WDT:       return "OTHER_WATCHDOG";
        case ESP_RST_BROWNOUT:  return "BROWNOUT (power dip)";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

// ============================================================================
// SETUP & MAIN LOOP
// ============================================================================
void setup() {
    // Initialize USB Serial Debug Console
    Serial.begin(115200);
    delay(500);

    Serial.println("\n==================================================");
    Serial.println("   HONDA CL250 DUAL-TRANSPORT TELEMETRY STARTING  ");
    Serial.println("==================================================");
    Serial.printf(" [BOOT] Reset reason: %s\n", resetReasonName(esp_reset_reason()));

    // G1.4: enable Task Watchdog Timer and subscribe the main loop task to it.
    esp_task_wdt_init(TWDT_TIMEOUT_S, true /* panic + reset on timeout */);
    esp_task_wdt_add(NULL);

    pinMode(DEBUG_LOOP_PIN, OUTPUT);
    digitalWrite(DEBUG_LOOP_PIN, LOW);

    // Initialize all registered system modules (producers first, then consumers).
    for (uint8_t i = 0; i < PRODUCER_COUNT; i++) {
        producerActive[i] = beginAndLog(producers[i], "Producer", i, PRODUCER_NAMES[i]);
    }
    for (uint8_t i = 0; i < CONSUMER_COUNT; i++) {
        consumerActive[i] = beginAndLog(consumers[i], "Consumer", i, CONSUMER_NAMES[i]);
    }

    Serial.println(" [SYSTEM] Setup completed. Dual BLE + Wi-Fi active.\n");
}

void loop() {
    digitalWrite(DEBUG_LOOP_PIN, HIGH);
    int64_t loopStartUs = esp_timer_get_time();
    unsigned long now = millis();

    // G3.1 -- producers run first so every consumer below sees this pass's freshest
    // data. Modules that failed begin() (or later report unhealthy) are skipped so one
    // broken peripheral cannot stall the ones that are working (G0.2). G3.4 -- each
    // module's own getPeriodMs() decides whether it's actually due this pass.
    for (uint8_t i = 0; i < PRODUCER_COUNT; i++) {
        if (!checkHealthTransition(producers[i], producerActive[i], "Producer", i, PRODUCER_NAMES[i])) {
            continue;
        }
        if (!isDue(now, producerLastRunMs[i], producers[i]->getPeriodMs())) {
            continue;
        }

        int64_t moduleStartUs = esp_timer_get_time();
        producers[i]->update(globalState);
        producerTiming[i].record((uint32_t)(esp_timer_get_time() - moduleStartUs));
    }

    for (uint8_t i = 0; i < CONSUMER_COUNT; i++) {
        if (!checkHealthTransition(consumers[i], consumerActive[i], "Consumer", i, CONSUMER_NAMES[i])) {
            continue;
        }
        if (!isDue(now, consumerLastRunMs[i], consumers[i]->getPeriodMs())) {
            continue;
        }

        int64_t moduleStartUs = esp_timer_get_time();
        consumers[i]->update(globalState); // implicit SystemState -> const SystemState&
        consumerTiming[i].record((uint32_t)(esp_timer_get_time() - moduleStartUs));
    }

    loopTiming.record((uint32_t)(esp_timer_get_time() - loopStartUs));
    digitalWrite(DEBUG_LOOP_PIN, LOW);

    // G1.4: feed the watchdog once per completed loop pass. If any module hangs
    // (blocking I/O, infinite loop) this stops happening and TWDT resets the board.
    esp_task_wdt_reset();

    if (now - lastTimingReport >= TIMING_REPORT_INTERVAL_MS) {
        lastTimingReport = now;
        Serial.println("\n---- LOOP TIMING (last 10s, microseconds) ----");
        for (uint8_t i = 0; i < PRODUCER_COUNT; i++) {
            printTimingRow(PRODUCER_NAMES[i], producerTiming[i]);
        }
        for (uint8_t i = 0; i < CONSUMER_COUNT; i++) {
            printTimingRow(CONSUMER_NAMES[i], consumerTiming[i]);
        }
        printTimingRow("LOOP", loopTiming);
        Serial.println("-----------------------------------------------\n");
    }
}