#include <Arduino.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>

#include "SystemState.h"
#include "IModule.h"
// D-021/D-023: while no moto-rt-core exists, this node is the TEMPORARY sole tester on
// the vehicle bus (its TWAI is wired to the CL250 DLC). CONN_VEHICLE_TESTER selects it:
//   1 = poll the ECU (only while rt-core does not poll; two testers are never allowed)
//   0 = poller off: the TWAI driver is never installed, so this node transmits nothing
//       on any CAN bus. Use it as soon as rt-core polls the ECU. Receiving rt-core's
//       republished values (platform.dbc VehicleSpeed 0x021 / VehicleEngine 0x110) is not
//       implemented yet, so the engine values then stay "stale" on BLE/Wi-Fi/Nextion.
#ifndef CONN_VEHICLE_TESTER
#error "CONN_VEHICLE_TESTER must be defined: 1 = temporary vehicle-bus tester (D-023), 0 = poller off (D-021)."
#endif

// D-058 item 4: CONN_CAN_LISTEN_ONLY=1 builds the CAN capture probe (env
// esp32-s3-devkitc-1-listen-only): TWAI in listen-only mode, every frame streamed over USB
// serial, nothing else running, no transmit code linked. It is a different firmware, so it
// excludes the tester (CONN_VEHICLE_TESTER=1) and the synthetic data source.
#ifndef CONN_CAN_LISTEN_ONLY
#define CONN_CAN_LISTEN_ONLY 0
#endif
#if CONN_CAN_LISTEN_ONLY && (CONN_VEHICLE_TESTER || defined(MOCK_CAN_DATA))
#error "CONN_CAN_LISTEN_ONLY=1 is incompatible with CONN_VEHICLE_TESTER=1 and MOCK_CAN_DATA: the capture build never transmits."
#endif

// USB serial speed. The listen-only env raises it (platformio.ini) for the frame stream.
#ifndef CONN_SERIAL_BAUD
#define CONN_SERIAL_BAUD 115200
#endif

#if CONN_CAN_LISTEN_ONLY
#include "CanCaptureModule.h"
#include "hal/TwaiListenOnlyRx.h"
#else
#if defined(MOCK_CAN_DATA)
#include "MockCANModule.h"
#elif CONN_VEHICLE_TESTER
#include "HondaCANModule.h"
#include "hal/TwaiCanBus.h"
#include "hal/RtcTesterLatchStore.h"
#else
#warning "CONN_VEHICLE_TESTER=0: vehicle poller disabled, no CAN transmission, no engine data source."
#endif
#include "NextionModule.h"
#include "BLEServerModule.h"
#include "ImuModule.h"
#include "WiFiServerModule.h"
#include "SerialLoggerModule.h"
#endif // CONN_CAN_LISTEN_ONLY

// ============================================================================
// HARDWARE PIN DEFINITIONS (ESP32-S3 DevKit C1)
// ============================================================================
#define CAN_TX_PIN      GPIO_NUM_4
#define CAN_RX_PIN      GPIO_NUM_5

#define UART2_TX_PIN    GPIO_NUM_17
#define UART2_RX_PIN    GPIO_NUM_18

// Free GPIO used purely for loop-timing observation (oscilloscope / logic analyzer probe).
// Not wired to any peripheral above (4,5,17,18 are taken; 1/2 are the IMU's I2C, see ImuModule).
#define DEBUG_LOOP_PIN  GPIO_NUM_8

// G1.4 -- Task Watchdog Timer. If loop() ever stalls (a module hangs) for longer than
// this, the TWDT panics and resets the board rather than leaving a dead unit riding.
#define TWDT_TIMEOUT_S  5

#if CONN_CAN_LISTEN_ONLY
// ============================================================================
// D-058 item 4 -- CAN capture probe (Q-001, VWP section 5.5 step 1)
// Only the capture module runs and only it writes to Serial (comment lines start with '#'):
// no Nextion, BLE, Wi-Fi, IMU, logger or G0.1 timing report, so the loop stays short and
// the stream clean. The driver is TWAI_MODE_LISTEN_ONLY behind the receive-only ICanRx.
// ============================================================================
// Serial TX ring buffer (framework driver, allocated once at Serial.begin) so a burst of
// frame lines does not hit the 128-byte UART FIFO; the capture never blocks on it.
const size_t SERIAL_TX_BUFFER_BYTES = 4096;

TwaiListenOnlyRx canRx(CAN_TX_PIN, CAN_RX_PIN);
CanCaptureModule captureModule(canRx);
SystemState globalState; // the producer interface passes it; the capture does not use it
bool captureActive = false;

void setup() {
    // Hold the transceiver's TXD recessive until the driver takes the pin (see the
    // normal build's setup()); in listen-only mode the controller never drives it low.
    gpio_set_level(CAN_TX_PIN, 1);
    gpio_config_t canTxHigh = {};
    canTxHigh.pin_bit_mask = 1ULL << CAN_TX_PIN;
    canTxHigh.mode = GPIO_MODE_OUTPUT;
    gpio_config(&canTxHigh);
    gpio_set_level(CAN_TX_PIN, 1);

    Serial.setTxBufferSize(SERIAL_TX_BUFFER_BYTES);
    Serial.begin(CONN_SERIAL_BAUD);
    delay(500);

    esp_task_wdt_init(TWDT_TIMEOUT_S, true /* panic + reset on timeout */);
    esp_task_wdt_add(NULL);

    captureActive = captureModule.begin();
    if (!captureActive) {
        Serial.println("# ERROR: the CAN listen-only driver failed to start; check the transceiver RX pin.");
    }
}

// The Arduino loop task (priority 1, core 1) never yields by itself, and the task watchdog
// also watches core 1's idle task (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1): a loop that
// only polls would starve it and reset the board after TWDT_TIMEOUT_S. So each pass waits
// up to CAPTURE_WAIT_MS for the next frame, blocked (the idle task runs; a frame wakes the
// loop at once, so the timestamps keep their precision). A bus so busy that the queue never
// empties still gets a one-tick delay every CAPTURE_FORCED_YIELD_MS; the 64-slot queue holds
// about 8 ms of a fully loaded 500 kbps bus.
const uint32_t CAPTURE_WAIT_MS = 1;
const uint32_t CAPTURE_FORCED_YIELD_MS = 1000;
uint32_t lastYieldMs = 0;

void loop() {
    if (captureActive) {
        captureModule.update(globalState);
    }
    esp_task_wdt_reset();
    const uint32_t now = millis();
    if (canRx.waitForFrame(CAPTURE_WAIT_MS)) {
        lastYieldMs = now;
    } else if (now - lastYieldMs >= CAPTURE_FORCED_YIELD_MS) {
        vTaskDelay(1);
        lastYieldMs = now;
    }
}
#else // normal builds

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
#if defined(MOCK_CAN_DATA)
MockCANModule      canModule;
#elif CONN_VEHICLE_TESTER
TwaiCanBus         canBus(CAN_TX_PIN, CAN_RX_PIN);
RtcTesterLatchStore canLatchStore; // Q-018: D-030 latch survives non-power-on resets
HondaCANModule     canModule(canBus, canLatchStore);
#endif
NextionModule      displayModule(NextionSerial, UART2_RX_PIN, UART2_TX_PIN);
// D-032: raw 100 Hz IMU samples for data collection. The sampler task (core 0) fills this
// static ring; BLEServerModule drains it in 10-sample blocks. Independent of the CAN
// poller and present in every build (no vehicle-bus involvement).
ImuRing            imuRing;
ImuModule          imuModule(imuRing);
BLEServerModule    bleModule(&imuRing);
WiFiServerModule   wifiModule(80);      // SoftAP HTTP JSON Backend Server on port 80
SerialLoggerModule loggerModule(1000); // Prints serial log every 1000ms

// G3.1 -- Producer/consumer module arrays, not one polymorphic IModule[]. Producers
// (CAN, BLE) are the only modules that ever write into SystemState; consumers
// (Nextion, WiFi, Logger) get a const SystemState& so the compiler rejects any
// accidental write. Producers run first each pass so every consumer in that same
// pass sees that pass's freshest data (see PR discussion: this is not an added-latency
// change, it removes a pre-existing one-loop staleness gap for BLE-writen telematics
// reaching Nextion, since BLE used to run after Nextion in the old single array).
IProducerModule* producers[] = {
#if defined(MOCK_CAN_DATA) || CONN_VEHICLE_TESTER
    &canModule,
#endif
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
const char* PRODUCER_NAMES[PRODUCER_COUNT] = {
#if defined(MOCK_CAN_DATA) || CONN_VEHICLE_TESTER
    "CAN",
#endif
    "IMU", "BLE"};
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
// from update() so one broken peripheral (e.g. Nextion not wired) cannot stall
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
// (CAN/UDS timing, WiFi HTTP responsiveness, BLE queue draining)
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
    // First thing after every reset, in every build: hold the transceiver's TXD recessive
    // (high) so an undriven pin can never pull the vehicle bus dominant. The tester build
    // hands the pin to TWAI only when HondaCANModule installs the driver (listen-only
    // first, Q-018); a latched or no-tester build never does. The board should also have
    // a pull-up on TXD for the time before this line runs. ESP-IDF calls, level first:
    // Arduino's digitalWrite() ignores a pin not yet set up, so pinMode() alone would
    // drive the reset value (low, dominant) for a moment.
    gpio_set_level(CAN_TX_PIN, 1);
    gpio_config_t canTxHigh = {};
    canTxHigh.pin_bit_mask = 1ULL << CAN_TX_PIN;
    canTxHigh.mode = GPIO_MODE_OUTPUT;
    gpio_config(&canTxHigh);
    gpio_set_level(CAN_TX_PIN, 1);
    // Initialize USB Serial Debug Console
    Serial.begin(CONN_SERIAL_BAUD);
    delay(500);

    Serial.println("\n==================================================");
    Serial.println("   MOTO-CONNECTIVITY-NODE (CL250 TELEMETRY) START  ");
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
#if !defined(MOCK_CAN_DATA) && CONN_VEHICLE_TESTER
    if (!producerActive[0]) {
        // Tester build whose TWAI driver failed to start: report it as stopped so BLE v3
        // CAN health does not look like the poller-off build (NOT_INSTALLED, no flags).
        globalState.can.busState = CanHealthState::STOPPED;
        globalState.can.flags = CAN_HEALTH_FLAG_POLLER_ENABLED;
    }
#endif
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
        // One-time-init heap only is the goal; BLE notifications still allocate inside the
        // framework, so watch the low-water mark and fragmentation during soak tests.
        Serial.printf("  [HEAP    ] free=%lu  min_free=%lu  largest_block=%lu\n",
            (unsigned long)esp_get_free_heap_size(),
            (unsigned long)esp_get_minimum_free_heap_size(),
            (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        Serial.println("-----------------------------------------------\n");
    }
}

#endif // CONN_CAN_LISTEN_ONLY
