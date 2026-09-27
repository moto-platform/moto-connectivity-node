#include "ImuModule.h"

#include <driver/i2c.h>

namespace {

// Wiring of the legacy port (GPIO 1/2 are free since the complementary filter was dropped).
const i2c_port_t kI2cPort = I2C_NUM_0;
const gpio_num_t kSdaPin = GPIO_NUM_1;
const gpio_num_t kSclPin = GPIO_NUM_2;
const uint32_t kI2cClockHz = 400000;
const TickType_t kI2cTimeoutTicks = pdMS_TO_TICKS(5); // 14-byte burst read takes ~0.4 ms

// MPU-6050 register map (InvenSense PS-MPU-6000A / RM-MPU-6000A). Sensor datasheet
// constants, not vehicle signals.
const uint8_t kAddress = 0x68;
const uint8_t kRegSmplrtDiv = 0x19;
const uint8_t kRegConfig = 0x1A;
const uint8_t kRegGyroConfig = 0x1B;
const uint8_t kRegAccelConfig = 0x1C;
const uint8_t kRegAccelXoutH = 0x3B; // 14 bytes: accel xyz, temp, gyro xyz (big-endian)
const uint8_t kRegPwrMgmt1 = 0x6B;
const uint8_t kRegWhoAmI = 0x75;

const uint8_t kPwrReset = 0x80;
const uint8_t kPwrClockPllGyroX = 0x01; // also clears SLEEP (the power-on default is asleep)
const uint8_t kSmplrtDiv1kHz = 0x00;    // internal 1 kHz; we read the latest value at 100 Hz
const uint8_t kConfigDlpf44Hz = 0x03;   // anti-alias below the 50 Hz Nyquist of our 100 Hz reads
const uint8_t kGyroFs500Dps = 0x08;     // FS_SEL=1: 65.5 LSB/(deg/s), schema imuBlock.scale.gyro
const uint8_t kAccelFs8g = 0x10;        // AFS_SEL=2: 4096 LSB/g, schema imuBlock.scale.accel
const uint8_t kFullScaleMask = 0x18;    // FS_SEL / AFS_SEL bits (self-test bits ignored)

// WHO_AM_I of register-compatible parts: MPU-6050, MPU-6500, MPU-9250, MPU-9255.
bool isKnownWhoAmI(uint8_t id) { return id == 0x68 || id == 0x70 || id == 0x71 || id == 0x73; }

const int64_t kSamplePeriodUs = 10000; // 100 Hz, schema imuBlock.samplePeriodMs
// Configuration check cadence (samples) and the failed-read streak that forces a
// re-configuration: a brown-out resets the sensor to asleep / +-2 g / +-250 deg/s, which
// would otherwise deliver wrong-scale or frozen data while reads still succeed.
const uint32_t kConfigCheckEverySamples = 100;
const uint32_t kErrorsBeforeReconfigure = 3;
const unsigned long kInitWaitMs = 500;

// Sampler task: static stack/TCB (no heap), pinned to core 0 so it never preempts the
// Arduino loop task (core 1) that runs the CAN poller. Priority above idle, below the
// Bluetooth/Wi-Fi stack tasks that also live on core 0.
const uint32_t kTaskStackBytes = 3072;
const UBaseType_t kTaskPriority = 5;
const BaseType_t kTaskCore = 0;
StackType_t s_taskStack[kTaskStackBytes];
StaticTask_t s_taskTcb;

} // namespace

bool ImuModule::writeRegister(uint8_t reg, uint8_t value) {
    const uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(kI2cPort, kAddress, buf, sizeof(buf), kI2cTimeoutTicks) == ESP_OK;
}

bool ImuModule::readRegisters(uint8_t reg, uint8_t* buf, size_t len) {
    return i2c_master_write_read_device(kI2cPort, kAddress, &reg, 1, buf, len, kI2cTimeoutTicks) == ESP_OK;
}

bool ImuModule::readSample(ImuSample& sample) {
    uint8_t raw[14];
    if (!readRegisters(kRegAccelXoutH, raw, sizeof(raw))) {
        return false;
    }
    for (int axis = 0; axis < 3; axis++) {
        sample.accel[axis] = (int16_t)((raw[2 * axis] << 8) | raw[2 * axis + 1]);
        sample.gyro[axis] = (int16_t)((raw[8 + 2 * axis] << 8) | raw[8 + 2 * axis + 1]);
    }
    return true;
}

bool ImuModule::configureSensor() {
    bool ok = writeRegister(kRegPwrMgmt1, kPwrClockPllGyroX);
    vTaskDelay(pdMS_TO_TICKS(10)); // PLL settle after wake-up
    ok = ok && writeRegister(kRegSmplrtDiv, kSmplrtDiv1kHz);
    ok = ok && writeRegister(kRegConfig, kConfigDlpf44Hz);
    ok = ok && writeRegister(kRegGyroConfig, kGyroFs500Dps);
    ok = ok && writeRegister(kRegAccelConfig, kAccelFs8g);
    return ok;
}

bool ImuModule::configurationIntact() {
    uint8_t pwr = 0, gyro = 0, accel = 0;
    if (!readRegisters(kRegPwrMgmt1, &pwr, 1) || !readRegisters(kRegGyroConfig, &gyro, 1) ||
        !readRegisters(kRegAccelConfig, &accel, 1)) {
        return false;
    }
    return pwr == kPwrClockPllGyroX && (gyro & kFullScaleMask) == kGyroFs500Dps &&
           (accel & kFullScaleMask) == kAccelFs8g;
}

// Sampler task, core 0: the I2C driver (and its interrupt) is installed from here.
bool ImuModule::initSensor() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = kSdaPin;
    conf.scl_io_num = kSclPin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = kI2cClockHz;
    if (i2c_param_config(kI2cPort, &conf) != ESP_OK ||
        i2c_driver_install(kI2cPort, conf.mode, 0, 0, 0) != ESP_OK) {
        return false;
    }
    if (!readRegisters(kRegWhoAmI, &_whoAmI, 1) || !isKnownWhoAmI(_whoAmI)) {
        i2c_driver_delete(kI2cPort);
        return false;
    }
    bool ok = writeRegister(kRegPwrMgmt1, kPwrReset);
    vTaskDelay(pdMS_TO_TICKS(100)); // device reset
    ok = ok && configureSensor();
    if (!ok) {
        i2c_driver_delete(kI2cPort);
    }
    return ok;
}

bool ImuModule::begin() {
    _task = xTaskCreateStaticPinnedToCore(taskEntry, "imu", kTaskStackBytes, this, kTaskPriority,
                                          s_taskStack, &s_taskTcb, kTaskCore);
    if (_task == nullptr) {
        Serial.println("[IMU] Sampler task creation failed -- IMU disabled.");
        return false;
    }

    // Wait for the task's sensor probe (~120 ms with a sensor, a few ms without). This runs
    // in setup(): the CAN module's begin() has already started the driver and sent the
    // session request, so this only delays the first poll pass once, at boot.
    unsigned long start = millis();
    while (_initState.load() == INIT_PENDING && millis() - start < kInitWaitMs) {
        delay(5);
    }
    if (_initState.load() != INIT_OK) {
        Serial.printf("[IMU] No MPU-6050 compatible sensor at 0x%02X (WHO_AM_I=0x%02X) -- IMU disabled.\n",
            kAddress, _whoAmI);
        return false; // the task deletes itself on failure
    }

    // esp_timer_create allocates its handle once here (framework API); never freed.
    esp_timer_create_args_t timerArgs = {};
    timerArgs.callback = &ImuModule::timerCallback;
    timerArgs.arg = this;
    timerArgs.dispatch_method = ESP_TIMER_TASK;
    timerArgs.name = "imu_tick";
    if (esp_timer_create(&timerArgs, &_timer) != ESP_OK) {
        Serial.println("[IMU] Timer creation failed -- IMU disabled.");
        return false;
    }
    _timerStartUs = esp_timer_get_time();
    if (esp_timer_start_periodic(_timer, kSamplePeriodUs) != ESP_OK) {
        Serial.println("[IMU] Timer start failed -- IMU disabled.");
        return false;
    }

    Serial.printf("[IMU] Sampling at 100 Hz (WHO_AM_I=0x%02X, +-8 g, +-500 deg/s).\n", _whoAmI);
    _initialized = true;
    return true;
}

// esp_timer task context: wake the sampler, nothing else.
void ImuModule::timerCallback(void* arg) {
    xTaskNotifyGive(static_cast<ImuModule*>(arg)->_task);
}

void ImuModule::taskEntry(void* arg) {
    static_cast<ImuModule*>(arg)->taskLoop();
}

void ImuModule::taskLoop() {
    if (!initSensor()) {
        _initState.store(INIT_FAILED);
        vTaskDelete(nullptr);
        return;
    }
    _initState.store(INIT_OK);

    uint32_t index = 0;
    uint32_t sinceCheck = 0;
    uint32_t consecutiveErrors = 0;
    for (;;) {
        // The notification value counts the ticks since the last wake-up. The index
        // advances by that count, so a tick this task was too late for is a sample index
        // gap the receiver counts, never a duplicate or a late read.
        uint32_t ticks = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        index += ticks;
        sinceCheck += ticks;

        if (sinceCheck >= kConfigCheckEverySamples || consecutiveErrors >= kErrorsBeforeReconfigure) {
            bool needed = consecutiveErrors >= kErrorsBeforeReconfigure || !configurationIntact();
            sinceCheck = 0;
            if (needed) {
                // Ticks during the ~10 ms re-configuration become an index gap.
                if (configureSensor()) {
                    consecutiveErrors = 0;
                }
                _reconfigurations.fetch_add(1, std::memory_order_relaxed);
                _ring.markEvent(IMU_EVENT_SENSOR_RECONFIGURED);
                continue;
            }
        }

        ImuSample sample;
        sample.index = index;
        // esp_timer periodic alarms do not drift and millis() uses the same clock, so the
        // tick's nominal time is the sample time on the telemetry deviceTimeMs clock.
        sample.timeMs = (uint32_t)((_timerStartUs + (int64_t)index * kSamplePeriodUs) / 1000);

        if (!readSample(sample)) {
            consecutiveErrors++;
            _readErrors.fetch_add(1, std::memory_order_relaxed);
            _lastReadOk.store(false, std::memory_order_relaxed);
            _ring.markReadError();
            continue;
        }
        consecutiveErrors = 0;
        _readsOk.fetch_add(1, std::memory_order_relaxed);
        _lastReadOk.store(true, std::memory_order_relaxed);
        if (_ring.push(sample)) {
            _samplesTaken.fetch_add(1, std::memory_order_relaxed);
        } else {
            _samplesDropped.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void ImuModule::update(SystemState& state) {
    unsigned long now = millis();
    uint32_t readsOk = _readsOk.load(std::memory_order_relaxed);
    if (readsOk != _lastReadsOk) {
        _lastReadsOk = readsOk;
        _lastProgressMs = now;
    }
    // Active only while reads keep succeeding: a stopped timer or task clears it too.
    state.imu.active = _lastReadOk.load(std::memory_order_relaxed) && _lastProgressMs != 0 &&
                       (now - _lastProgressMs) <= 100;
    state.imu.samplesTaken = _samplesTaken.load(std::memory_order_relaxed);
    state.imu.samplesDropped = _samplesDropped.load(std::memory_order_relaxed);
    state.imu.readErrors = _readErrors.load(std::memory_order_relaxed);
}
