# moto-connectivity-node

ESP32-S3 firmware of the [moto-platform](https://github.com/moto-platform) motorcycle platform: Wi-Fi/BLE connectivity and (later) ESP-SR voice commands. Until `moto-rt-core` exists it also carries the CL250 telemetry that was verified on the bike, ported from the archived `HondaCl250_Telemetry@legacy-final` (decision D-023):

- `HondaCANModule`: UDS tester for the CL250 engine ECU. This node is the **temporary sole vehicle-bus tester** (D-021/D-023); only the D-020 service allow-list can be sent, enforced by the generated `vehicle_cl250_frame_allowed()` guard.
- `BLEServerModule`: telemetry notifications to moto-mobile, layout in moto-vehicle-defs `ble/ble_schema.json` (D-061, generated `gen/c/conn/ble_schema.h`): version 4 with node clock, signal ages, CAN health and the tester statistics of D-058 (step gap, one rotating per-DID ECU round-trip record); version 2 when the negotiated MTU is too small, plus 100 Hz IMU sample blocks on a second characteristic (D-032) and, in the GPS build, one speed/heading block per UBX NAV-PVT on a third (D-060; no position; subscribing needs a bonded link).
- `ImuModule`: MPU-6050 compatible IMU on I2C (SDA GPIO1, SCL GPIO2), sampled at 100 Hz by an esp_timer-driven task on core 0 into a static ring buffer, so it never delays the CAN poller in the main loop. Raw data for analysis only, no lean estimate.
- `WiFiServerModule`: on-demand access point with `/api/telemetry` JSON (static buffers, no Arduino `String`).
- `NextionModule`, `SerialLoggerModule`, `MockCANModule` (synthetic data for bench tests).

Signal definitions come from the `external/moto-vehicle-defs` submodule (generated `gen/c/conn/`); nothing vehicle-specific is hand-written here.

### Poller timing against D-053 (temporary tester scope)

- The poller takes the generated periods (speed and RPM 110 ms since D-053) and keeps one request in flight. It picks the first due DID in table order; the `priority` field and the D-050..D-052 fault rules are implemented in rt-core only, which replaces this tester (D-021).
- Order (D-053 item 2): each `update()` drains RX before its response-timeout check (Q-018), so an answer already queued resolves its request instead of counting as a timeout.
- Step (D-053 `client_step_max_ms` = 10 ms): the step is one `loop()` pass, which also runs the BLE, Wi-Fi, Nextion and logger modules, and its worst case is not bounded or counted at runtime. The G0.1 loop timing report (max per 10 s) is the measurement; a pass longer than 10 ms can let an answer just inside the base timeout be seen after its DID is due again. Accepted for the temporary tester until rt-core polls; a measured worst case above 10 ms needs a conn change or a larger defs value.

### Measurement tools (D-058)

- **Round trip per DID** (BLE v4, `TesterStatsTracker`): from the request's send to the step that drains its answer, so one step is included. A request answered after NRC 0x78 counts in `nrc78` and gives no sample. A request sent within `RESPONSE_TIMEOUT_MAX_MS` (2000 ms, gen/) after any timeout gives no sample either: the timed-out request's answer may still arrive and resolve it early (a positive answer for the next read of the same DID, an NRC, which carries no DID, for any read), which would make the minimum too low; a 0x78 in that window is not counted in `nrc78` either. An ECU answering later than that is outside its own P2* limit and not covered. The counters never change request timing, order, the gate or the latch (native test compares every sent frame and its pass).
- **Step gap**: every `update()` entry of the poller, also in the listen window and when latched.
- **TWAI listen-only on the ESP32-S3** (capture env and the tester's 2 s listen window): the controller still sends a dominant error flag on a bus error in listen-only mode unless `CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM=y` (set in `sdkconfig.defaults`; CI checks it and the built `sdkconfig.h` of the tester and capture envs). The workaround holds REC at 128 (error passive, recessive bits only) while listen-only runs, so BLE health reports `canRxErrorCount` 128 and ERROR_WARNING during the tester's listen window at boot. The switch to normal mode reinstalls the driver, which starts error active with TEC = REC = 0 (IDF 4.4.7 `twai_hal_start`).
- **Capture loop**: each pass waits up to 1 ms for the next frame (blocked, so core 1's idle task runs and the task watchdog does not reset the board), with a forced one-tick delay at least once a second when the queue never empties.
- **Bench checks still to do (hardware)**: HIL sends a frame with a corrupted CRC while a scope on the bus shows no dominant error flag from conn (capture env and tester listen window); 10 min of listen-only capture without a task-watchdog reset and with the `FINAL` summary at 300 s.

## Build and test

```bash
git submodule update --init
cp platformio_local.ini.example platformio_local.ini   # set your own AP password
pio run -e esp32-s3-devkitc-1          # real CAN/UDS
pio run -e esp32-s3-devkitc-1-mock     # synthetic telemetry, no ECU needed
pio run -e esp32-s3-devkitc-1-listen-only -t upload   # Q-001 capture probe (D-058): never transmits
pio device monitor -e esp32-s3-devkitc-1-listen-only | tee capture.csv   # 921600 baud; never commit captures
pio test -e native                     # host tests (or: scripts/native_tests.sh)
cppcheck --std=c++11 --enable=warning,performance,portability --error-exitcode=1 \
  --inline-suppr --suppress=missingIncludeSystem \
  -I src -I external/moto-vehicle-defs/gen/c/conn src   # blocking in CI (D-046)
```

Toolchain: PlatformIO, `framework = arduino, espidf` (Arduino-ESP32 2.0.x on ESP-IDF 4.4), board `esp32-s3-devkitc-1`.

## License

MIT, see `LICENSE` (D-036).
