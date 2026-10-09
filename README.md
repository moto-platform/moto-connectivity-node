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
- **TWAI listen-only on the ESP32-S3** (capture env and the tester's 2 s listen window): the controller still sends a dominant error flag on a bus error in listen-only mode unless `CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM=y` (set in `sdkconfig.defaults`; CI checks it and the built `sdkconfig.h` of the tester, capture, GPS and probe envs). The workaround holds REC at 128 (error passive, recessive bits only) while listen-only runs, so BLE health reports `canRxErrorCount` 128 and ERROR_WARNING during the tester's listen window at boot. The switch to normal mode reinstalls the driver, which starts error active with TEC = REC = 0 (IDF 4.4.7 `twai_hal_start`).
- **Capture loop**: each pass waits up to 1 ms for the next frame (blocked, so core 1's idle task runs and the task watchdog does not reset the board), with a forced one-tick delay at least once a second when the queue never empties.
- **Bench checks still to do (hardware)**: HIL sends a frame with a corrupted CRC while a scope on the bus shows no dominant error flag from conn (capture env and tester listen window); 10 min of listen-only capture without a task-watchdog reset and with the `FINAL` summary at 300 s.

### Discovery probe (D-059, env `esp32-s3-devkitc-1-probe`)

The tester build plus `CONN_DISCOVERY_PROBE=1`; no other env compiles any of this.
- **When:** after the listen window and the session request, the generated `discovery_scan` list (24 requests, defs `uds/vehicle_cl250.yaml`) runs once: as soon as 0x50 confirms the extended session, or one `SESSION_RETRY_INTERVAL_MS` after the first session request without it (the banner says which; NRCs and timeouts are then the findings). A late 0x50 does not restart the scan. DID polling starts when the list is done, then runs as usual.
- **Requests:** one in flight, as Single Frames through the D-020 gate on both request IDs, with the yaml timeouts (base, 0x78 doubling up to `RESPONSE_TIMEOUT_MAX_MS`). Tester present and the session retry go out only between requests during the scan. An entry whose `after` gate did not answer with a bitmap marking its id (NRC, no answer, abort) is skipped, and so is its chain. After an unanswered scan request the next one waits `RESPONSE_TIMEOUT_MAX_MS` (a late answer is not taken for the next entry).
- **Segmented answers:** a First Frame that carries the in-flight entry's positive SID and echo with 8 ≤ FF_DL ≤ `MAX_FF_DL` (255) gets the generated FC.CTS (`vehicle_cl250_fc_cts`, byte for byte) once, on the request ID paired with the response ID it came on, through the same gate, latch and listen-window checks as every request. Nothing else is sent until the reception ends. It is aborted on N_Cr (`RESPONSE_TIMEOUT_BASE_MS`, checked after the RX drain against the drain time of the last CF), a sequence-number gap, a lost frame (driver lost count moved), a second FF or an SF on that ID, or after `RESPONSE_TIMEOUT_MAX_MS` in total. A First Frame that gets no FC, or an aborted reception, is followed by `RESPONSE_TIMEOUT_MAX_MS` without any request (yaml `flow_control`); a segmented DID answer is then not counted as a DID timeout.
- **RX queue:** 64 frames in this env (`src/hal/CanRxQueue.h`: ≥ `FC_MAX_CF_BURST` 36 + 16), drained up to 128 per pass, so the whole CF burst fits; 32 in the other builds.
- **Output:** USB serial lines starting with `[SCAN]` (hex in 32-byte rows, support bitmaps also as id lists) and `[ISOTP]` for each FC. **The VIN is printed as its WMI (first 3 characters) only**; its raw bytes are never printed, also not when its reception is aborted. The VIN entry is found by its SAE J1979 request (0x09 0x02); if that fails, every non-bitmap 0x09 answer prints only its length. Never commit the output (D-033); discovered PIDs/DIDs enter defs through `/signal-change` with `verified: false` (D-059 item 4).
- Not done: the yaml's optional retry of OBD 0x01/0x09 in the default session.

## Build and test

```bash
git submodule update --init
cp platformio_local.ini.example platformio_local.ini   # set your own AP password
pio run -e esp32-s3-devkitc-1          # real CAN/UDS
pio run -e esp32-s3-devkitc-1-mock     # synthetic telemetry, no ECU needed
pio run -e esp32-s3-devkitc-1-listen-only -t upload   # Q-001 capture probe (D-058): never transmits
pio run -e esp32-s3-devkitc-1-probe -t upload   # D-059 discovery scan once, then normal polling
pio device monitor -e esp32-s3-devkitc-1-probe | grep -E '^\[(SCAN|ISOTP)'   # never commit the output
pio device monitor -e esp32-s3-devkitc-1-listen-only | tee capture.csv   # 921600 baud; never commit captures
pio test -e native                     # host tests (or: scripts/native_tests.sh)
cppcheck --std=c++11 --enable=warning,performance,portability --error-exitcode=1 \
  --inline-suppr --suppress=missingIncludeSystem \
  -I src -I external/moto-vehicle-defs/gen/c/conn src   # blocking in CI (D-046)
```

Toolchain: PlatformIO, `framework = arduino, espidf` (Arduino-ESP32 2.0.x on ESP-IDF 4.4), board `esp32-s3-devkitc-1`.

## License

MIT, see `LICENSE` (D-036).
