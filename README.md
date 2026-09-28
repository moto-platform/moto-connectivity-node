# moto-connectivity-node

ESP32-S3 firmware of the [moto-platform](https://github.com/moto-platform) motorcycle platform: Wi-Fi/BLE connectivity and (later) ESP-SR voice commands. Until `moto-rt-core` exists it also carries the CL250 telemetry that was verified on the bike, ported from the archived `HondaCl250_Telemetry@legacy-final` (decision D-023):

- `HondaCANModule`: UDS tester for the CL250 engine ECU. This node is the **temporary sole vehicle-bus tester** (D-021/D-023); only the D-020 service allow-list can be sent, enforced by the generated `vehicle_cl250_frame_allowed()` guard.
- `BLEServerModule`: telemetry notifications to moto-mobile, layout in [`docs/ble_telemetry_packet_schema.json`](docs/ble_telemetry_packet_schema.json) (version 3 with node clock, signal ages and CAN health; version 2 when the negotiated MTU is too small), plus 100 Hz IMU sample blocks on a second characteristic (D-032).
- `ImuModule`: MPU-6050 compatible IMU on I2C (SDA GPIO1, SCL GPIO2), sampled at 100 Hz by an esp_timer-driven task on core 0 into a static ring buffer, so it never delays the CAN poller in the main loop. Raw data for analysis only, no lean estimate.
- `WiFiServerModule`: on-demand access point with `/api/telemetry` JSON (static buffers, no Arduino `String`).
- `NextionModule`, `SerialLoggerModule`, `MockCANModule` (synthetic data for bench tests).

Signal definitions come from the `external/moto-vehicle-defs` submodule (generated `gen/c/conn/`); nothing vehicle-specific is hand-written here.

## Build and test

```bash
git submodule update --init
cp platformio_local.ini.example platformio_local.ini   # set your own AP password
pio run -e esp32-s3-devkitc-1          # real CAN/UDS
pio run -e esp32-s3-devkitc-1-mock     # synthetic telemetry, no ECU needed
pio test -e native                     # host tests (or: scripts/native_tests.sh)
```

Toolchain: PlatformIO, `framework = arduino, espidf` (Arduino-ESP32 2.0.x on ESP-IDF 4.4), board `esp32-s3-devkitc-1`.

## License

MIT, see `LICENSE` (D-036).
