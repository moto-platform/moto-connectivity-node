# CLAUDE.md — moto-connectivity-node

@.claude/PLATFORM-RULES.md

## What this repo is

Firmware running on **ESP32-S3**. Its role is narrow and clear: Wi-Fi/BLE connectivity (phone, server sync) and voice commands (ESP-SR, TinyML). This is where the currently working telemetry code **temporarily** lives — as the project progresses, the heavy work (telemetry/logging/UDS) will move to `moto-rt-core` (STM32H7), leaving only connectivity+voice here.

## What this repo is NOT (scope boundary — the most important rule)

- UDS/ISO-TP/bootloader does NOT live here (target: `moto-rt-core`)
- Cornering EKF estimation, context classification, virtual dyno do NOT live here (target: `moto-rt-core`)
- If a task would move outside this boundary, ask the user — "should we do this here or in rt-core."

## Voice command subsystem

- Uses Espressif's **official ESP-SR** framework (WakeNet + MultiNet). **Do not train your own model or build a TinyML pipeline from scratch** — MultiNet works with up to 300 words without retraining.
- Trigger: push-to-talk (handlebar button), NOT a wake word — a deliberate choice to avoid false triggers from wind noise. If a suggestion comes in to switch to a wake word because it "would be cooler," reject it; the rationale stays in a code comment.

## Bridge (to the H7)

Simple messaging with `moto-rt-core` (STM32H7) over SPI/UART (future target: SOME/IP). The message format is not yet finalized (Q-004; proposal: COBS + CRC16 + msg-id, defined in moto-vehicle-defs) — leave a placeholder/TODO here; the real format will be settled once both repos design it together.

Target state: connected to the platform CAN via TWAI (classic CAN, 500 kbps): receives the io-node's park-mode alarm and relays it over Wi-Fi, publishes its own heartbeat, not connected to the vehicle bus. Until rt-core takes over, the single TWAI is on the vehicle bus instead (temporary tester, see below), so no heartbeat is sent yet.

## Dependencies

Reads signal definitions from `moto-vehicle-defs` (submodule: `external/moto-vehicle-defs`, pinned to a tag; generated code at `external/moto-vehicle-defs/gen/c/<node>/`).

## Build

PlatformIO with **Arduino as an ESP-IDF component** (`framework = arduino, espidf`, D-023). Target chip: `esp32s3`.
- `git submodule update --init` (generated `external/moto-vehicle-defs/gen/c/conn/`, pinned to `v0.5.1`); the defs repo is public (D-033), so CI needs no secret
- `pio run -e esp32-s3-devkitc-1` (real) / `-e esp32-s3-devkitc-1-mock` / `-e esp32-s3-devkitc-1-no-tester` (poller off); needs `platformio_local.ini` with `[local] build_flags = -D AP_PASSWORD=...`
- `pio test -e native`, or `scripts/native_tests.sh` (g++ + Unity, no PlatformIO registry needed)
- Static analysis in CI (D-046 item 3, cppcheck 2.22.0 like moto-vehicle-defs): warning/portability/performance on `src/` is **blocking** (run the CI command locally before pushing); the MISRA C:2012 addon + style checks only **report** (job summary)

## Legacy telemetry port (D-023)

The code comes from the read-only reference repo `moto-platform/HondaCl250_Telemetry` (archived). Ported as-is: CAN/UDS module, `ICanBus`/`TwaiCanBus`, mock CAN, BLE server + packet schema, Nextion, serial logger, native tests. Rewritten: WiFi server (no Arduino `String`). Dropped: web PWA and the complementary-filter lean angle.
- This node is the **temporary sole tester** on the vehicle bus until rt-core takes over. Then its poller must be disabled (`CONN_VEHICLE_TESTER=0`, env `esp32-s3-devkitc-1-no-tester`: no TWAI driver at all); two testers are never allowed (D-021). At runtime the poller also latches off when it sees another tester or repeated bus-off; the latch and the bus-off count are kept in RTC no-init memory across every reset except power-on, and an invalid record after such a reset counts as latched (Q-018). After boot the TWAI driver runs listen-only until it has received 2 s with no lost frames and no diagnostic request or unsolicited ECU answer, and only then sends the first request. Every loop pass drains RX (bounded) before any transmission or timeout check. Power-cycle the node to clear a latch. Only the D-020 service allow-list may be sent.
- Vehicle IDs, DIDs, formulas and UDS timing come from `external/moto-vehicle-defs/gen/c/conn/vehicle_cl250.h`; every frame passes `vehicle_cl250_frame_allowed()` (D-020). Never hand-write them again (`vss-schema-guardian` flags leftovers).
- The BLE layouts are defined in moto-vehicle-defs `ble/ble_schema.json` (D-061): telemetry v4 (v2 as low-MTU fallback; v3 is only decoded for old sessions) and the 100 Hz IMU block (D-032). The packed structs in `src/BLETelemetryPacket.h` and the IMU packer static_assert against the generated `gen/c/conn/ble_schema.h`; moto-mobile and moto-server use the generated Dart/Python. A layout change is a defs change, and conn, moto-mobile and moto-server take it together.
- v4 carries the tester's step gap and one rotating per-DID ECU round-trip record (D-058, `TesterStatsTracker`, fed by `HondaCANModule`): counters only, they never change request timing, order, the gate or the latch. TEMPORARY until rt-core's health DID 0xFD02.
- `esp32-s3-devkitc-1-listen-only` (`CONN_CAN_LISTEN_ONLY=1`, D-058 item 4) is the Q-001 capture probe: TWAI listen-only behind the receive-only `ICanRx`, `TwaiCanBus` not compiled, only `CanCaptureModule` runs and writes to serial (921600 baud). It can never be combined with the tester or the mock (`#error`), and CI checks its ELF has no `twai_transmit` (`scripts/check_no_twai_tx.sh`). Listen-only on the ESP32-S3 sends dominant error flags unless `CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM=y` (`sdkconfig.defaults`; REC held at 128, error passive); the same script checks it in the capture and tester builds (the tester's listen window uses the same mode). Never remove it.

## Context

Hardware architecture: `../moto-vehicle-defs/docs/ARCHITECTURE.md` (summary) · detail: `../moto-vehicle-defs/docs/hardware-architecture.md` (section index in `docs/README.md`) section 2-3 (general), section 5b.6 (voice command detail).
