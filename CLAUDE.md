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

Connected to the platform CAN via TWAI (classic CAN, 500 kbps): receives the io-node's park-mode alarm and relays it over Wi-Fi, publishes its own heartbeat. Not connected to the vehicle bus.

## Dependencies

Reads signal definitions from `moto-vehicle-defs` (submodule: `external/moto-vehicle-defs`, pinned to a tag; generated code at `external/moto-vehicle-defs/gen/c/<node>/`).

## Build

PlatformIO with **Arduino as an ESP-IDF component** (`framework = arduino, espidf`, D-023). Target chip: `esp32s3`. Native unit tests with `pio test -e native`.

## Legacy telemetry port (D-023)

The code comes from the read-only reference repo `moto-platform/HondaCl250_Telemetry` (archived). Ported as-is: CAN/UDS module, `ICanBus`/`TwaiCanBus`, mock CAN, BLE server + packet schema, Nextion, serial logger, native tests. Rewritten: WiFi server (no Arduino `String`). Dropped: web PWA and the complementary-filter lean angle.
- This node is the **temporary sole tester** on the vehicle bus until rt-core takes over. Then its poller must be disabled; two testers are never allowed (D-021). Only the D-020 service allow-list may be sent.
- Hand-written DIDs/IDs are temporary. Replace them with `external/moto-vehicle-defs/gen/c/conn/` once codegen exists (`vss-schema-guardian` flags leftovers).

## Context

Hardware architecture: `../moto-vehicle-defs/docs/ARCHITECTURE.md` (summary) · detail: `../moto-vehicle-defs/docs/hardware-architecture.md` (section index in `docs/README.md`) section 2-3 (general), section 5b.6 (voice command detail).
