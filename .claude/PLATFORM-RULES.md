<!-- Synced from moto-workspace/PLATFORM-RULES.md by scripts/sync_claude.py. Do not edit here; edit the source and re-run the sync. -->

# moto-platform — platform rules (shared by every repo)

<!-- Source of truth: moto-workspace/PLATFORM-RULES.md. Copies in each repo's .claude/ are written by scripts/sync_claude.py — edit only the source. -->

SDV-architecture diagnostics / telemetry / ADAS platform for motorcycles (first vehicle: Honda CL250). Senior thesis → GitHub org `moto-platform` (12 repos, private). The user writes in Turkish. Reply in Turkish, but write every project artifact (code, comments, docs, commits) in English.

## Where things are

- Platform docs live in the `moto-vehicle-defs` repo under `docs/`. Depending on where you run, the path is `moto-vehicle-defs/docs/` (workspace root), `docs/` (inside moto-vehicle-defs) or `external/moto-vehicle-defs/docs/` (consumer repo with the submodule). If none of these exist (e.g. a single-repo cloud session), say so and ask. Never guess architecture facts.
- Read first: `ARCHITECTURE.md` (summary) and `DECISIONS.md` (D-xxx decisions, Q-xxx open questions). `docs/README.md` indexes the large docs by line range. Read only the needed range, never a whole raw doc.
- Do not re-litigate recorded decisions. New decisions → ask the user, then record them in `DECISIONS.md`.

## Repo map (dependencies are one-way: everyone → moto-vehicle-defs)

| Repo | Runtime | Tooling | Role |
|---|---|---|---|
| `moto-vehicle-defs` | — | DBC, VSS, UDS YAML, Python codegen | Single source of truth for signals + platform docs |
| `moto-rt-core` | STM32H7 | C, CMake+CubeMX, FreeRTOS | Domain controller: CAN gateway, logging, fusion, UDS/ISO-TP, bootloader, XCP, EKF, context, dyno, anomaly safety net |
| `moto-safety-node` | STM32G4 | C, CMake+CubeMX, bare-metal | ONLY the cornering safety decision, isolated |
| `moto-io-node` | STM32G0 (proto F103) | C, CMake+CubeMX, bare-metal | Blind spot + immobilizer + power management |
| `moto-connectivity-node` | ESP32-S3 | C, ESP-IDF | Wi-Fi/BLE + voice (ESP-SR); telemetry here is TEMPORARY |
| `moto-linux-node` | Raspberry Pi 5 | Python/C++ | Kuksa, CAN→VSS, lane keeping (classic CV), anomaly model, HMI, OTA |
| `moto-mcp` | Raspberry Pi 5 | Python | Standalone open-source MCP server (read-only tools) |
| `moto-hil-bench` | STM32F4 + i7 host | C + Python | Restbus HIL bench, scenario engine, CI runner |
| `moto-server` | Server | Python (FastAPI) | Ingestion, storage, MDF4/Parquet, OTA packages |
| `moto-ml` | Offline | Python | Training only |
| `moto-mobile` | Phone | Flutter (D-022) | Loosely coupled companion |
| `moto-workspace` | — | — | Manifest, setup script, shared Claude agents/skills (source of truth) |
| `HondaCl250_Telemetry` | — | archived | Read-only legacy reference (verified CL250 CAN/UDS code). Never modify it; extract facts per D-023 |

## Platform invariants (non-negotiable; if one must be broken, STOP and ask)

1. **Never write to the ECU.** No repo writes to the engine ECU, flashes it, or changes maps. rt-core is the **only** tester on the vehicle bus (D-021); until rt-core exists, connectivity-node is the temporary sole tester (D-023), and there are never two. It may send only: `0x10` with sub-function 0x01/0x03, `0x3E`, `0x22`, `0x19`, and OBD `0x01/0x09`. NEVER `0x10 0x02`, `0x11`, `0x14`, `0x27`, `0x2E`, `0x2F`, `0x31`, `0x34`, `0x36`, `0x37` (D-020). All other nodes stay off the vehicle bus or strictly listen-only. rt-core republishes decoded vehicle signals on the separate platform CAN bus, where our nodes talk (D-009, D-021).
2. **Never invent signals.** CAN IDs, signal names, scales, VSS paths and DIDs come only from `moto-vehicle-defs` (generated `gen/` code or DBC/VSS). Hand-written literals → `vss-schema-guardian`.
3. **The safety path is isolated.** The cornering decision (safety-node) and the blind-spot decision (io-node) work without the Raspi, the phone, Wi-Fi, VSS or ML. ML may only tighten thresholds, never loosen the safety ceiling.
4. **Critical work never depends on a slow channel.** µs-critical → MCU + CAN. ms → Raspi. Latency-tolerant → phone/server.
5. **No dynamic memory on MCUs** (strictly none on safety paths, avoided elsewhere). No string/DBC/VSS parsing on MCUs.
6. **The immobilizer acts on the starter-relay coil circuit only.** Default state unlocked, hidden bypass, 10 s timeout to unlock.
7. **The LLM never does its own math, and raw GPS never goes to the cloud.**
8. After any change to a safety-critical module (cornering/, safety-node, immobilizer, blind spot, watchdog/heartbeat, anomaly-safety-net, E2E, bootloader), run `safety-reviewer` before committing.

## Conventions

- Conventional Commits (`feat(uds): ...`). Commit or push only when the user asks.
- `moto-vehicle-defs` uses semver. Consumers pin the `external/moto-vehicle-defs` submodule (relative URL) to a **tag**. The known-good combination is recorded in `moto-workspace/manifest.yaml`.
- Skills: `/repo-bootstrap` (repo skeleton), `/signal-change` (signals), `/feature-module` (firmware feature), `/handoff` (end of session, workspace only).
- Token budget is limited: delegate bulk mechanical work to `sonnet`/`haiku` subagents, use `Explore` for broad searches, one task per session.
