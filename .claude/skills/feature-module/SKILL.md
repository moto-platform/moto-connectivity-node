---
name: feature-module
description: Add or substantially change a firmware feature module (src/features/<name>/) in moto-rt-core, moto-safety-node, moto-io-node or the HIL simulator, following the platform's layering, memory, test and safety rules. Use for requests like "implement ISO-TP", "add the cornering EKF", "add the blind-spot filter".
argument-hint: <repo> <feature-name>
---

<!-- Synced from moto-workspace/.claude/skills/feature-module/SKILL.md by scripts/sync_claude.py. Do not edit here; edit the source and re-run the sync. -->

# Firmware feature module

## 1. Plan (short, before code)

- Confirm the feature belongs in this repo: check its `CLAUDE.md`. If unsure, run `architecture-guard` on the plan.
- Write down inputs (which generated signals/HAL services), outputs (signals published, actuators), period/deadline, task priority, and failure behaviour (what the module does on missing/invalid input).
- Signals it needs that do not exist yet → `/signal-change` first.
- For standards-based modules (ISO-TP ISO 15765-2, UDS ISO 14229, XCP), list the services/parameters in scope and cite the clause. Keep them minimal, matching the thesis scope (Ç2/Ç3 in `tr/bitirme-projesi-kapsam.md`).

## 2. Structure

```
src/features/<name>/
  <name>.h          # public API: init(), step()/on_frame(), status getter — the only header others may use
  <name>.c          # glue: reads services, calls core, publishes outputs
  <name>_core.c/.h  # pure logic, no HAL/RTOS includes → host-testable
  README.md         # responsibility, I/O, timing, failure behaviour, requirement IDs
tests/host/test_<name>.c
```
- No includes from other `features/`. Talk through `services/` (signal pool, com, diag, log).
- Static allocation only: fixed-size buffers sized by named constants, no heap, no recursion, and bounded loops.
- Time comes from the service timebase, never busy-waits. ISR work is minimal (queue, then return).
- Generated code (`gen/c/<node>/`) provides pack/unpack and E2E. Do not reimplement.

## 3. Tests (L0 required)

- Unity tests for the `_core` logic: nominal cases, boundary values, invalid/missing input, E2E faults (bad CRC, frozen counter, timeout) where applicable, and state-machine transitions.
- Name tests after requirement IDs where they exist (`test_REQ_UDS_012_...`) for the traceability matrix (Ç8).
- Propose one HIL scenario (name + expected measurable result) for `moto-hil-bench/scenarios/`. Write it only if the host schema exists.

## 4. Integrate

- Register the module in `src/app/` with its period and priority. Document the priority rationale if it is safety-relevant (freedom from interference).
- Build all presets and run host tests. Report flash/RAM delta from the linker map (`arm-none-eabi-size`).

## 5. Review gates

- Safety-relevant (cornering, safety-node logic, blind spot, immobilizer, watchdog/heartbeat, anomaly-safety-net, E2E, bootloader) → `safety-reviewer` before proposing a commit.
- CAN/VSS literals appeared → `vss-schema-guardian`.
- Update the repo `CLAUDE.md` module list if a new module was added.
