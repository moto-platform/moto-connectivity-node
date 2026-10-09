#!/usr/bin/env bash
# D-058 item 4: the listen-only capture firmware must contain no TWAI transmit code, so it
# can never become a second tester on the vehicle bus (D-037). Checks the built ELFs:
#   - esp32-s3-devkitc-1-listen-only: no twai_transmit symbol
#   - esp32-s3-devkitc-1 (the tester): has it, so the check is not vacuous
# A missing TX API alone does not keep the bus untouched: in listen-only mode the ESP32-S3
# still sends a dominant error flag on a bus error unless the IDF errata workaround is
# compiled in. Every build that uses listen-only (the capture, and the tester's Q-018
# window: the tester env, the GPS bring-up env esp32-s3-devkitc-1-gps (D-060) and the D-059
# discovery probe esp32-s3-devkitc-1-probe) must enable CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM
# in its generated sdkconfig.h.
# D-059 item 3: only the probe compiles the discovery scan and the segmented reception (the
# one FC.CTS sender). Their strings must be in the probe ELF and absent from the tester ELF.
# (vehicle_cl250_fc_cts itself is in every tester ELF: the stateless D-020 frame gate
# compares against it, so the check uses the probe-only code's strings.)
# Run after `pio run` of these envs.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NM="${NM:-$(ls "$HOME"/.platformio/packages/toolchain-xtensa-esp32s3/bin/xtensa-esp32s3-elf-nm 2>/dev/null | head -n 1)}"
if [ -z "$NM" ] || [ ! -x "$NM" ]; then
    echo "xtensa-esp32s3-elf-nm not found (set NM=...)" >&2
    exit 1
fi

symbols() {
    local elf="$ROOT/.pio/build/$1/firmware.elf"
    if [ ! -f "$elf" ]; then
        echo "missing $elf -- run: pio run -e $1" >&2
        exit 1
    fi
    "$NM" "$elf"
}

listen_only=$(symbols esp32-s3-devkitc-1-listen-only | grep -c 'twai_transmit' || true)
tester=$(symbols esp32-s3-devkitc-1 | grep -c 'twai_transmit' || true)

for env in esp32-s3-devkitc-1-listen-only esp32-s3-devkitc-1 esp32-s3-devkitc-1-gps esp32-s3-devkitc-1-probe; do
    cfg="$ROOT/.pio/build/$env/config/sdkconfig.h"
    if [ ! -f "$cfg" ]; then
        echo "missing $cfg -- run: pio run -e $env" >&2
        exit 1
    fi
    if ! grep -qx '#define CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM 1' "$cfg"; then
        echo "$env: CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM is not enabled (listen-only would send dominant error flags)" >&2
        exit 1
    fi
done

# Probe-only strings (src/DiscoveryScan.h banner, HondaCANModule::sendFlowControl log).
probe_markers=("D-059 discovery scan" "D-059 FC.CTS sent")
for marker in "${probe_markers[@]}"; do
    for env in esp32-s3-devkitc-1-probe esp32-s3-devkitc-1; do
        elf="$ROOT/.pio/build/$env/firmware.elf"
        if [ ! -f "$elf" ]; then
            echo "missing $elf -- run: pio run -e $env" >&2
            exit 1
        fi
    done
    if ! grep -aq "$marker" "$ROOT/.pio/build/esp32-s3-devkitc-1-probe/firmware.elf"; then
        echo "positive control failed: the probe ELF has no '$marker'" >&2
        exit 1
    fi
    if grep -aq "$marker" "$ROOT/.pio/build/esp32-s3-devkitc-1/firmware.elf"; then
        echo "the tester ELF contains the probe-only '$marker' (segmented reception outside the probe env)" >&2
        exit 1
    fi
done

if [ "$tester" -eq 0 ]; then
    echo "positive control failed: the tester ELF has no twai_transmit symbol" >&2
    exit 1
fi
if [ "$listen_only" -ne 0 ]; then
    echo "the listen-only ELF contains twai_transmit ($listen_only symbol lines)" >&2
    exit 1
fi
echo "listen-only ELF: no twai_transmit; tester ELF: $tester twai_transmit symbol line(s), no discovery scan; listen-only errata workaround on in the capture, tester, GPS and probe builds"
