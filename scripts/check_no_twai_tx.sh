#!/usr/bin/env bash
# D-058 item 4: the listen-only capture firmware must contain no TWAI transmit code, so it
# can never become a second tester on the vehicle bus (D-037). Checks the built ELFs:
#   - esp32-s3-devkitc-1-listen-only: no twai_transmit symbol
#   - esp32-s3-devkitc-1 (the tester): has it, so the check is not vacuous
# A missing TX API alone does not keep the bus untouched: in listen-only mode the ESP32-S3
# still sends a dominant error flag on a bus error unless the IDF errata workaround is
# compiled in. Every build that uses listen-only (the capture, and the tester's Q-018
# window: the tester env and the GPS bring-up env esp32-s3-devkitc-1-gps, D-060) must
# enable CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM in its generated sdkconfig.h.
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

for env in esp32-s3-devkitc-1-listen-only esp32-s3-devkitc-1 esp32-s3-devkitc-1-gps; do
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

if [ "$tester" -eq 0 ]; then
    echo "positive control failed: the tester ELF has no twai_transmit symbol" >&2
    exit 1
fi
if [ "$listen_only" -ne 0 ]; then
    echo "the listen-only ELF contains twai_transmit ($listen_only symbol lines)" >&2
    exit 1
fi
echo "listen-only ELF: no twai_transmit; tester ELF: $tester twai_transmit symbol line(s); listen-only errata workaround on in the capture, tester and GPS builds"
