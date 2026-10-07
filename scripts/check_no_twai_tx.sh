#!/usr/bin/env bash
# D-058 item 4: the listen-only capture firmware must contain no TWAI transmit code, so it
# can never become a second tester on the vehicle bus (D-037). Checks the built ELFs:
#   - esp32-s3-devkitc-1-listen-only: no twai_transmit symbol
#   - esp32-s3-devkitc-1 (the tester): has it, so the check is not vacuous
# Run after `pio run` of both envs.
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

if [ "$tester" -eq 0 ]; then
    echo "positive control failed: the tester ELF has no twai_transmit symbol" >&2
    exit 1
fi
if [ "$listen_only" -ne 0 ]; then
    echo "the listen-only ELF contains twai_transmit ($listen_only symbol lines)" >&2
    exit 1
fi
echo "listen-only ELF: no twai_transmit; tester ELF: $tester twai_transmit symbol line(s)"
