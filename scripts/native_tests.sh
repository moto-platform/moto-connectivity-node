#!/usr/bin/env bash
# Runs the native (host) test suites without PlatformIO: same sources and flags as
# [env:native] in platformio.ini, compiled with g++ against Unity. CI uses
# `pio test -e native`; this script is for machines without PlatformIO registry access.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
UNITY_TAG="v2.6.0"
CACHE="$ROOT/.cache"
UNITY="$CACHE/unity-$UNITY_TAG"
GEN="$ROOT/external/moto-vehicle-defs/gen/c/conn"

if [ ! -f "$GEN/vehicle_cl250.h" ]; then
    echo "missing $GEN -- run: git submodule update --init" >&2
    exit 1
fi
if [ ! -d "$UNITY/src" ]; then
    mkdir -p "$CACHE"
    git -c advice.detachedHead=false clone --quiet --depth 1 --branch "$UNITY_TAG" \
        https://github.com/ThrowTheSwitch/Unity.git "$UNITY"
fi

OUT="$CACHE/native"
mkdir -p "$OUT"
status=0
for suite in test_native test_can_protocol; do
    exe="$OUT/$suite"
    g++ -std=c++11 -Wall -Wextra \
        -I "$ROOT/test/native_stubs" -I "$GEN" -I "$UNITY/src" \
        "$ROOT/test/$suite/test_main.cpp" "$UNITY/src/unity.c" -o "$exe"
    echo "== $suite"
    # Module logs start with "[" (e.g. "[UDS WARNING]"); show only Unity's lines.
    if "$exe" > "$OUT/$suite.log"; then :; else status=1; fi
    grep -v '^\[' "$OUT/$suite.log" || true
done
exit $status
