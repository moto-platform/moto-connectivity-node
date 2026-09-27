#!/usr/bin/env python3
"""Checks the firmware BLE layouts against docs/ble_telemetry_packet_schema.json.

Compiles a small host probe (g++) that prints sizeof/offsetof of the telemetry structs
and packs one IMU block, then compares every field with the schema: offsets, sizes,
totals, versions, constants and the IMU block bytes. Run in CI next to the native tests;
exits non-zero on any mismatch so the schema and the firmware cannot drift apart.
"""

import json
import os
import struct
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCHEMA = os.path.join(ROOT, "docs", "ble_telemetry_packet_schema.json")
GEN = os.path.join(ROOT, "external", "moto-vehicle-defs", "gen", "c", "conn")

TELEMETRY_V3 = [
    "version", "seq", "deviceTimeMs", "rpm", "speed", "coolantTemp", "throttlePos",
    "batteryVolt", "leanAngle", "maxLeanRight", "maxLeanLeft", "flags", "rpmAgeMs",
    "speedAgeMs", "coolantTempAgeMs", "throttlePosAgeMs", "batteryVoltAgeMs", "canBusState",
    "canTxErrorCount", "canRxErrorCount", "canBusOffCount", "unansweredDidCount", "canFlags",
]
TELEMETRY_V2 = [
    "version", "seq", "rpm", "speed", "coolantTemp", "throttlePos", "batteryVolt",
    "leanAngle", "maxLeanRight", "maxLeanLeft", "flags",
]

PROBE = r"""
#include <cstdio>
#include <cstddef>
#include "BLETelemetryPacket.h"
#include "ImuBlockPacket.h"
#define F(S, M) std::printf("\"%s.%s\": [%u, %u],\n", #S, #M, (unsigned)offsetof(S, M), (unsigned)sizeof(((S*)0)->M));
int main() {
    std::printf("{\n");
    %FIELDS%
    std::printf("\"v3.size\": %u, \"v2.size\": %u,\n", (unsigned)sizeof(BLETelemetryPacketV3), (unsigned)sizeof(BLETelemetryPacketV2));
    std::printf("\"v3.version\": %u, \"v2.version\": %u,\n", BLE_PACKET_VERSION, BLE_PACKET_VERSION_LEGACY);
    std::printf("\"lean.na\": %d, \"age.never\": %u, \"age.max\": %u,\n", BLE_LEAN_NOT_AVAILABLE, BLE_AGE_NEVER_RECEIVED, BLE_AGE_MAX_MS);
    std::printf("\"imu.version\": %u, \"imu.header\": %u, \"imu.sample\": %u, \"imu.max\": %u, \"imu.period\": %u, \"imu.maxBytes\": %u,\n",
        IMU_BLOCK_VERSION, IMU_BLOCK_HEADER_BYTES, IMU_BLOCK_SAMPLE_BYTES, IMU_BLOCK_MAX_SAMPLES, IMU_SAMPLE_PERIOD_MS, (unsigned)IMU_BLOCK_MAX_BYTES);
    std::printf("\"imu.perBlock\": [%u, %u, %u],\n", imuSamplesPerBlock(23), imuSamplesPerBlock(40), imuSamplesPerBlock(185));
    ImuRingBuffer<4> ring;
    ImuSample s; s.index = 0x10002; s.timeMs = 0xA0B0C0D0u;
    s.accel[0] = 1; s.accel[1] = -2; s.accel[2] = 4096; s.gyro[0] = -32768; s.gyro[1] = 655; s.gyro[2] = 32767;
    ring.push(s);
    uint8_t out[IMU_BLOCK_MAX_BYTES];
    size_t len = packImuBlock(ring, 10, 0x33, IMU_EVENT_READ_ERROR, out, sizeof(out));
    std::printf("\"imu.block\": \"");
    for (size_t i = 0; i < len; i++) std::printf("%02x", out[i]);
    std::printf("\"\n}\n");
    return 0;
}
"""

FMT = {"uint8": "B", "int8": "b", "uint16": "H", "int16": "h", "uint32": "I"}


def fail(errors, msg):
    errors.append(msg)


def check_fields(errors, label, fields, struct_name, probe, names):
    by_name = {f["name"]: f for f in fields}
    if [f["name"] for f in fields] != names:
        fail(errors, f"{label}: schema field order {[f['name'] for f in fields]} != firmware {names}")
    expected = 0
    for f in fields:
        if f["offset"] != expected:
            fail(errors, f"{label}.{f['name']}: offset {f['offset']} leaves a gap/overlap (expected {expected})")
        if struct.calcsize("<" + FMT[f["type"]]) != f["size"]:
            fail(errors, f"{label}.{f['name']}: type {f['type']} does not have size {f['size']}")
        expected = f["offset"] + f["size"]
    for name in names:
        key = f"{struct_name}.{name}"
        if name not in by_name or key not in probe:
            continue
        off, size = probe[key]
        if [by_name[name]["offset"], by_name[name]["size"]] != [off, size]:
            fail(errors, f"{label}.{name}: schema {by_name[name]['offset']}/{by_name[name]['size']} != firmware {off}/{size}")
    return expected


def main():
    schema = json.load(open(SCHEMA))
    fields_src = "\n    ".join(
        [f"F(BLETelemetryPacketV3, {n})" for n in TELEMETRY_V3] + [f"F(BLETelemetryPacketV2, {n})" for n in TELEMETRY_V2]
    )
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "probe.cpp")
        exe = os.path.join(tmp, "probe")
        open(src, "w").write(PROBE.replace("%FIELDS%", fields_src))
        subprocess.check_call(
            ["g++", "-std=c++11", "-D", "CONN_NATIVE_TEST", "-w",
             "-I", os.path.join(ROOT, "test", "native_stubs"), "-I", GEN, "-I", os.path.join(ROOT, "src"),
             src, os.path.join(GEN, "vehicle_cl250.c"), "-o", exe]
        )
        probe = json.loads(subprocess.check_output([exe]).decode())

    errors = []
    v3_total = check_fields(errors, "v3", schema["fields"], "BLETelemetryPacketV3", probe, TELEMETRY_V3)
    fb = schema["lowMtuFallback"]
    v2_total = check_fields(errors, "v2", fb["fields"], "BLETelemetryPacketV2", probe, TELEMETRY_V2)
    for label, got, want in [
        ("v3 totalBytes", v3_total, schema["totalBytes"]),
        ("v3 sizeof", probe["v3.size"], schema["totalBytes"]),
        ("v2 totalBytes", v2_total, fb["totalBytes"]),
        ("v2 sizeof", probe["v2.size"], fb["totalBytes"]),
        ("v3 version", probe["v3.version"], schema["version"]),
        ("currentVersion", probe["v3.version"], schema["versioning"]["currentVersion"]),
        ("v2 version", probe["v2.version"], fb["version"]),
        ("notAvailable", probe["lean.na"], schema["notAvailable"]["int16"]),
        ("age neverReceived", probe["age.never"], schema["age"]["neverReceived"]),
        ("age max", probe["age.max"], schema["age"]["max"]),
    ]:
        if got != want:
            fail(errors, f"{label}: firmware {got} != schema {want}")

    imu = schema["imuBlock"]
    for label, got, want in [
        ("imu version", probe["imu.version"], imu["version"]),
        ("imu headerBytes", probe["imu.header"], imu["headerBytes"]),
        ("imu sampleBytes", probe["imu.sample"], imu["sampleBytes"]),
        ("imu maxSamples", probe["imu.max"], imu["maxSamples"]),
        ("imu samplePeriodMs", probe["imu.period"], imu["samplePeriodMs"]),
        ("imu totalBytesMax", probe["imu.maxBytes"], imu["totalBytesMax"]),
    ]:
        if got != want:
            fail(errors, f"{label}: firmware {got} != schema {want}")
    # mtuRule: min(maxSamples, floor((MTU - 3 - header) / sample)), 0 = suspended
    for mtu, got in zip([23, 40, 185], probe["imu.perBlock"]):
        want = max(0, min(imu["maxSamples"], (mtu - 3 - imu["headerBytes"]) // imu["sampleBytes"]))
        if got != want:
            fail(errors, f"imu samples per block at MTU {mtu}: firmware {got} != schema rule {want}")

    # Decode the probe's packed block with the schema tables only.
    block = bytes.fromhex(probe["imu.block"])
    hdr = {f["name"]: struct.unpack_from("<" + FMT[f["type"]], block, f["offset"])[0] for f in imu["headerFields"]}
    want_hdr = {"version": 1, "seq": 0x33, "deviceTimeMs": 0xA0B0C0D0, "firstSampleIndex": 0x0002,
                "sampleCount": 1, "samplePeriodMs": 10, "flags": 0x02, "reserved": 0}
    if hdr != want_hdr:
        fail(errors, f"imu header decoded with the schema {hdr} != {want_hdr}")
    base = imu["headerBytes"]
    sample = {f["name"]: struct.unpack_from("<" + FMT[f["type"]], block, base + f["offset"])[0] for f in imu["sampleFields"]}
    want_sample = {"ax": 1, "ay": -2, "az": 4096, "gx": -32768, "gy": 655, "gz": 32767}
    if sample != want_sample:
        fail(errors, f"imu sample decoded with the schema {sample} != {want_sample}")
    if len(block) != base + imu["sampleBytes"]:
        fail(errors, f"imu block length {len(block)} != header + 1 sample")

    if errors:
        print("BLE schema check FAILED:")
        for e in errors:
            print("  - " + e)
        return 1
    print("BLE schema check OK: telemetry v3/v2 and IMU block match docs/ble_telemetry_packet_schema.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
