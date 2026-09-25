#ifndef NATIVE_TEST_ARDUINO_STUB_H
#define NATIVE_TEST_ARDUINO_STUB_H

// G5.1/G5.2 -- Minimal Arduino.h stand-in so header-only, hardware-independent files
// (SystemState.h, BLETelemetryPacket.h) can be compiled and unit-tested on the host
// (platform = native) with no ESP32 toolchain or device attached. Only on the native
// build's include path (see platformio.ini's [env:native] build_flags) -- the real
// ESP32 environments never see this file, so this cannot affect on-device behavior.
//
// Deliberately NOT a general-purpose Arduino shim: add symbols here only as the
// specific headers under test require them.

#include <stdint.h>
#include <stddef.h>
#include <cstdio>
#include <cstdarg>
#include <algorithm>

// SystemState.h's isStale() calls millis(). Tests control the fake clock directly
// instead of sleeping in real time, so results are deterministic.
inline unsigned long& native_millis_ref() {
    static unsigned long fakeMillis = 0;
    return fakeMillis;
}

inline unsigned long millis() {
    return native_millis_ref();
}

// Test helper (not part of the real Arduino API) -- lets test code set the fake
// clock explicitly, e.g. to simulate "500ms have passed since the last update".
inline void test_setMillis(unsigned long ms) {
    native_millis_ref() = ms;
}

// HondaCANModule::begin() calls delay(200)/delay(50) between session-start frames.
// No real hardware timing to respect on the host -- a no-op keeps tests fast and
// deterministic (they never actually need to wait).
inline void delay(unsigned long) {}

// Arduino's min()/max() are used by HondaCANModule for backoff/timeout capping.
using std::min;
using std::max;

// Minimal Serial stand-in -- HondaCANModule logs status/warnings through this.
// Tests don't assert on log output, so this just forwards to stdout; it exists
// purely so those calls compile and don't crash on the host.
struct NativeSerialStub {
    void println(const char* s) { std::puts(s); }
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        std::vprintf(fmt, args);
        va_end(args);
    }
};
// static (not C++17 `inline` variable) so this header stays valid under -std=c++11.
static NativeSerialStub Serial;

#endif // NATIVE_TEST_ARDUINO_STUB_H
