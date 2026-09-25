#include "TelemetryJson.h"

#include <stdarg.h>
#include <stdio.h>

namespace {
// Appends printf-style text; once anything does not fit, the whole document is invalid.
struct JsonWriter {
    char* buf;
    size_t cap;
    size_t len;
    bool overflow;

    void append(const char* fmt, ...) {
        if (overflow) {
            return;
        }
        va_list args;
        va_start(args, fmt);
        int n = vsnprintf(buf + len, cap - len, fmt, args);
        va_end(args);
        if (n < 0 || (size_t)n >= cap - len) {
            overflow = true;
            return;
        }
        len += (size_t)n;
    }

    // "key":value or "key":null depending on freshness.
    void number(const char* key, bool fresh, const char* fmt, double value) {
        if (!fresh) {
            append("\"%s\":null,", key);
            return;
        }
        append("\"%s\":", key);
        append(fmt, value);
        append(",");
    }
};
} // namespace

size_t jsonEscape(const char* in, char* out, size_t cap) {
    if (cap == 0) {
        return 0;
    }
    size_t o = 0;
    for (size_t i = 0; in[i] != '\0'; i++) {
        unsigned char c = (unsigned char)in[i];
        if (c == '"' || c == '\\') {
            if (o + 2 >= cap) {
                out[0] = '\0';
                return 0;
            }
            out[o++] = '\\';
            out[o++] = (char)c;
        } else if (c >= 0x20 && c < 0x7F) {
            if (o + 1 >= cap) {
                out[0] = '\0';
                return 0;
            }
            out[o++] = (char)c;
        }
        // else: drop control/non-ASCII bytes entirely
    }
    out[o] = '\0';
    return o;
}

size_t buildTelemetryJson(const SystemState& state, char* buf, size_t cap) {
    if (buf == nullptr || cap == 0) {
        return 0;
    }
    const EngineData& e = state.engine;
    const TelematicsData& t = state.telematics;
    // Escaped copies of the phone-supplied strings: at most 2 bytes per input byte.
    char song[2 * sizeof(t.songTitle)];
    char artist[2 * sizeof(t.artistName)];
    jsonEscape(t.songTitle, song, sizeof(song));
    jsonEscape(t.artistName, artist, sizeof(artist));

    JsonWriter w{buf, cap, 0, false};
    w.append("{");
    w.number("rpm", !isStale(e.rpmUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_DID_INDEX_ENGINE_SPEED)),
             "%.1f", e.rpm);
    w.number("speed", !isStale(e.speedUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_DID_INDEX_VEHICLE_SPEED)),
             "%.0f", (double)e.speed);
    w.number("coolantTemp",
             !isStale(e.coolantTempUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_DID_INDEX_COOLANT_TEMPERATURE)),
             "%.0f", (double)e.coolantTemp);
    w.number("throttlePos",
             !isStale(e.throttlePosUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_DID_INDEX_THROTTLE_POSITION)),
             "%.1f", e.throttlePos);
    w.number("batteryVoltage",
             !isStale(e.batteryVoltageUpdatedMs, didStaleThresholdMs(VEHICLE_CL250_DID_INDEX_BATTERY_VOLTAGE)),
             "%.2f", e.batteryVoltage);
    w.append("\"leanAngle\":null,\"maxLeanLeft\":null,\"maxLeanRight\":null,");
    w.append("\"ecuPresent\":%s,", e.ecuPresent ? "true" : "false");
    w.append("\"phoneConnected\":%s,", t.phoneConnected ? "true" : "false");
    w.append("\"songTitle\":\"%s\",\"artistName\":\"%s\"}", song, artist);

    if (w.overflow) {
        buf[0] = '\0';
        return 0;
    }
    return w.len;
}
