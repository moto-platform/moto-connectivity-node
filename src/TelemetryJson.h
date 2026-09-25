#ifndef TELEMETRY_JSON_H
#define TELEMETRY_JSON_H

#include <stddef.h>

#include "SystemState.h"

// Worst case of buildTelemetryJson(): numbers at their widest plus two 31-character
// strings where every character is escaped. WiFiServerModule's static buffer uses this.
constexpr size_t TELEMETRY_JSON_MAX_LEN = 512;

/**
 * @brief Writes the /api/telemetry JSON document for `state` into `buf` (static buffer,
 * no heap, no Arduino String). A stale value is written as null, like the BLE packet's
 * validity flags; lean fields are always null (no lean source on this node, D-023).
 * @return Length written (excluding the NUL), or 0 if it did not fit (buf then holds "").
 */
size_t buildTelemetryJson(const SystemState& state, char* buf, size_t cap);

/**
 * @brief G4.3 -- Copies `in` into `out` as JSON string content: escapes '"' and '\\',
 * drops control and non-ASCII bytes. Always NUL-terminates when cap > 0.
 * @return Length written, or 0 with out = "" if it did not fit.
 */
size_t jsonEscape(const char* in, char* out, size_t cap);

#endif // TELEMETRY_JSON_H
