#ifndef TELEMATICS_PARSER_H
#define TELEMATICS_PARSER_H

#include <cstddef>
#include <cstdint>

// E-25 (3) -- The BLE telematics write is parsed on the BLE stack task, so the parser is a
// pure, allocation-free function over the raw ATT value: no Arduino/BLE dependency (it also
// builds on the host), no heap, no std::string and no sscanf. It never reads past `len`;
// the write is not NUL-terminated.

// Longest accepted telematics write. A longer write is dropped as a whole. The largest
// valid write is about 100 bytes ("SONG:" + 31, "|ARTIST:" + 31, "|DIST:65535",
// "|ICON:255"); 160 leaves room for the phone adding a field without making the
// bound a scan cost (E-25 (3), conn#39).
constexpr size_t kTelematicsMaxWrite = 160;

enum class TelematicsEventType : uint8_t { CONNECTION, DATA };

struct TelematicsEvent {
    TelematicsEventType type = TelematicsEventType::DATA;

    // Valid when type == CONNECTION.
    bool connected = false;

    // Valid when type == DATA. Each field is applied in update() only if its
    // "has" flag is set, preserving the original "partial message" semantics
    // (e.g. a DIST-only write must not clear songTitle/artistName).
    bool hasSong = false;
    char songTitle[32] = {0};
    bool hasArtist = false;
    char artistName[32] = {0};
    bool hasDistance = false;
    uint16_t navDistance = 0;
    bool hasIcon = false;
    uint8_t navIconID = 0;
};

/**
 * @brief Parses one telematics BLE write into `out`.
 *
 * Fields "SONG:", "ARTIST:", "DIST:" and "ICON:" may appear in any order, separated by
 * '|'. SONG/ARTIST take up to 31 characters up to the next '|' or the end and are always
 * NUL-terminated. DIST/ICON are decimal integers; DIST must fit uint16_t and ICON uint8_t
 * (an out-of-range value is rejected instead of wrapping). A field that is absent or
 * invalid leaves its "has" flag clear, so a partial write never clears other fields.
 *
 * @return true if at least one field was parsed and set; false otherwise (including a
 *         write longer than kTelematicsMaxWrite, which is dropped entirely).
 */
bool parseTelematicsWrite(const uint8_t* data, size_t len, TelematicsEvent& out);

#endif // TELEMATICS_PARSER_H
