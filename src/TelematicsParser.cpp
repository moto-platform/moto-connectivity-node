#include "TelematicsParser.h"

namespace {

// First occurrence of `key` in data[0..len), or nullptr. Unlike strstr this never looks
// past `len` and needs no terminator.
const uint8_t* findKey(const uint8_t* data, size_t len, const char* key, size_t keyLen) {
    if (len < keyLen) {
        return nullptr;
    }
    for (size_t i = 0; i + keyLen <= len; i++) {
        size_t k = 0;
        while (k < keyLen && data[i + k] == (uint8_t)key[k]) {
            k++;
        }
        if (k == keyLen) {
            return data + i;
        }
    }
    return nullptr;
}

// Copies the value after a text key up to the next '|' or the end, at most 31 characters
// (the same bound as the old sscanf "%31[^|]"), and always NUL-terminates.
void copyTextField(const uint8_t* data, size_t len, size_t start, char out[32]) {
    size_t n = 0;
    while (start + n < len && n < 31) {
        const uint8_t c = data[start + n];
        if (c == '|' || c == '\0') {
            break;
        }
        out[n] = (char)c;
        n++;
    }
    out[n] = '\0';
}

// Decimal integer after a numeric key, sscanf("%d")-like: optional whitespace and sign,
// then at least one digit, stopping at the first non-digit. The value saturates far above
// any accepted range so a long input cannot overflow.
bool parseDecimal(const uint8_t* data, size_t len, size_t start, long& value) {
    size_t i = start;
    while (i < len && (data[i] == ' ' || data[i] == '\t')) {
        i++;
    }
    bool negative = false;
    if (i < len && (data[i] == '-' || data[i] == '+')) {
        negative = data[i] == '-';
        i++;
    }
    if (i >= len || data[i] < '0' || data[i] > '9') {
        return false;
    }
    long v = 0;
    while (i < len && data[i] >= '0' && data[i] <= '9') {
        if (v < 100000L) {
            v = v * 10 + (long)(data[i] - '0');
        }
        i++;
    }
    value = negative ? -v : v;
    return true;
}

} // namespace

bool parseTelematicsWrite(const uint8_t* data, size_t len, TelematicsEvent& out) {
    out = TelematicsEvent();
    out.type = TelematicsEventType::DATA;
    if (data == nullptr || len == 0 || len > kTelematicsMaxWrite) {
        return false;
    }

    // The ATT value is not NUL-terminated. An embedded NUL ended the string for the old
    // std::string-based parser (c_str()), so it ends the scanned range here too.
    size_t textLen = len;
    for (size_t i = 0; i < len; i++) {
        if (data[i] == '\0') {
            textLen = i;
            break;
        }
    }

    const uint8_t* p = findKey(data, textLen, "SONG:", 5);
    if (p != nullptr) {
        out.hasSong = true;
        copyTextField(data, textLen, (size_t)(p - data) + 5, out.songTitle);
    }

    p = findKey(data, textLen, "ARTIST:", 7);
    if (p != nullptr) {
        out.hasArtist = true;
        copyTextField(data, textLen, (size_t)(p - data) + 7, out.artistName);
    }

    long value = 0;
    p = findKey(data, textLen, "DIST:", 5);
    if (p != nullptr && parseDecimal(data, textLen, (size_t)(p - data) + 5, value) &&
        value >= 0 && value <= 65535) {
        out.navDistance = (uint16_t)value;
        out.hasDistance = true;
    }

    p = findKey(data, textLen, "ICON:", 5);
    if (p != nullptr && parseDecimal(data, textLen, (size_t)(p - data) + 5, value) &&
        value >= 0 && value <= 255) {
        out.navIconID = (uint8_t)value;
        out.hasIcon = true;
    }

    return out.hasSong || out.hasArtist || out.hasDistance || out.hasIcon;
}
