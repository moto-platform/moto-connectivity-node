#ifndef UBX_PARSER_H
#define UBX_PARSER_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief Byte-wise UBX parser for the u-blox NEO-M8N (D-060 item 2). Pure C++11, no heap,
 * no strings (invariant 5), no ESP-IDF: GpsModule feeds it from the UART task on core 0
 * and the native tests feed it directly.
 *
 * Frame: 0xB5 0x62, class, id, length (uint16 LE), payload, CK_A, CK_B. The 8-bit Fletcher
 * checksum runs over class, id, length and payload (u-blox 8 / M8 Receiver Description
 * incl. Protocol Specification, "UBX Checksum"). Only a NAV-PVT payload is stored (static
 * buffer); any other message (ACK, NAK) is checksummed without storing it. A length above
 * kMaxLength is taken as a false sync and dropped; a bad checksum is counted and dropped.
 * Either way the parser looks for the next sync pair, so garbage or a NMEA sentence costs
 * only that message. A 0xB5 0x62 inside garbage right before a real frame can cost that
 * frame too (its header bytes are consumed as the false frame's body).
 *
 * Only UBX-NAV-PVT (0x01 0x07, 92 bytes) is decoded, and only into the fields that may
 * leave the node (D-060 item 3): ground speed, heading of motion, their accuracies, fix
 * type, gnssFixOK and the satellite count. Latitude, longitude and height are never
 * copied out, and the payload buffer is zeroed after every message, so no position stays
 * in the parser between messages (invariant 7).
 */

#include "GpsFix.h"

namespace ubx {

constexpr uint8_t kSync1 = 0xB5u;
constexpr uint8_t kSync2 = 0x62u;
constexpr uint8_t kClassNav = 0x01u;
constexpr uint8_t kIdNavPvt = 0x07u;
constexpr uint16_t kNavPvtLength = 92u;

// NAV-PVT payload offsets (u-blox M8 protocol spec, UBX-NAV-PVT). Bytes 24..47 (lon, lat,
// height, hMSL, hAcc, vAcc) are deliberately not listed: nothing reads them.
constexpr size_t kPvtITow = 0u;
constexpr size_t kPvtFixType = 20u;
constexpr size_t kPvtFlags = 21u;
constexpr size_t kPvtNumSv = 23u;
constexpr size_t kPvtGSpeed = 60u;
constexpr size_t kPvtHeadMot = 64u;
constexpr size_t kPvtSAcc = 68u;
constexpr size_t kPvtHeadAcc = 72u;

inline uint32_t readU32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8u) | ((uint32_t)p[2] << 16u) |
           ((uint32_t)p[3] << 24u);
}

inline int32_t readI32(const uint8_t* p) {
    uint32_t u = readU32(p);
    int32_t v;
    memcpy(&v, &u, sizeof(v));
    return v;
}

// Running 8-bit Fletcher checksum (CK_A, CK_B).
struct Fletcher8 {
    uint8_t a = 0;
    uint8_t b = 0;
    void reset() {
        a = 0;
        b = 0;
    }
    void add(uint8_t byte) {
        a = (uint8_t)(a + byte);
        b = (uint8_t)(b + a);
    }
};

} // namespace ubx

class UbxParser {
public:
    // Payload buffer: NAV-PVT only (92 bytes in the M8 layout).
    static constexpr uint16_t kMaxPayload = ubx::kNavPvtLength;
    // Longest message accepted at all (checksummed, not stored); a longer length field is
    // a false sync. The node enables nothing longer than NAV-PVT.
    static constexpr uint16_t kMaxLength = 512u;

    enum Result : uint8_t {
        NONE = 0,     // byte consumed, no complete message yet
        NAV_PVT = 1,  // a valid NAV-PVT was decoded into the fix (see lastFix())
        OTHER = 2,    // a valid UBX message other than NAV-PVT (e.g. an ACK), ignored
        ERROR = 3,    // a frame was dropped (checksum or length), counted
    };

    // Feeds one byte; returns what, if anything, that byte completed.
    Result feed(uint8_t byte) {
        Result result = NONE;
        switch (_state) {
        case SYNC1:
            if (byte == ubx::kSync1) {
                _state = SYNC2;
            }
            break;
        case SYNC2:
            if (byte == ubx::kSync2) {
                _state = CLASS;
            } else if (byte != ubx::kSync1) { // 0xB5 0xB5 0x62: stay ready for 0x62
                _state = SYNC1;
            }
            break;
        case CLASS:
            _ck.reset();
            _ck.add(byte);
            _class = byte;
            _state = ID;
            break;
        case ID:
            _ck.add(byte);
            _id = byte;
            _state = LEN1;
            break;
        case LEN1:
            _ck.add(byte);
            _length = byte;
            _state = LEN2;
            break;
        case LEN2:
            _ck.add(byte);
            _length = (uint16_t)(_length | ((uint16_t)byte << 8u));
            _index = 0u;
            _store = (_class == ubx::kClassNav) && (_id == ubx::kIdNavPvt) &&
                     (_length == ubx::kNavPvtLength);
            if (_length > kMaxLength) {
                // A false sync (or a message the node never enabled): resync on the next
                // sync pair; the following bytes are scanned for it like garbage.
                _lengthErrors++;
                result = ERROR;
                _state = SYNC1;
            } else {
                _state = (_length == 0u) ? CK_A : PAYLOAD;
            }
            break;
        case PAYLOAD:
            _ck.add(byte);
            if (_store) {
                _payload[_index] = byte;
            }
            _index++;
            if (_index >= _length) {
                _state = CK_A;
            }
            break;
        case CK_A:
            _rxCkA = byte;
            _state = CK_B;
            break;
        case CK_B:
        default:
            result = finish(byte);
            _state = SYNC1;
            break;
        }
        return result;
    }

    // The last decoded NAV-PVT. Valid after feed() returned NAV_PVT.
    const GpsFix& lastFix() const { return _fix; }

    // Class and id of the last valid message (any type).
    uint8_t lastClass() const { return _lastClass; }
    uint8_t lastId() const { return _lastId; }

    uint32_t messagesOk() const { return _messagesOk; }
    uint32_t navPvtCount() const { return _navPvt; }
    uint32_t checksumErrors() const { return _checksumErrors; }
    uint32_t lengthErrors() const { return _lengthErrors; }

#ifdef CONN_NATIVE_TEST
    // Test hook: the payload buffer is cleared after every message (no position kept).
    bool payloadIsClear() const {
        for (uint16_t i = 0u; i < kMaxPayload; i++) {
            if (_payload[i] != 0u) {
                return false;
            }
        }
        return true;
    }
#endif

private:
    enum State : uint8_t { SYNC1, SYNC2, CLASS, ID, LEN1, LEN2, PAYLOAD, CK_A, CK_B };

    Result finish(uint8_t ckB) {
        Result result;
        if ((_rxCkA != _ck.a) || (ckB != _ck.b)) {
            _checksumErrors++;
            result = ERROR;
        } else if ((_class == ubx::kClassNav) && (_id == ubx::kIdNavPvt)) {
            if (_length == ubx::kNavPvtLength) {
                decodeNavPvt();
                _messagesOk++;
                _navPvt++;
                result = NAV_PVT;
            } else {
                _lengthErrors++; // a NAV-PVT of another length is not the M8 layout
                result = ERROR;
            }
        } else {
            _messagesOk++;
            result = OTHER;
        }
        if ((result == NAV_PVT) || (result == OTHER)) {
            _lastClass = _class;
            _lastId = _id;
        }
        wipePayload();
        return result;
    }

    // Volatile stores: the wipe must not be dropped as a dead store (no position may stay
    // in RAM after the message, invariant 7).
    void wipePayload() {
        volatile uint8_t* p = _payload;
        for (uint16_t i = 0u; i < kMaxPayload; i++) {
            p[i] = 0u;
        }
    }

    void decodeNavPvt() {
        GpsFix fix;
        fix.iTowMs = ubx::readU32(&_payload[ubx::kPvtITow]);
        fix.fixType = _payload[ubx::kPvtFixType];
        fix.gnssFixOk = (uint8_t)(_payload[ubx::kPvtFlags] & 0x01u);
        fix.numSv = _payload[ubx::kPvtNumSv];
        fix.groundSpeedMmps = ubx::readI32(&_payload[ubx::kPvtGSpeed]);
        fix.headingMotionE5 = ubx::readI32(&_payload[ubx::kPvtHeadMot]);
        fix.speedAccMmps = ubx::readU32(&_payload[ubx::kPvtSAcc]);
        fix.headingAccE5 = ubx::readU32(&_payload[ubx::kPvtHeadAcc]);
        _fix = fix;
    }

    State _state = SYNC1;
    uint8_t _class = 0u;
    uint8_t _id = 0u;
    uint16_t _length = 0u;
    uint16_t _index = 0u;
    uint8_t _rxCkA = 0u;
    bool _store = false;
    ubx::Fletcher8 _ck;
    uint8_t _payload[kMaxPayload] = {};
    GpsFix _fix;

    uint8_t _lastClass = 0u;
    uint8_t _lastId = 0u;
    uint32_t _messagesOk = 0u;
    uint32_t _navPvt = 0u;
    uint32_t _checksumErrors = 0u;
    uint32_t _lengthErrors = 0u;
};

#endif // UBX_PARSER_H
