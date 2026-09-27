#ifndef UDS_ISO14229_H
#define UDS_ISO14229_H

#include <stdint.h>

// Protocol constants from ISO 14229-1 (UDS) and ISO 11898 (CAN) that are the same for
// every ECU. They are NOT vehicle definitions: vehicle IDs, DIDs, formulas, timings and
// the D-020 allow-list come only from the generated vehicle_cl250.h (moto-vehicle-defs).
// Whatever is built from these constants still has to pass vehicle_cl250_frame_allowed().

namespace uds {

const uint8_t kSidReadDataByIdentifier = 0x22;  // the only data service this node sends
const uint8_t kSidNegativeResponse = 0x7F;      // [PCI][0x7F][echoed SID][NRC]
const uint8_t kNrcResponsePending = 0x78;       // ECU busy: keep waiting, do not time out

const uint32_t kCanMaxStandardId = 0x7FF;       // above this an identifier is 29-bit

// ISO-TP (ISO 15765-2) single frame: PCI high nibble 0, low nibble = payload length 1..7.
const uint8_t kIsoTpSingleFrameMaxLen = 7;

inline bool isExtendedId(uint32_t id) { return id > kCanMaxStandardId; }

// Human-readable Negative Response Code name (ISO 14229-1 Annex A), for logs only.
inline const char* nrcName(uint8_t nrc) {
    switch (nrc) {
        case 0x10: return "generalReject";
        case 0x11: return "serviceNotSupported";
        case 0x12: return "subFunctionNotSupported";
        case 0x13: return "incorrectMessageLengthOrInvalidFormat";
        case 0x21: return "busyRepeatRequest";
        case 0x22: return "conditionsNotCorrect";
        case 0x24: return "requestSequenceError";
        case 0x31: return "requestOutOfRange";
        case 0x33: return "securityAccessDenied";
        case 0x35: return "invalidKey";
        case 0x78: return "responsePending";
        case 0x7E: return "subFunctionNotSupportedInActiveSession";
        case 0x7F: return "serviceNotSupportedInActiveSession";
        default:   return "unknownNRC";
    }
}

} // namespace uds

#endif // UDS_ISO14229_H
