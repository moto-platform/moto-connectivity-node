#ifndef ISOTP_CAN_H
#define ISOTP_CAN_H

#include <stdint.h>

// Transport-layer constants from ISO 11898 (CAN) and ISO 15765-2 (ISO-TP) that are the
// same for every ECU. They are NOT vehicle definitions: vehicle IDs, DIDs, formulas,
// timings and the D-020 allow-list come only from the generated vehicle_cl250.h, and the
// UDS codes from the generated uds_iso14229.h (D-040), both from moto-vehicle-defs.
// Whatever is built from these constants still has to pass vehicle_cl250_frame_allowed().

namespace isotp {

const uint32_t kCanMaxStandardId = 0x7FF;       // above this an identifier is 29-bit

// ISO 15765-2 normal fixed addressing (29-bit 0x18DA<TA><SA> physical, 0x18DB<TA><SA>
// functional): the low byte is the source address. Any SA sending to a watched target
// address is a tester.
const uint32_t kNormalFixedSourceAddressMask = 0xFFu;

// Single frame: PCI high nibble 0, low nibble = payload length 1..7.
const uint8_t kSingleFrameMaxLen = 7;

inline bool isExtendedId(uint32_t id) { return id > kCanMaxStandardId; }

} // namespace isotp

#endif // ISOTP_CAN_H
