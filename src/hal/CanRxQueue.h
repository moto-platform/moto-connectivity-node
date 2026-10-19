#ifndef CAN_RX_QUEUE_H
#define CAN_RX_QUEUE_H

#include <stdint.h>

#include "vehicle_cl250.h" // generated: external/moto-vehicle-defs/gen/c/conn/

#ifndef CONN_DISCOVERY_PROBE
#define CONN_DISCOVERY_PROBE 0
#endif

// Depth of the TWAI RX queue (TwaiCanBus), shared with HondaCANModule's bounded drain.
// Deeper than the IDF default (5) so a slow loop pass does not drop a queued ECU answer
// before HondaCANModule drains the queue (Q-018). One queue item is ~16 B.
// D-080 / D-059 probe (safety-reviewer MAJOR-1 on D-059): after the one FC.CTS (BS 0, STmin
// 0) the ECU sends every Consecutive Frame back to back, up to VEHICLE_CL250_FC_MAX_CF_BURST
// (about 10 ms at 500 kbps). Every tester build receives segmented 0x19 answers (D-080), so
// the queue holds the whole burst plus a margin for other traffic and a slow pass; an
// overflow is still seen (rxLostCount) and aborts the reception.
const uint32_t kCanRxQueueLen = 64;
const uint32_t kCanRxQueueBurstMargin = 16;
static_assert(kCanRxQueueLen >= VEHICLE_CL250_FC_MAX_CF_BURST + kCanRxQueueBurstMargin,
              "the TWAI RX queue must hold the CF burst plus a margin");

#endif // CAN_RX_QUEUE_H
