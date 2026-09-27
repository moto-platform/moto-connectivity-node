#ifndef RTC_TESTER_LATCH_STORE_H
#define RTC_TESTER_LATCH_STORE_H

#include "../TesterLatch.h"

/**
 * @brief Q-018 -- keeps the D-030 poller latch and the bus-off count in RTC no-init
 * memory (see TesterLatch.h for the fail-safe restore rule).
 *
 * Kept across software, task/interrupt-watchdog, panic, USB-Serial-JTAG and shallow
 * brownout resets. A power-on reset clears it (power removed, a deep brownout while
 * cranking, the EN pin, esptool auto-reset over a UART bridge): that is the deliberate
 * human action that re-enables the temporary tester. Only if the node is powered from
 * the ignition does an ignition cycle count as one.
 */
class RtcTesterLatchStore : public ITesterLatchStore {
public:
    TesterLatchStatus load() override;
    void save(TesterLatchReason reason, uint32_t busOffCount) override;
};

#endif // RTC_TESTER_LATCH_STORE_H
