#include "RtcTesterLatchStore.h"
#include <esp_attr.h>
#include <esp_system.h>

namespace {
// Not zeroed by the startup code, so it keeps its content across every reset except
// power-on. Only this file touches it.
RTC_NOINIT_ATTR TesterLatchRecord s_testerLatch;
} // namespace

TesterLatchStatus RtcTesterLatchStore::load() {
    // Power-on (incl. the EN pin / auto-reset over a UART bridge) re-arms the tester.
    // Any other reset keeps the record; an invalid one reads as latched (UNKNOWN).
    return tester_latch::restore(s_testerLatch, esp_reset_reason() == ESP_RST_POWERON);
}

void RtcTesterLatchStore::save(TesterLatchReason reason, uint32_t busOffCount) {
    tester_latch::encode(reason, busOffCount, s_testerLatch);
}
