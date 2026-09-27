#ifndef MOCK_TESTER_LATCH_STORE_H
#define MOCK_TESTER_LATCH_STORE_H

#include "../../src/TesterLatch.h"

/**
 * @brief Q-018 -- In-memory stand-in for the RTC no-init latch record. It keeps the raw
 * record, so the tests go through the same encode/decode/restore path as the ESP32.
 * A new store starts as after a power-on reset; a "reset" is a new HondaCANModule built
 * on the same store (not power-on), and powerOn() makes the next boot a power-on one.
 */
class MockTesterLatchStore : public ITesterLatchStore {
public:
    TesterLatchRecord record = {0, 0, 0, 0, 0};
    bool powerOnAtNextLoad = true;
    int saveCallCount = 0;

    TesterLatchStatus load() override {
        TesterLatchStatus status = tester_latch::restore(record, powerOnAtNextLoad);
        powerOnAtNextLoad = false;
        return status;
    }

    void save(TesterLatchReason reason, uint32_t busOffCount) override {
        saveCallCount++;
        tester_latch::encode(reason, busOffCount, record);
    }

    void powerOn() {
        powerOnAtNextLoad = true;
    }

    // What the next non-power-on boot would restore, without consuming a boot.
    TesterLatchStatus peek() const {
        TesterLatchRecord copy = record;
        return tester_latch::restore(copy, false);
    }
};

#endif // MOCK_TESTER_LATCH_STORE_H
