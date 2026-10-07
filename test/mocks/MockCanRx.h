#ifndef MOCK_CAN_RX_H
#define MOCK_CAN_RX_H

#include "../../src/hal/ICanRx.h"
#include <deque>

/**
 * @brief D-058 item 4 -- in-memory ICanRx test double for the capture tests. Like ICanRx
 * itself it has no transmit path. Only used from test/test_can_capture.
 */
class MockCanRx : public ICanRx {
public:
    bool failBegin = false;
    int beginCalls = 0;
    uint32_t lost = 0; // what rxLostCount() reports

    bool begin() override {
        beginCalls++;
        return !failBegin;
    }

    bool receive(CanFrame& frame) override {
        if (_queue.empty()) {
            return false;
        }
        frame = _queue.front();
        _queue.pop_front();
        return true;
    }

    uint32_t rxLostCount() override { return lost; }

    // --- Test-only control surface ---

    void inject(uint32_t id, bool extended, uint8_t dlc, const uint8_t* data = nullptr) {
        CanFrame f;
        f.id = id;
        f.extended = extended;
        f.dlc = dlc;
        for (uint8_t i = 0; i < dlc && i < 8 && data != nullptr; i++) {
            f.data[i] = data[i];
        }
        _queue.push_back(f);
    }

    size_t pending() const { return _queue.size(); }

private:
    std::deque<CanFrame> _queue;
};

#endif // MOCK_CAN_RX_H
