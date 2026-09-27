#ifndef MOCK_CAN_BUS_H
#define MOCK_CAN_BUS_H

#include "../../src/hal/ICanBus.h"
#include <deque>
#include <vector>

/**
 * @brief G3.2/G5.1 -- In-memory ICanBus test double. Lets test code simulate a Honda
 * ECU (injectRxFrame) and inspect exactly what HondaCANModule transmitted (txLog),
 * with no real CAN hardware. Only used from test/test_can_protocol; never linked into
 * ESP32 firmware.
 */
class MockCanBus : public ICanBus {
public:
    enum class Mode { NOT_INSTALLED, LISTEN_ONLY, NORMAL };

    std::vector<CanFrame> txLog;
    Mode mode = Mode::NOT_INSTALLED;
    int beginListenOnlyCallCount = 0;
    int enterNormalModeCallCount = 0;
    int txOutsideNormalMode = 0; // transmit() attempts a real TWAI would refuse
    bool failBeginListenOnly = false;
    bool failEnterNormalMode = false;
    int stopCallCount = 0;
    uint32_t rxLost = 0; // what rxLostCount() reports (queue overflow simulation)
    int initiateRecoveryCallCount = 0;
    int startCallCount = 0;

    bool beginListenOnly() override {
        beginListenOnlyCallCount++;
        if (failBeginListenOnly) return false;
        mode = Mode::LISTEN_ONLY;
        return true;
    }

    bool enterNormalMode() override {
        enterNormalModeCallCount++;
        if (failEnterNormalMode) {
            mode = Mode::NOT_INSTALLED;
            return false;
        }
        mode = Mode::NORMAL;
        return true;
    }

    bool transmit(const CanFrame& frame) override {
        if (mode != Mode::NORMAL) {
            txOutsideNormalMode++; // never reaches txLog: nothing was put on the bus
            return false;
        }
        txLog.push_back(frame);
        return true;
    }

    bool receive(CanFrame& frame) override {
        if (_rxQueue.empty()) {
            return false;
        }
        frame = _rxQueue.front();
        _rxQueue.pop_front();
        return true;
    }

    CanBusState getState() override {
        return _state;
    }

    uint32_t rxLostCount() override {
        return rxLost;
    }

    void getErrorCounters(uint16_t& txErrorCount, uint16_t& rxErrorCount) override {
        txErrorCount = _txErrorCount;
        rxErrorCount = _rxErrorCount;
    }

    bool initiateRecovery() override {
        initiateRecoveryCallCount++;
        return true;
    }

    bool start() override {
        startCallCount++;
        return true;
    }

    void stop() override {
        stopCallCount++;
        mode = Mode::NOT_INSTALLED;
    }

    // --- Test-only control surface (not part of ICanBus) ---

    void injectRxFrame(const CanFrame& frame) {
        _rxQueue.push_back(frame);
    }

    void setState(CanBusState state) {
        _state = state;
    }

    void setErrorCounters(uint16_t txErrorCount, uint16_t rxErrorCount) {
        _txErrorCount = txErrorCount;
        _rxErrorCount = rxErrorCount;
    }

    // How many frames in txLog requested a given DID (SID 0x22 ReadDataByIdentifier),
    // regardless of 11-bit/29-bit framing -- HondaCANModule sends both per request.
    int countDidRequests(uint16_t did) const {
        int count = 0;
        for (const CanFrame& f : txLog) {
            if (f.dlc >= 4 && f.data[1] == 0x22 &&
                ((f.data[2] << 8) | f.data[3]) == did) {
                count++;
            }
        }
        return count;
    }

    size_t rxPending() const {
        return _rxQueue.size();
    }

    void clearTxLog() {
        txLog.clear();
    }

private:
    std::deque<CanFrame> _rxQueue;
    CanBusState _state = CanBusState::RUNNING;
    uint16_t _txErrorCount = 0;
    uint16_t _rxErrorCount = 0;
};

#endif // MOCK_CAN_BUS_H
