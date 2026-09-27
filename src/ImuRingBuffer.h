#ifndef IMU_RING_BUFFER_H
#define IMU_RING_BUFFER_H

#include <stdint.h>
#include <atomic>

// One raw IMU sample, taken at a 100 Hz timer tick (ImuModule). Raw int16 counts in the
// scale given by docs/ble_telemetry_packet_schema.json `imuBlock.scale`.
struct ImuSample {
    uint32_t index = 0;   // timer tick index; consecutive samples differ by exactly 1
    uint32_t timeMs = 0;  // node clock (millis()) of that tick
    int16_t accel[3] = {0, 0, 0};
    int16_t gyro[3] = {0, 0, 0};
};

// Sticky event bits reported with the next IMU block (schema `imuBlock.flags`).
constexpr uint8_t IMU_EVENT_OVERFLOW = 1u << 0;
constexpr uint8_t IMU_EVENT_READ_ERROR = 1u << 1;
constexpr uint8_t IMU_EVENT_SENSOR_RECONFIGURED = 1u << 2;

/**
 * @brief Fixed-size single-producer/single-consumer ring of IMU samples. Static storage,
 * no heap, no locks: the IMU sampler task is the only producer (push, markReadError),
 * the main loop (BLEServerModule) the only consumer (size, peek, drop, takeEvents).
 *
 * When full, push() drops the NEW sample and keeps the older unsent ones; the gap shows
 * up downstream as a jump in the sample index (and IMU_EVENT_OVERFLOW).
 */
template <uint32_t N>
class ImuRingBuffer {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "capacity must be a power of two");

public:
    static constexpr uint32_t kCapacity = N;

    // Producer side.
    bool push(const ImuSample& s) {
        uint32_t head = _head.load(std::memory_order_relaxed);
        uint32_t tail = _tail.load(std::memory_order_acquire);
        if (head - tail >= N) {
            _overflowCount.fetch_add(1, std::memory_order_relaxed);
            _events.fetch_or(IMU_EVENT_OVERFLOW, std::memory_order_relaxed);
            return false;
        }
        _buf[head & (N - 1)] = s;
        _head.store(head + 1, std::memory_order_release);
        return true;
    }

    // Producer side: sets IMU_EVENT_* bits reported with the next block.
    void markEvent(uint8_t bits) { _events.fetch_or(bits, std::memory_order_relaxed); }
    void markReadError() { markEvent(IMU_EVENT_READ_ERROR); }

    // Consumer side.
    uint32_t size() const {
        return _head.load(std::memory_order_acquire) - _tail.load(std::memory_order_relaxed);
    }

    // i-th oldest unread sample; only valid for i < size().
    const ImuSample& peek(uint32_t i) const {
        return _buf[(_tail.load(std::memory_order_relaxed) + i) & (N - 1)];
    }

    void drop(uint32_t n) {
        uint32_t available = size();
        if (n > available) {
            n = available;
        }
        _tail.store(_tail.load(std::memory_order_relaxed) + n, std::memory_order_release);
    }

    void clear() { drop(size()); }

    // Returns and clears the IMU_EVENT_* bits collected since the previous call.
    uint8_t takeEvents() { return _events.exchange(0, std::memory_order_relaxed); }

    uint32_t overflowCount() const { return _overflowCount.load(std::memory_order_relaxed); }

private:
    ImuSample _buf[N];
    std::atomic<uint32_t> _head{0};
    std::atomic<uint32_t> _tail{0};
    std::atomic<uint32_t> _overflowCount{0};
    std::atomic<uint8_t> _events{0};
};

// 64 samples = 640 ms at 100 Hz: covers several missed 100 ms notify periods.
constexpr uint32_t IMU_RING_CAPACITY = 64;
using ImuRing = ImuRingBuffer<IMU_RING_CAPACITY>;

#endif // IMU_RING_BUFFER_H
