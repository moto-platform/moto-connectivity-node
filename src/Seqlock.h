#ifndef SEQLOCK_H
#define SEQLOCK_H

#include <stdint.h>
#include <string.h>
#include <atomic>
#include <type_traits>

/**
 * @brief Single-writer seqlock for a small trivially copyable struct (D-060 item 2: the
 * GPS task on core 0 publishes, the Arduino loop on core 1 reads). No heap, no lock, the
 * writer never waits.
 *
 * The value is stored as std::atomic<uint32_t> words, so a reader racing the writer reads
 * torn words but never invokes a data race; the sequence number tells it to retry. The
 * sequence is odd while a write is in progress. A reader gives up after `maxTries`
 * inconsistent copies and returns false (the caller keeps its previous copy), so it is
 * bounded too.
 *
 * Ordering: the writer stores the odd sequence, a release fence, the words (relaxed), then
 * the even sequence with release. The reader loads the sequence with acquire, the words
 * (relaxed), an acquire fence, and re-reads the sequence; equal and even means the copy
 * is one complete write. Single writer only; a second writer breaks the protocol.
 */
template <typename T>
class Seqlock {
    static_assert(std::is_trivially_copyable<T>::value, "Seqlock holds plain data only");
    static_assert(sizeof(T) % 4u == 0u, "Seqlock copies whole 32-bit words");

public:
    static constexpr uint32_t kWords = (uint32_t)(sizeof(T) / 4u);

    Seqlock() {
        for (uint32_t i = 0u; i < kWords; i++) {
            _words[i].store(0u, std::memory_order_relaxed);
        }
    }

    // Writer side (one writer only).
    void write(const T& value) {
        uint32_t buf[kWords];
        memcpy(buf, &value, sizeof(T));
        uint32_t seq = _seq.load(std::memory_order_relaxed);
        _seq.store(seq + 1u, std::memory_order_relaxed); // odd: write in progress
        std::atomic_thread_fence(std::memory_order_release);
        for (uint32_t i = 0u; i < kWords; i++) {
            _words[i].store(buf[i], std::memory_order_relaxed);
        }
        _seq.store(seq + 2u, std::memory_order_release); // even: consistent
    }

    // Reader side. Returns false if no consistent copy was obtained in maxTries attempts
    // or nothing was ever written; `out` is left unchanged then.
    bool read(T& out, uint32_t maxTries = 4u) const {
        for (uint32_t attempt = 0u; attempt < maxTries; attempt++) {
            uint32_t before = _seq.load(std::memory_order_acquire);
            if ((before & 1u) != 0u) {
                continue;
            }
            uint32_t buf[kWords];
            for (uint32_t i = 0u; i < kWords; i++) {
                buf[i] = _words[i].load(std::memory_order_relaxed);
            }
            std::atomic_thread_fence(std::memory_order_acquire);
            if (_seq.load(std::memory_order_relaxed) == before) {
                if (before == 0u) {
                    return false; // never written
                }
                memcpy(&out, buf, sizeof(T));
                return true;
            }
        }
        return false;
    }

    // Number of completed writes (wraps); lets a reader see whether anything is new.
    uint32_t writes() const { return _seq.load(std::memory_order_acquire) / 2u; }

private:
    std::atomic<uint32_t> _seq{0u};
    std::atomic<uint32_t> _words[kWords];
};

#endif // SEQLOCK_H
