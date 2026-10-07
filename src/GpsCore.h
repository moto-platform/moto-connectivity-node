#ifndef GPS_CORE_H
#define GPS_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <atomic>

#include "GpsFix.h"
#include "Seqlock.h"
#include "UbxConfig.h"
#include "UbxParser.h"
#include "hal/IGpsUart.h"

/**
 * @brief Hardware-independent half of the GPS path (D-060 item 2), like CanCaptureCore for
 * the capture probe. Runs in GpsModule's task on core 0: configures the receiver over an
 * IGpsUart, feeds the received bytes to UbxParser and publishes each NAV-PVT through a
 * seqlock. The Arduino loop (core 1) only calls the const readers below.
 *
 * Receiver setup (UBX-CFG to RAM, every start): the NEO-M8N keeps a CFG-PRT baud across a
 * reset of the node alone, so the node first tries kGpsBaud, then the factory 9600 with a
 * CFG-PRT that moves the receiver to kGpsBaud. Any valid UBX answer (ACK or NAV-PVT) proves
 * the link. No position is kept: see UbxParser and GpsFix.
 */
class GpsCore {
public:
    enum Link : uint8_t {
        LINK_PENDING = 0,     // not configured yet
        LINK_OK = 1,          // the receiver answered in UBX at kGpsBaud
        LINK_NO_RECEIVER = 2, // no valid UBX at either baud; retried later
    };

    static constexpr uint32_t kListenMs = 1500u;   // answer window per baud attempt
    static constexpr uint32_t kReadChunkMs = 50u;  // read timeout while listening
    static constexpr size_t kReadChunk = 64u;
    static constexpr uint32_t kPollMs = 100u;      // read timeout of the running link
    static constexpr uint32_t kSilenceMs = 3000u;  // no valid UBX this long: link lost

    // One step of the GPS task: configures while the link is down, else reads once and
    // watches for silence (e.g. the receiver lost power and came back at 9600). Returns
    // false when the receiver did not answer; the task then waits before the next try.
    bool step(IGpsUart& uart) {
        if (link() != LINK_OK) {
            if (!configure(uart)) {
                return false;
            }
            _lastHeardMs = uart.nowMs();
            _lastHeardCount = _parser.messagesOk();
            return true;
        }
        poll(uart, kPollMs);
        uint32_t now = uart.nowMs();
        if (_parser.messagesOk() != _lastHeardCount) {
            _lastHeardCount = _parser.messagesOk();
            _lastHeardMs = now;
        } else if ((uint32_t)(now - _lastHeardMs) >= kSilenceMs) {
            _linkLosses.fetch_add(1u, std::memory_order_relaxed);
            _link.store(LINK_PENDING, std::memory_order_release);
        }
        return true;
    }

    // Task side ------------------------------------------------------------------------

    // Brings the receiver to UBX-only NAV-PVT at 10 Hz on kGpsBaud. Returns true once a
    // valid UBX message arrived at kGpsBaud.
    bool configure(IGpsUart& uart) {
        _attempts.fetch_add(1u, std::memory_order_relaxed);
        // 1. The receiver may already run at kGpsBaud (only the node was reset).
        if (uart.setBaud(ubx::kGpsBaud) && sendConfig(uart) && listen(uart, kListenMs)) {
            return linkUp();
        }
        // 2. Factory default: move it from 9600 to kGpsBaud, then configure there.
        uint8_t frame[ubx::kMaxCfgFrame];
        size_t n = ubx::buildCfgPrtUart1(ubx::kGpsBaud, frame, sizeof(frame));
        if (uart.setBaud(ubx::kModuleDefaultBaud) && uart.write(frame, n)) {
            uart.waitTxDone(100u);
            if (uart.setBaud(ubx::kGpsBaud) && sendConfig(uart) && listen(uart, kListenMs)) {
                return linkUp();
            }
        }
        _link.store(LINK_NO_RECEIVER, std::memory_order_release);
        return false;
    }

    // One read of the running link: up to kReadChunk bytes within timeoutMs.
    void poll(IGpsUart& uart, uint32_t timeoutMs) {
        uint8_t buf[kReadChunk];
        size_t n = uart.read(buf, sizeof(buf), timeoutMs);
        if (uart.takeOverflow()) {
            _uartOverflows.fetch_add(1u, std::memory_order_relaxed);
        }
        feed(buf, n, uart.nowMs());
    }

    // Feeds received bytes; publishes every NAV-PVT, then the counters.
    void feed(const uint8_t* data, size_t len, uint32_t nowMs) {
        for (size_t i = 0u; i < len; i++) {
            if (_parser.feed(data[i]) == UbxParser::NAV_PVT) {
                GpsFix fix = _parser.lastFix();
                fix.rxTimeMs = nowMs;
                _fix.write(fix);
            }
        }
        _navPvt.store(_parser.navPvtCount(), std::memory_order_relaxed);
        _checksumErrors.store(_parser.checksumErrors(), std::memory_order_relaxed);
        _lengthErrors.store(_parser.lengthErrors(), std::memory_order_relaxed);
        _messagesOk.store(_parser.messagesOk(), std::memory_order_relaxed);
    }

    // Reader side (any core) -----------------------------------------------------------

    // The latest NAV-PVT; false if none yet or no consistent copy (keep the previous one).
    bool latest(GpsFix& out) const { return _fix.read(out); }

    Link link() const { return (Link)_link.load(std::memory_order_acquire); }
    uint32_t configureAttempts() const { return _attempts.load(std::memory_order_relaxed); }
    uint32_t linkLosses() const { return _linkLosses.load(std::memory_order_relaxed); }
    uint32_t navPvtCount() const { return _navPvt.load(std::memory_order_relaxed); }
    uint32_t messagesOk() const { return _messagesOk.load(std::memory_order_relaxed); }
    uint32_t checksumErrors() const { return _checksumErrors.load(std::memory_order_relaxed); }
    uint32_t lengthErrors() const { return _lengthErrors.load(std::memory_order_relaxed); }
    uint32_t uartOverflows() const { return _uartOverflows.load(std::memory_order_relaxed); }

private:
    bool linkUp() {
        _link.store(LINK_OK, std::memory_order_release);
        return true;
    }

    static bool sendConfig(IGpsUart& uart) {
        uint8_t frame[ubx::kMaxCfgFrame];
        size_t n = ubx::buildCfgPrtUart1(ubx::kGpsBaud, frame, sizeof(frame));
        bool ok = uart.write(frame, n);
        uart.waitTxDone(100u); // the receiver applies CFG-PRT before the next frames
        n = ubx::buildCfgRate(frame, sizeof(frame));
        ok = ok && uart.write(frame, n);
        n = ubx::buildCfgMsgNavPvt(frame, sizeof(frame));
        ok = ok && uart.write(frame, n);
        uart.waitTxDone(100u);
        return ok;
    }

    // True if a valid UBX message arrives within windowMs.
    bool listen(IGpsUart& uart, uint32_t windowMs) {
        uint32_t before = _parser.messagesOk();
        uint32_t start = uart.nowMs();
        while ((uint32_t)(uart.nowMs() - start) < windowMs) {
            poll(uart, kReadChunkMs);
            if (_parser.messagesOk() != before) {
                return true;
            }
        }
        return false;
    }

    UbxParser _parser; // task only
    uint32_t _lastHeardMs = 0u;    // task only
    uint32_t _lastHeardCount = 0u; // task only
    Seqlock<GpsFix> _fix;
    std::atomic<uint8_t> _link{LINK_PENDING};
    std::atomic<uint32_t> _attempts{0u};
    std::atomic<uint32_t> _linkLosses{0u};
    std::atomic<uint32_t> _navPvt{0u};
    std::atomic<uint32_t> _messagesOk{0u};
    std::atomic<uint32_t> _checksumErrors{0u};
    std::atomic<uint32_t> _lengthErrors{0u};
    std::atomic<uint32_t> _uartOverflows{0u};
};

#endif // GPS_CORE_H
