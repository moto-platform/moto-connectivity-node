#ifndef CAN_CAPTURE_CORE_H
#define CAN_CAPTURE_CORE_H

#include <stddef.h>
#include <stdint.h>

#include "hal/ICanRx.h"
#include "IsoTpCan.h"
#include "vehicle_cl250.h" // generated: vehicle_cl250_functional_watch (D-040)

// D-058 item 4 -- the pure logic of the CAN capture probe: no Arduino, no driver, no heap.
// It drains an ICanRx (receive-only), streams one text line per frame to an ICaptureSink and
// keeps a static per-ID statistics table, printed as summaries. Everything is driven by the
// timestamp the caller passes to step(), so the native tests run it with a fake clock.
//
// Output lines (text, '\n' terminated; lines starting with '#' are comments):
//   F,<t_us>,<id hex>,<S|X>,<dlc>,<data hex>
//        one received frame; t_us = node clock in microseconds at the pass that drained it,
//        id = 3 hex digits for a standard (S) id, 8 for an extended (X) id.
//   S,<t_us>,<id hex>,<S|X>,<count>,<min_ms>,<max_ms>,<W|->
//        per-ID summary row, every 10 s; min/max = shortest/longest gap between two frames
//        of that ID in ms (3 decimals, '-' until the second frame); W = watched ID (D-040:
//        the OBD functional request IDs; a frame there means a tester is on the bus).
//   S,END,<t_us>,frames=<n>,ids=<n>,overflow=<n>,lost=<n>,dropped=<n>
//        closes a summary: frames seen, IDs tracked, frames of IDs beyond the table,
//        frames lost by the driver, lines dropped because the serial buffer was full.
//   FINAL,...  the same rows and END line once, after 300 s of capture; the capture goes on.
// Nothing here ever blocks: a line that does not fit the sink's free space is counted as
// dropped (frame lines) or retried on the next pass (summary lines).

constexpr uint8_t CAPTURE_TABLE_SIZE = 64;
constexpr uint8_t CAPTURE_MAX_FRAMES_PER_STEP = 64; // bounded drain per pass, like the tester
constexpr uint32_t CAPTURE_SUMMARY_PERIOD_US = 10u * 1000u * 1000u;
constexpr uint64_t CAPTURE_FINAL_AFTER_US = 300ull * 1000ull * 1000ull;
constexpr uint32_t CAPTURE_NO_PERIOD = 0xFFFFFFFFu; // minPeriodUs while fewer than 2 frames

/** @brief Where the capture writes its lines (USB serial on the device). */
class ICaptureSink {
public:
    virtual ~ICaptureSink() {}
    /** @brief Bytes that can be written right now without blocking. */
    virtual size_t availableForWrite() = 0;
    virtual void write(const char* data, size_t len) = 0;
};

struct CanIdStats {
    uint32_t id = 0;
    bool extended = false;
    bool watched = false;
    uint32_t count = 0;
    uint64_t lastUs = 0;
    uint32_t minPeriodUs = CAPTURE_NO_PERIOD;
    uint32_t maxPeriodUs = 0;
};

class CanCaptureCore {
public:
    CanCaptureCore(ICanRx& rx, ICaptureSink& sink) : _rx(rx), _sink(sink) {}

    /** @brief One pass: continue a pending summary, drain RX (bounded), start a due summary. */
    void step(uint64_t nowUs) {
        if (!_started) {
            _started = true;
            _nextSummaryUs = nowUs + CAPTURE_SUMMARY_PERIOD_US;
            _finalAtUs = nowUs + CAPTURE_FINAL_AFTER_US;
        }
        _lost = _rx.rxLostCount();
        emitSummary(); // first: a summary must not starve behind frame lines
        CanFrame frame;
        for (uint8_t n = 0; n < CAPTURE_MAX_FRAMES_PER_STEP; n++) {
            if (!_rx.receive(frame)) {
                break;
            }
            _frames++;
            record(frame, nowUs);
            emitFrame(frame, nowUs);
        }
        if (!_summaryActive) {
            if (!_finalStarted && nowUs >= _finalAtUs) {
                _finalStarted = true;
                beginSummary(true, nowUs);
            } else if (nowUs >= _nextSummaryUs) {
                beginSummary(false, nowUs);
            }
            emitSummary();
        }
    }

    uint32_t framesSeen() const { return _frames; }
    uint32_t droppedLines() const { return _droppedLines; }
    uint32_t overflowFrames() const { return _overflowFrames; }
    uint32_t lostFrames() const { return _lost; }
    uint8_t idCount() const { return _used; }
    const CanIdStats& idStats(uint8_t index) const { return _table[index]; }
    bool summaryActive() const { return _summaryActive; }
    bool finalDone() const { return _finalStarted && !_summaryActive; }

private:
    ICanRx& _rx;
    ICaptureSink& _sink;
    bool _started = false;
    uint32_t _frames = 0;
    uint32_t _droppedLines = 0;
    uint32_t _overflowFrames = 0;
    uint32_t _lost = 0;

    CanIdStats _table[CAPTURE_TABLE_SIZE];
    uint8_t _used = 0;

    uint64_t _nextSummaryUs = 0;
    uint64_t _finalAtUs = 0;
    bool _finalStarted = false;
    bool _summaryActive = false;
    bool _summaryFinal = false;
    uint8_t _summaryRow = 0;
    uint64_t _summaryTimeUs = 0;

    // Fixed-size line builder: never writes past its buffer (cap sized for the longest line).
    class Line {
    public:
        Line(char* buf, size_t cap) : _buf(buf), _cap(cap) {}
        void put(char c) {
            if (_len < _cap) {
                _buf[_len++] = c;
            }
        }
        void put(const char* s) {
            while (*s != '\0') {
                put(*s++);
            }
        }
        void dec(uint64_t v) {
            char tmp[20];
            uint8_t n = 0;
            do {
                tmp[n++] = (char)('0' + (uint8_t)(v % 10u));
                v /= 10u;
            } while (v != 0u);
            while (n > 0) {
                put(tmp[--n]);
            }
        }
        void hex(uint32_t v, uint8_t digits) {
            for (uint8_t i = digits; i > 0; i--) {
                put("0123456789ABCDEF"[(v >> (4u * (i - 1u))) & 0xFu]);
            }
        }
        // Microseconds as milliseconds with three decimals; "-" for CAPTURE_NO_PERIOD.
        void ms(uint32_t us) {
            if (us == CAPTURE_NO_PERIOD) {
                put('-');
                return;
            }
            dec(us / 1000u);
            put('.');
            const uint32_t frac = us % 1000u;
            put((char)('0' + frac / 100u));
            put((char)('0' + (frac / 10u) % 10u));
            put((char)('0' + frac % 10u));
        }
        size_t length() const { return _len; }
        const char* data() const { return _buf; }

    private:
        char* _buf;
        size_t _cap;
        size_t _len = 0;
    };

    static bool isWatched(const CanFrame& f) {
        for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
            const vehicle_cl250_watch_id_t& w = vehicle_cl250_functional_watch[i];
            // A 29-bit watch entry (0x18DB<TA><SA>) matches from any source address, like the
            // tester's own foreign-tester check; an 11-bit entry matches exactly.
            const uint32_t mask = w.extended ? ~isotp::kNormalFixedSourceAddressMask : 0xFFFFFFFFu;
            if (f.extended == w.extended && (f.id & mask) == (w.id & mask)) {
                return true;
            }
        }
        return false;
    }

    void record(const CanFrame& f, uint64_t nowUs) {
        CanIdStats* e = nullptr;
        for (uint8_t i = 0; i < _used; i++) {
            if (_table[i].id == f.id && _table[i].extended == f.extended) {
                e = &_table[i];
                break;
            }
        }
        if (e == nullptr) {
            if (_used >= CAPTURE_TABLE_SIZE) {
                if (_overflowFrames != 0xFFFFFFFFu) {
                    _overflowFrames++;
                }
                return;
            }
            e = &_table[_used++];
            e->id = f.id;
            e->extended = f.extended;
            e->watched = isWatched(f);
        }
        if (e->count != 0) {
            const uint64_t gap = nowUs - e->lastUs;
            const uint32_t gap32 = gap >= CAPTURE_NO_PERIOD ? (CAPTURE_NO_PERIOD - 1u) : (uint32_t)gap;
            if (gap32 < e->minPeriodUs) {
                e->minPeriodUs = gap32;
            }
            if (gap32 > e->maxPeriodUs) {
                e->maxPeriodUs = gap32;
            }
        }
        e->lastUs = nowUs;
        if (e->count != 0xFFFFFFFFu) {
            e->count++;
        }
    }

    void emitFrame(const CanFrame& f, uint64_t nowUs) {
        char buf[64];
        Line line(buf, sizeof(buf));
        const uint8_t dlc = f.dlc > 8 ? 8 : f.dlc;
        line.put("F,");
        line.dec(nowUs);
        line.put(',');
        line.hex(f.id, f.extended ? 8 : 3);
        line.put(f.extended ? ",X," : ",S,");
        line.dec(dlc);
        line.put(',');
        for (uint8_t i = 0; i < dlc; i++) {
            line.hex(f.data[i], 2);
        }
        line.put('\n');
        if (_sink.availableForWrite() < line.length()) {
            if (_droppedLines != 0xFFFFFFFFu) {
                _droppedLines++;
            }
            return;
        }
        _sink.write(line.data(), line.length());
    }

    void beginSummary(bool final, uint64_t nowUs) {
        _summaryActive = true;
        _summaryFinal = final;
        _summaryRow = 0;
        _summaryTimeUs = nowUs;
        while (_nextSummaryUs <= nowUs) {
            _nextSummaryUs += CAPTURE_SUMMARY_PERIOD_US; // no catch-up burst after a stall
        }
    }

    // Writes as many pending summary lines as the sink has room for; the rest waits for the
    // next pass.
    void emitSummary() {
        while (_summaryActive) {
            char buf[128];
            Line line(buf, sizeof(buf));
            line.put(_summaryFinal ? "FINAL," : "S,");
            const bool row = _summaryRow < _used;
            if (row) {
                const CanIdStats& e = _table[_summaryRow];
                line.dec(_summaryTimeUs);
                line.put(',');
                line.hex(e.id, e.extended ? 8 : 3);
                line.put(e.extended ? ",X," : ",S,");
                line.dec(e.count);
                line.put(',');
                line.ms(e.minPeriodUs);
                line.put(',');
                line.ms(e.count > 1 ? e.maxPeriodUs : CAPTURE_NO_PERIOD);
                line.put(e.watched ? ",W" : ",-");
            } else {
                line.put("END,");
                line.dec(_summaryTimeUs);
                line.put(",frames=");
                line.dec(_frames);
                line.put(",ids=");
                line.dec(_used);
                line.put(",overflow=");
                line.dec(_overflowFrames);
                line.put(",lost=");
                line.dec(_lost);
                line.put(",dropped=");
                line.dec(_droppedLines);
            }
            line.put('\n');
            if (_sink.availableForWrite() < line.length()) {
                return;
            }
            _sink.write(line.data(), line.length());
            if (row) {
                _summaryRow++;
            } else {
                _summaryActive = false;
            }
        }
    }
};

#endif // CAN_CAPTURE_CORE_H
