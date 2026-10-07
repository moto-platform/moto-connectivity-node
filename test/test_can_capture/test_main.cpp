#include <unity.h>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

// D-058 item 4 -- native tests of the CAN capture probe: the pure core against a MockCanRx
// and a recording sink, plus CanCaptureModule (compiled here against the Arduino stub).
// [env:native] builds no src/, so the implementation files are included directly.
#include "../../src/CanCaptureModule.h"
#include "../../src/CanCaptureModule.cpp"
#include "../../external/moto-vehicle-defs/gen/c/conn/vehicle_cl250.c"

// Compile-level guarantee: the capture code only ever sees the receive-only interface. If
// anything in CanCaptureModule.h / CanCaptureCore.h pulled in ICanBus (which can transmit),
// its include guard would be defined here.
#ifdef I_CAN_BUS_H
#error "CanCaptureModule must not include ICanBus.h: the capture build has no transmit path."
#endif

#include "../mocks/MockCanRx.h"

template <typename T>
struct HasTransmit {
    template <typename U>
    static auto probe(int) -> decltype(std::declval<U&>().transmit(std::declval<const CanFrame&>()), std::true_type());
    template <typename>
    static std::false_type probe(...);
    static const bool value = decltype(probe<T>(0))::value;
};
struct SendCapable { // positive control for the trait itself
    bool transmit(const CanFrame&) { return true; }
};
static_assert(HasTransmit<SendCapable>::value, "the HasTransmit probe must detect a transmit method");
static_assert(!HasTransmit<ICanRx>::value, "ICanRx must have no transmit method");
static_assert(!HasTransmit<MockCanRx>::value, "the capture test double must have no transmit method");
static_assert(std::is_constructible<CanCaptureModule, ICanRx&>::value, "CanCaptureModule is built from an ICanRx");
static_assert(!std::is_constructible<CanCaptureModule, int>::value, "CanCaptureModule takes nothing but an ICanRx");

namespace {

class RecordingSink : public ICaptureSink {
public:
    std::string text;
    size_t room = 1 << 20;
    size_t availableForWrite() override { return room; }
    void write(const char* data, size_t len) override { text.append(data, len); }

    std::vector<std::string> lines() const {
        std::vector<std::string> out;
        size_t start = 0;
        for (size_t i = 0; i < text.size(); i++) {
            if (text[i] == '\n') {
                out.push_back(text.substr(start, i - start));
                start = i + 1;
            }
        }
        return out;
    }
};

const uint64_t kSec = 1000ull * 1000ull;

size_t countPrefix(const std::vector<std::string>& lines, const char* prefix) {
    size_t n = 0;
    for (const std::string& l : lines) {
        if (l.compare(0, std::string(prefix).size(), prefix) == 0) {
            n++;
        }
    }
    return n;
}

} // namespace

void setUp(void) {
    test_setMicros(0);
    Serial.written.clear();
    Serial.room = 1 << 20;
}
void tearDown(void) {}

void test_frame_lines_follow_the_documented_format(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    const uint8_t ext[] = {0x04, 0x62, 0xF4, 0x0C, 0x12, 0x34, 0xAA, 0xAA};
    const uint8_t std3[] = {0x01, 0x02, 0x03};
    rx.inject(0x18DAF110u, true, 8, ext);
    core.step(1234567);
    rx.inject(0x7E8u, false, 3, std3);
    core.step(1234600);
    rx.inject(0x123u, false, 0);
    core.step(1234700);
    std::vector<std::string> lines = sink.lines();
    TEST_ASSERT_EQUAL(3, lines.size());
    TEST_ASSERT_EQUAL_STRING("F,1234567,18DAF110,X,8,0462F40C1234AAAA", lines[0].c_str());
    TEST_ASSERT_EQUAL_STRING("F,1234600,7E8,S,3,010203", lines[1].c_str());
    TEST_ASSERT_EQUAL_STRING("F,1234700,123,S,0,", lines[2].c_str());
    TEST_ASSERT_EQUAL_UINT32(3, core.framesSeen());
    TEST_ASSERT_EQUAL_UINT32(0, core.droppedLines());
}

void test_same_id_in_standard_and_extended_format_are_separate_entries(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    rx.inject(0x123u, false, 0);
    rx.inject(0x123u, true, 0);
    core.step(10);
    TEST_ASSERT_EQUAL_UINT8(2, core.idCount());
}

void test_id_table_counts_frames_and_min_max_period(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    // 0x100 at 1000, 11000 and 31000 us: gaps of 10.000 and 20.000 ms. 0x200 once.
    rx.inject(0x100u, false, 0);
    core.step(1000);
    rx.inject(0x100u, false, 0);
    rx.inject(0x200u, false, 0);
    core.step(11000);
    rx.inject(0x100u, false, 0);
    core.step(31000);
    TEST_ASSERT_EQUAL_UINT8(2, core.idCount());
    const CanIdStats& a = core.idStats(0);
    TEST_ASSERT_EQUAL_HEX32(0x100, a.id);
    TEST_ASSERT_EQUAL_UINT32(3, a.count);
    TEST_ASSERT_EQUAL_UINT32(10000, a.minPeriodUs);
    TEST_ASSERT_EQUAL_UINT32(20000, a.maxPeriodUs);
    const CanIdStats& b = core.idStats(1);
    TEST_ASSERT_EQUAL_UINT32(1, b.count);
    TEST_ASSERT_EQUAL_UINT32(CAPTURE_NO_PERIOD, b.minPeriodUs);
}

void test_watch_flag_marks_the_generated_functional_request_ids(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    // Every generated watch ID is flagged, as is the same 29-bit target from another source
    // address (the tester's own foreign-tester rule); a plain ID is not.
    for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        rx.inject(vehicle_cl250_functional_watch[i].id, vehicle_cl250_functional_watch[i].extended, 0);
    }
    uint8_t ext = 0;
    for (uint8_t i = 0; i < VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT; i++) {
        if (vehicle_cl250_functional_watch[i].extended) {
            rx.inject(vehicle_cl250_functional_watch[i].id ^ 0x5Au, true, 0); // other SA
            ext++;
        }
    }
    rx.inject(0x1A0u, false, 0);
    core.step(100);
    TEST_ASSERT_EQUAL_UINT8(VEHICLE_CL250_FUNCTIONAL_WATCH_COUNT + ext + 1, core.idCount());
    for (uint8_t i = 0; i < core.idCount() - 1; i++) {
        TEST_ASSERT_TRUE(core.idStats(i).watched);
    }
    TEST_ASSERT_FALSE(core.idStats(core.idCount() - 1).watched);
}

void test_table_overflow_is_counted_not_tracked(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    // 70 distinct IDs over several passes (the drain is bounded per pass): the first 64 are
    // tracked, frames of the other 6 only count as overflow. They still stream as F lines.
    for (uint32_t id = 0x100; id < 0x100 + 70; id++) {
        rx.inject(id, false, 0);
    }
    uint64_t t = 1000;
    while (rx.pending() > 0) {
        core.step(t);
        t += 1000;
    }
    TEST_ASSERT_EQUAL_UINT8(CAPTURE_TABLE_SIZE, core.idCount());
    TEST_ASSERT_EQUAL_UINT32(6, core.overflowFrames());
    TEST_ASSERT_EQUAL_UINT32(70, core.framesSeen());
    TEST_ASSERT_EQUAL(70, countPrefix(sink.lines(), "F,"));
    // A frame of a tracked ID after the overflow is still counted normally.
    rx.inject(0x100, false, 0);
    core.step(t);
    TEST_ASSERT_EQUAL_UINT32(2, core.idStats(0).count);
    TEST_ASSERT_EQUAL_UINT32(6, core.overflowFrames());
}

void test_drain_per_pass_is_bounded(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    for (int i = 0; i < 100; i++) {
        rx.inject(0x100, false, 0);
    }
    core.step(1);
    TEST_ASSERT_EQUAL_UINT32(CAPTURE_MAX_FRAMES_PER_STEP, core.framesSeen());
    TEST_ASSERT_EQUAL(100 - CAPTURE_MAX_FRAMES_PER_STEP, rx.pending());
}

void test_lines_are_dropped_and_counted_when_the_sink_is_full_and_never_blocked(void) {
    MockCanRx rx;
    RecordingSink sink;
    sink.room = 5; // smaller than any frame line
    CanCaptureCore core(rx, sink);
    for (int i = 0; i < 3; i++) {
        rx.inject(0x100, false, 0);
    }
    core.step(100);
    TEST_ASSERT_EQUAL_UINT32(3, core.droppedLines());
    TEST_ASSERT_EQUAL_UINT32(3, core.framesSeen());
    TEST_ASSERT_EQUAL_UINT32(3, core.idStats(0).count); // statistics do not depend on the output
    TEST_ASSERT_EQUAL(0, sink.text.size());
    // Room again: the next frame is written, the counter keeps its value.
    sink.room = 1000;
    rx.inject(0x100, false, 0);
    core.step(200);
    TEST_ASSERT_EQUAL_UINT32(3, core.droppedLines());
    TEST_ASSERT_EQUAL(1, countPrefix(sink.lines(), "F,"));
}

void test_summary_every_10_seconds_with_rows_and_end_line(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    core.step(0);
    rx.inject(0x100u, false, 0);
    core.step(1000);
    rx.inject(0x100u, false, 0);
    rx.inject(0x7DFu, false, 0); // watched
    core.step(11000);
    sink.text.clear();
    core.step(10 * kSec - 1);
    TEST_ASSERT_EQUAL(0, countPrefix(sink.lines(), "S,"));
    core.step(10 * kSec);
    std::vector<std::string> lines = sink.lines();
    TEST_ASSERT_EQUAL(3, lines.size());
    TEST_ASSERT_EQUAL_STRING("S,10000000,100,S,2,10.000,10.000,-", lines[0].c_str());
    TEST_ASSERT_EQUAL_STRING("S,10000000,7DF,S,1,-,-,W", lines[1].c_str());
    TEST_ASSERT_EQUAL_STRING("S,END,10000000,frames=3,ids=2,overflow=0,lost=0,dropped=0", lines[2].c_str());
    // And again 10 s later.
    sink.text.clear();
    core.step(20 * kSec);
    TEST_ASSERT_EQUAL(3, sink.lines().size());
}

void test_final_summary_after_300_seconds_and_capture_keeps_going(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    core.step(0);
    rx.inject(0x100u, false, 0);
    core.step(1000);
    sink.text.clear();
    core.step(300 * kSec);
    std::vector<std::string> lines = sink.lines();
    TEST_ASSERT_EQUAL(2, lines.size());
    TEST_ASSERT_EQUAL(0, countPrefix(lines, "S,")); // the FINAL replaces the periodic summary
    TEST_ASSERT_EQUAL_STRING("FINAL,300000000,100,S,1,-,-,-", lines[0].c_str());
    TEST_ASSERT_EQUAL_STRING("FINAL,END,300000000,frames=1,ids=1,overflow=0,lost=0,dropped=0", lines[1].c_str());
    TEST_ASSERT_TRUE(core.finalDone());
    // Frames are still captured and the periodic summaries go on, but only one FINAL.
    sink.text.clear();
    rx.inject(0x100u, false, 0);
    core.step(305 * kSec);
    rx.inject(0x100u, false, 0);
    core.step(310 * kSec);
    lines = sink.lines();
    TEST_ASSERT_EQUAL_UINT32(3, core.framesSeen());
    TEST_ASSERT_EQUAL(2, countPrefix(lines, "F,"));
    TEST_ASSERT_EQUAL(2, countPrefix(lines, "S,"));
    TEST_ASSERT_EQUAL(0, countPrefix(lines, "FINAL,"));
}

void test_summary_waits_for_room_instead_of_blocking_and_is_not_dropped(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    core.step(0);
    rx.inject(0x100u, false, 0);
    core.step(1000);
    sink.text.clear();
    sink.room = 10; // a summary line does not fit
    core.step(10 * kSec);
    TEST_ASSERT_TRUE(core.summaryActive());
    TEST_ASSERT_EQUAL(0, sink.text.size());
    TEST_ASSERT_EQUAL_UINT32(0, core.droppedLines());
    sink.room = 1000;
    core.step(10 * kSec + 100);
    TEST_ASSERT_FALSE(core.summaryActive());
    TEST_ASSERT_EQUAL(2, countPrefix(sink.lines(), "S,"));
}

void test_summary_reports_driver_lost_frames_and_dropped_lines(void) {
    MockCanRx rx;
    RecordingSink sink;
    CanCaptureCore core(rx, sink);
    core.step(0);
    sink.room = 3;
    rx.inject(0x100u, false, 0);
    core.step(1000); // dropped line
    sink.room = 1000;
    rx.lost = 12;
    core.step(10 * kSec);
    std::vector<std::string> lines = sink.lines();
    TEST_ASSERT_EQUAL_STRING("S,END,10000000,frames=1,ids=1,overflow=0,lost=12,dropped=1", lines.back().c_str());
    TEST_ASSERT_EQUAL_UINT32(12, core.lostFrames());
}

void test_module_streams_through_serial_with_a_64_bit_clock(void) {
    MockCanRx rx;
    CanCaptureModule module(rx);
    SystemState state;
    TEST_ASSERT_TRUE(module.begin());
    TEST_ASSERT_EQUAL(1, rx.beginCalls);
    TEST_ASSERT_TRUE(module.isHealthy());
    Serial.written.clear();
    const uint8_t data[] = {0xAB};
    // micros() wraps at 2^32: start 256 us before it, continue 256 us after it.
    test_setMicros(0xFFFFFF00ul);
    module.update(state);
    rx.inject(0x321u, false, 1, data);
    test_setMicros(0x100ul);
    module.update(state);
    // 0xFFFFFF00 us before the wrap plus 0x200 across it: the 64-bit clock carries on past 2^32.
    TEST_ASSERT_EQUAL_STRING("F,4294967552,321,S,1,AB\n", Serial.written.c_str());
    TEST_ASSERT_EQUAL_UINT32(1, module.core().framesSeen());
}

void test_module_drops_lines_when_serial_has_no_room(void) {
    MockCanRx rx;
    CanCaptureModule module(rx);
    SystemState state;
    TEST_ASSERT_TRUE(module.begin());
    Serial.written.clear();
    Serial.room = 4;
    rx.inject(0x321u, false, 0);
    test_setMicros(1000);
    module.update(state);
    TEST_ASSERT_EQUAL(0, Serial.written.size());
    TEST_ASSERT_EQUAL_UINT32(1, module.core().droppedLines());
}

void test_module_unhealthy_when_the_driver_does_not_start(void) {
    MockCanRx rx;
    rx.failBegin = true;
    CanCaptureModule module(rx);
    TEST_ASSERT_FALSE(module.begin());
    TEST_ASSERT_FALSE(module.isHealthy());
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_frame_lines_follow_the_documented_format);
    RUN_TEST(test_same_id_in_standard_and_extended_format_are_separate_entries);
    RUN_TEST(test_id_table_counts_frames_and_min_max_period);
    RUN_TEST(test_watch_flag_marks_the_generated_functional_request_ids);
    RUN_TEST(test_table_overflow_is_counted_not_tracked);
    RUN_TEST(test_drain_per_pass_is_bounded);
    RUN_TEST(test_lines_are_dropped_and_counted_when_the_sink_is_full_and_never_blocked);
    RUN_TEST(test_summary_every_10_seconds_with_rows_and_end_line);
    RUN_TEST(test_final_summary_after_300_seconds_and_capture_keeps_going);
    RUN_TEST(test_summary_waits_for_room_instead_of_blocking_and_is_not_dropped);
    RUN_TEST(test_summary_reports_driver_lost_frames_and_dropped_lines);
    RUN_TEST(test_module_streams_through_serial_with_a_64_bit_clock);
    RUN_TEST(test_module_drops_lines_when_serial_has_no_room);
    RUN_TEST(test_module_unhealthy_when_the_driver_does_not_start);
    return UNITY_END();
}
