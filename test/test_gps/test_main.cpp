#include <unity.h>
#include <atomic>
#include <string.h>
#include <thread>
#include <vector>

// D-060 item 2 -- native tests of the GPS path's pure parts: the byte-wise UBX parser, the
// UBX-CFG frame builder, the seqlock between the GPS task and the Arduino loop, and the
// BLE GPS block packer (D-060 item 3, ble_schema.json `gpsBlock`) and the notify decision.
#include "../../src/GpsBlockPacket.h"
#include "../../src/GpsCore.h"
#include "../../src/GpsNotifyTracker.h"
#include "../../src/Seqlock.h"
#include "../../src/UbxConfig.h"
#include "../../src/UbxParser.h"

namespace {

// Recognizable position bytes: none of them may appear in what the parser hands out.
const int32_t kLon = 0x1A2B3C4D;
const int32_t kLat = 0x0F1E2D3C;
const int32_t kHeight = 0x05060708;

void putU32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8u);
    p[2] = (uint8_t)(v >> 16u);
    p[3] = (uint8_t)(v >> 24u);
}

void putI32(uint8_t* p, int32_t v) {
    uint32_t u;
    memcpy(&u, &v, sizeof(u));
    putU32(p, u);
}

// A NAV-PVT frame (100 bytes) with the given motion values and the position sentinels.
size_t navPvt(uint8_t* out, int32_t gSpeed, int32_t headMot, uint8_t fixType = 3u,
              uint8_t flags = 0x01u, uint8_t numSv = 11u) {
    uint8_t p[92] = {};
    putU32(&p[0], 123456000u); // iTOW
    p[20] = fixType;
    p[21] = flags;
    p[23] = numSv;
    putI32(&p[24], kLon);
    putI32(&p[28], kLat);
    putI32(&p[32], kHeight);
    putI32(&p[36], kHeight);
    putU32(&p[40], 2500u);  // hAcc
    putU32(&p[44], 3500u);  // vAcc
    putI32(&p[48], 1000);   // velN
    putI32(&p[52], -2000);  // velE
    putI32(&p[60], gSpeed);
    putI32(&p[64], headMot);
    putU32(&p[68], 150u);   // sAcc
    putU32(&p[72], 80000u); // headAcc
    return ubx::buildFrame(ubx::kClassNav, ubx::kIdNavPvt, p, sizeof(p), out, 120u);
}

UbxParser::Result feedAll(UbxParser& parser, const uint8_t* data, size_t n, size_t* pvts = nullptr) {
    UbxParser::Result last = UbxParser::NONE;
    for (size_t i = 0; i < n; i++) {
        UbxParser::Result r = parser.feed(data[i]);
        if (r != UbxParser::NONE) {
            last = r;
        }
        if ((pvts != nullptr) && (r == UbxParser::NAV_PVT)) {
            (*pvts)++;
        }
    }
    return last;
}

bool containsBytes(const void* hay, size_t n, int32_t needle) {
    const uint8_t* h = static_cast<const uint8_t*>(hay);
    uint8_t b[4];
    memcpy(b, &needle, sizeof(b));
    for (size_t i = 0; i + 4u <= n; i++) {
        if (memcmp(&h[i], b, 4u) == 0) {
            return true;
        }
    }
    return false;
}


} // namespace

void setUp() {}
void tearDown() {}

void test_cfg_frames_match_known_good_bytes() {
    // Widely used reference frames for the NEO-M8N (u-blox M8 protocol spec examples).
    const uint8_t rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
    const uint8_t msg[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
    uint8_t out[ubx::kMaxCfgFrame];
    TEST_ASSERT_EQUAL_UINT32(sizeof(rate), ubx::buildCfgRate(out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(rate, out, sizeof(rate));
    TEST_ASSERT_EQUAL_UINT32(sizeof(msg), ubx::buildCfgMsgNavPvt(out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT8_ARRAY(msg, out, sizeof(msg));
}

void test_cfg_prt_is_ubx_only_at_the_requested_baud() {
    uint8_t out[ubx::kMaxCfgFrame];
    TEST_ASSERT_EQUAL_UINT32(28u, ubx::buildCfgPrtUart1(ubx::kGpsBaud, out, sizeof(out)));
    TEST_ASSERT_EQUAL_HEX8(0x06, out[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, out[3]);
    TEST_ASSERT_EQUAL_HEX8(20, out[4]);
    TEST_ASSERT_EQUAL_HEX8(0x01, out[6]);                                         // UART1
    TEST_ASSERT_EQUAL_UINT32(38400u, ubx::readU32(&out[6 + 8]));                  // baud
    TEST_ASSERT_EQUAL_HEX8(0x01, out[6 + 12]);                                    // in: UBX
    TEST_ASSERT_EQUAL_HEX8(0x01, out[6 + 14]);                                    // out: UBX only
    TEST_ASSERT_EQUAL_HEX8(0x00, out[6 + 15]);
    // The frame parses back as a valid UBX message (checksum consistent).
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::OTHER, feedAll(parser, out, 28u));
    TEST_ASSERT_EQUAL_UINT32(0u, parser.checksumErrors());
}

void test_cfg_builder_refuses_a_short_buffer() {
    uint8_t out[10];
    TEST_ASSERT_EQUAL_UINT32(0u, ubx::buildCfgRate(out, sizeof(out)));
    TEST_ASSERT_EQUAL_UINT32(0u, ubx::buildFrame(0x06, 0x08, nullptr, 6u, out, sizeof(out)));
}

void test_nav_pvt_decodes_motion_fields() {
    uint8_t frame[120];
    size_t n = navPvt(frame, 13889, 27012345, 3u, 0x01u, 14u);
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::NAV_PVT, feedAll(parser, frame, n));
    const GpsFix& fix = parser.lastFix();
    TEST_ASSERT_EQUAL_UINT32(123456000u, fix.iTowMs);
    TEST_ASSERT_EQUAL_INT32(13889, fix.groundSpeedMmps);
    TEST_ASSERT_EQUAL_INT32(27012345, fix.headingMotionE5);
    TEST_ASSERT_EQUAL_UINT32(150u, fix.speedAccMmps);
    TEST_ASSERT_EQUAL_UINT32(80000u, fix.headingAccE5);
    TEST_ASSERT_EQUAL_UINT8(3u, fix.fixType);
    TEST_ASSERT_EQUAL_UINT8(1u, fix.gnssFixOk);
    TEST_ASSERT_EQUAL_UINT8(14u, fix.numSv);
    TEST_ASSERT_EQUAL_UINT32(1u, parser.navPvtCount());
}

void test_negative_speed_and_flags_bits_other_than_fix_ok() {
    uint8_t frame[120];
    size_t n = navPvt(frame, -5, 0, 2u, 0xFEu); // gnssFixOK clear, every other bit set
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::NAV_PVT, feedAll(parser, frame, n));
    TEST_ASSERT_EQUAL_INT32(-5, parser.lastFix().groundSpeedMmps);
    TEST_ASSERT_EQUAL_UINT8(0u, parser.lastFix().gnssFixOk);
}

void test_no_position_leaves_the_parser() {
    uint8_t frame[120];
    size_t n = navPvt(frame, 1000, 2000);
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::NAV_PVT, feedAll(parser, frame, n));
    GpsFix fix = parser.lastFix();
    TEST_ASSERT_FALSE(containsBytes(&fix, sizeof(fix), kLat));
    TEST_ASSERT_FALSE(containsBytes(&fix, sizeof(fix), kLon));
    TEST_ASSERT_FALSE(containsBytes(&fix, sizeof(fix), kHeight));
    TEST_ASSERT_TRUE(parser.payloadIsClear()); // the payload buffer is zeroed after use
    TEST_ASSERT_FALSE(containsBytes(&parser, sizeof(parser), kLat));
    TEST_ASSERT_FALSE(containsBytes(&parser, sizeof(parser), kLon));
}

void test_bytes_split_across_feeds_and_back_to_back_frames() {
    uint8_t buf[400];
    size_t n = 0;
    n += navPvt(&buf[n], 1, 10);
    n += navPvt(&buf[n], 2, 20);
    n += navPvt(&buf[n], 3, 30);
    UbxParser parser;
    size_t pvts = 0;
    // Fed in uneven chunks, as UART reads deliver them.
    size_t pos = 0;
    const size_t chunks[] = {1, 7, 50, 3, 99, 140, 100};
    for (size_t c = 0; c < sizeof(chunks) / sizeof(chunks[0]) && pos < n; c++) {
        size_t len = (chunks[c] < n - pos) ? chunks[c] : n - pos;
        feedAll(parser, &buf[pos], len, &pvts);
        pos += len;
    }
    TEST_ASSERT_EQUAL_UINT32(n, pos);
    TEST_ASSERT_EQUAL_UINT32(3u, pvts);
    TEST_ASSERT_EQUAL_INT32(3, parser.lastFix().groundSpeedMmps);
}

void test_bad_checksum_is_counted_and_the_next_frame_parses() {
    uint8_t buf[300];
    size_t n = navPvt(buf, 111, 222);
    buf[n - 1] ^= 0xFFu; // corrupt CK_B
    size_t n2 = navPvt(&buf[n], 333, 444);
    UbxParser parser;
    size_t pvts = 0;
    feedAll(parser, buf, n + n2, &pvts);
    TEST_ASSERT_EQUAL_UINT32(1u, parser.checksumErrors());
    TEST_ASSERT_EQUAL_UINT32(1u, pvts);
    TEST_ASSERT_EQUAL_INT32(333, parser.lastFix().groundSpeedMmps);
}

void test_corrupt_payload_byte_fails_the_checksum() {
    uint8_t buf[120];
    size_t n = navPvt(buf, 111, 222);
    buf[6 + 60] ^= 0x01u; // flip one bit of gSpeed
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::ERROR, feedAll(parser, buf, n));
    TEST_ASSERT_EQUAL_UINT32(0u, parser.navPvtCount());
    TEST_ASSERT_EQUAL_INT32(0, parser.lastFix().groundSpeedMmps);
}

void test_oversized_length_is_counted_and_skipped() {
    uint8_t buf[300];
    const uint8_t head[] = {0xB5, 0x62, 0x01, 0x35, 0x00, 0x03}; // 768 bytes: a false sync
    memcpy(buf, head, sizeof(head));
    size_t n = sizeof(head);
    memset(&buf[n], 0x00, 40);
    n += 40;
    n += navPvt(&buf[n], 77, 88);
    UbxParser parser;
    size_t pvts = 0;
    feedAll(parser, buf, n, &pvts);
    TEST_ASSERT_EQUAL_UINT32(1u, parser.lengthErrors());
    TEST_ASSERT_EQUAL_UINT32(1u, pvts);
    TEST_ASSERT_EQUAL_INT32(77, parser.lastFix().groundSpeedMmps);
}

void test_nav_pvt_with_the_wrong_length_is_refused() {
    uint8_t p[84] = {}; // the older 84-byte layout
    uint8_t buf[100];
    size_t n = ubx::buildFrame(ubx::kClassNav, ubx::kIdNavPvt, p, sizeof(p), buf, sizeof(buf));
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::ERROR, feedAll(parser, buf, n));
    TEST_ASSERT_EQUAL_UINT32(1u, parser.lengthErrors());
    TEST_ASSERT_EQUAL_UINT32(0u, parser.navPvtCount());
}

void test_resync_after_garbage_nmea_and_a_lone_sync_byte() {
    const char nmea[] = "$GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
    uint8_t buf[300];
    size_t n = 0;
    memcpy(buf, nmea, sizeof(nmea) - 1u);
    n += sizeof(nmea) - 1u;
    buf[n++] = 0xB5;  // lone first sync byte
    buf[n++] = 0x00;
    buf[n++] = 0xB5;  // repeated first sync byte right before a real frame
    n += navPvt(&buf[n], 55, 66);
    UbxParser parser;
    size_t pvts = 0;
    feedAll(parser, buf, n, &pvts);
    TEST_ASSERT_EQUAL_UINT32(1u, pvts);
    TEST_ASSERT_EQUAL_UINT32(0u, parser.checksumErrors());
    TEST_ASSERT_EQUAL_INT32(55, parser.lastFix().groundSpeedMmps);
}

void test_header_lookalike_in_garbage_costs_at_most_the_next_frame() {
    // A false "B5 62" with a short length swallows the start of the real frame after it;
    // the frame after that one parses. Documented in UbxParser.h.
    uint8_t buf[400];
    size_t n = 0;
    const uint8_t fake[] = {0x13, 0xB5, 0x62, 0x05, 0x01, 0x02, 0x00};
    memcpy(buf, fake, sizeof(fake));
    n += sizeof(fake);
    n += navPvt(&buf[n], 1, 1);
    n += navPvt(&buf[n], 2, 2);
    UbxParser parser;
    size_t pvts = 0;
    feedAll(parser, buf, n, &pvts);
    TEST_ASSERT_TRUE(pvts >= 1u);
    TEST_ASSERT_EQUAL_INT32(2, parser.lastFix().groundSpeedMmps);
}

void test_other_messages_are_ignored_but_valid() {
    const uint8_t ackPayload[] = {0x06, 0x08}; // ACK-ACK for CFG-RATE
    uint8_t buf[20];
    size_t n = ubx::buildFrame(0x05, 0x01, ackPayload, sizeof(ackPayload), buf, sizeof(buf));
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::OTHER, feedAll(parser, buf, n));
    TEST_ASSERT_EQUAL_UINT32(1u, parser.messagesOk());
    TEST_ASSERT_EQUAL_UINT32(0u, parser.navPvtCount());
    TEST_ASSERT_EQUAL_HEX8(0x05, parser.lastClass());
    TEST_ASSERT_EQUAL_HEX8(0x01, parser.lastId());
    TEST_ASSERT_TRUE(parser.payloadIsClear()); // only NAV-PVT is ever stored
}

void test_zero_length_message() {
    uint8_t buf[10];
    size_t n = ubx::buildFrame(0x01, 0x07, nullptr, 0u, buf, sizeof(buf)); // poll request form
    UbxParser parser;
    TEST_ASSERT_EQUAL(UbxParser::ERROR, feedAll(parser, buf, n)); // NAV-PVT of length 0
    n = ubx::buildFrame(0x0A, 0x04, nullptr, 0u, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(UbxParser::OTHER, feedAll(parser, buf, n));
}


// Scripted receiver behind IGpsUart: it hears the node only when both sides use the same
// baud, answers every complete UBX frame with ACK-ACK and applies CFG-PRT's baud. At
// another baud the node reads line noise. Every read advances the clock.
class MockGpsUart : public IGpsUart {
public:
    explicit MockGpsUart(uint32_t receiverBaud, bool present = true)
        : receiverBaud(receiverBaud), present(present) {}

    bool setBaud(uint32_t baud) override {
        nodeBaud = baud;
        bauds.push_back(baud);
        return true;
    }
    bool write(const uint8_t* data, size_t len) override {
        for (size_t i = 0; i < len; i++) {
            if (!present || nodeBaud != receiverBaud) {
                continue;
            }
            UbxParser::Result r = rxParser.feed(data[i]);
            if (r == UbxParser::OTHER) {
                uint8_t ackPayload[2] = {rxParser.lastClass(), rxParser.lastId()};
                uint8_t ack[10];
                size_t n = ubx::buildFrame(0x05, 0x01, ackPayload, 2u, ack, sizeof(ack));
                pending.insert(pending.end(), ack, ack + n);
                if (rxParser.lastClass() == 0x06 && rxParser.lastId() == 0x00) {
                    newBaud = ubx::kGpsBaud; // the test only sends kGpsBaud
                }
            }
        }
        return true;
    }
    void waitTxDone(uint32_t) override {
        if (newBaud != 0u) {
            receiverBaud = newBaud;
            newBaud = 0u;
        }
    }
    size_t read(uint8_t* out, size_t cap, uint32_t timeoutMs) override {
        clock += timeoutMs;
        if (!present) {
            return 0u;
        }
        if (nodeBaud != receiverBaud) {
            size_t n = cap < 8u ? cap : 8u; // noise
            for (size_t i = 0; i < n; i++) {
                out[i] = (uint8_t)(0x55u ^ (i * 37u));
            }
            return n;
        }
        size_t n = 0;
        while (n < cap && !pending.empty()) {
            out[n++] = pending.front();
            pending.erase(pending.begin());
        }
        return n;
    }
    bool takeOverflow() override {
        bool o = overflow;
        overflow = false;
        return o;
    }
    uint32_t nowMs() override { return clock; }

    uint32_t receiverBaud;
    bool present;
    uint32_t nodeBaud = 0u;
    uint32_t newBaud = 0u;
    uint32_t clock = 1000u;
    bool overflow = false;
    std::vector<uint8_t> pending;
    std::vector<uint32_t> bauds;
    UbxParser rxParser;
};

void test_configure_warm_receiver_already_at_38400() {
    MockGpsUart uart(38400u);
    GpsCore core;
    TEST_ASSERT_TRUE(core.configure(uart));
    TEST_ASSERT_EQUAL(GpsCore::LINK_OK, core.link());
    TEST_ASSERT_EQUAL_UINT32(1u, (uint32_t)uart.bauds.size()); // never fell back to 9600
    TEST_ASSERT_EQUAL_UINT32(38400u, uart.nodeBaud);
}

void test_configure_cold_receiver_moves_from_9600() {
    MockGpsUart uart(9600u);
    GpsCore core;
    TEST_ASSERT_TRUE(core.configure(uart));
    TEST_ASSERT_EQUAL(GpsCore::LINK_OK, core.link());
    TEST_ASSERT_EQUAL_UINT32(38400u, uart.receiverBaud);
    TEST_ASSERT_EQUAL_UINT32(38400u, uart.nodeBaud);
    const uint32_t expected[] = {38400u, 9600u, 38400u};
    TEST_ASSERT_EQUAL_UINT32(3u, (uint32_t)uart.bauds.size());
    TEST_ASSERT_EQUAL_UINT32_ARRAY(expected, uart.bauds.data(), 3);
}

void test_configure_without_receiver_reports_it_and_is_bounded() {
    MockGpsUart uart(9600u, false);
    GpsCore core;
    TEST_ASSERT_FALSE(core.configure(uart));
    TEST_ASSERT_EQUAL(GpsCore::LINK_NO_RECEIVER, core.link());
    // Two listen windows of kListenMs each, nothing more.
    TEST_ASSERT_TRUE(uart.clock - 1000u <= 2u * (GpsCore::kListenMs + GpsCore::kReadChunkMs));
}

void test_core_publishes_fix_with_node_time_and_counts_overflow() {
    MockGpsUart uart(38400u);
    uart.nodeBaud = 38400u;
    uint8_t frame[120];
    size_t n = navPvt(frame, 4321, 9000000);
    uart.pending.assign(frame, frame + n);
    uart.overflow = true;
    GpsCore core;
    GpsFix fix;
    TEST_ASSERT_FALSE(core.latest(fix));
    core.poll(uart, 10u);
    core.poll(uart, 10u);
    TEST_ASSERT_TRUE(core.latest(fix));
    TEST_ASSERT_EQUAL_INT32(4321, fix.groundSpeedMmps);
    TEST_ASSERT_TRUE(fix.rxTimeMs >= 1010u);
    TEST_ASSERT_EQUAL_UINT32(1u, core.navPvtCount());
    TEST_ASSERT_EQUAL_UINT32(1u, core.uartOverflows());
}

void test_step_detects_silence_and_reconfigures() {
    MockGpsUart uart(38400u);
    GpsCore core;
    TEST_ASSERT_TRUE(core.step(uart)); // configures
    TEST_ASSERT_EQUAL(GpsCore::LINK_OK, core.link());
    uart.receiverBaud = 9600u; // receiver power-cycled: back to its factory baud
    uint32_t start = uart.nowMs();
    uint32_t steps = 0u;
    while (core.link() == GpsCore::LINK_OK && steps < 100u) {
        core.step(uart);
        steps++;
    }
    TEST_ASSERT_EQUAL(GpsCore::LINK_PENDING, core.link());
    // Declared lost after kSilenceMs of silence, not before, and within one more read.
    TEST_ASSERT_TRUE(uart.nowMs() - start >= GpsCore::kSilenceMs);
    TEST_ASSERT_TRUE(uart.nowMs() - start <= GpsCore::kSilenceMs + GpsCore::kPollMs);
    TEST_ASSERT_EQUAL_UINT32(1u, core.linkLosses());
    TEST_ASSERT_TRUE(core.step(uart)); // finds it at 9600 and moves it again
    TEST_ASSERT_EQUAL(GpsCore::LINK_OK, core.link());
    TEST_ASSERT_EQUAL_UINT32(38400u, uart.receiverBaud);
}

void test_seqlock_read_before_write_and_round_trip() {
    Seqlock<GpsFix> lock;
    GpsFix out;
    out.numSv = 99u;
    TEST_ASSERT_FALSE(lock.read(out));
    TEST_ASSERT_EQUAL_UINT8(99u, out.numSv); // untouched
    GpsFix in;
    in.groundSpeedMmps = -42;
    in.headingAccE5 = 0xDEADBEEFu;
    in.numSv = 7u;
    lock.write(in);
    TEST_ASSERT_TRUE(lock.read(out));
    TEST_ASSERT_EQUAL_INT32(-42, out.groundSpeedMmps);
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, out.headingAccE5);
    TEST_ASSERT_EQUAL_UINT8(7u, out.numSv);
    TEST_ASSERT_EQUAL_UINT32(1u, lock.writes());
}

void test_seqlock_reader_never_sees_a_torn_copy() {
    // The writer stores fixes whose every field derives from one counter; a consistent
    // copy satisfies the relation, a torn one almost surely does not.
    Seqlock<GpsFix> lock;
    std::atomic<bool> stop(false);
    std::thread writer([&]() {
        uint32_t k = 1u;
        while (!stop.load()) {
            GpsFix f;
            f.iTowMs = k;
            f.groundSpeedMmps = (int32_t)(k * 3u);
            f.headingMotionE5 = (int32_t)(k ^ 0x5A5A5A5Au);
            f.speedAccMmps = ~k;
            f.headingAccE5 = k * 7u;
            f.numSv = (uint8_t)k;
            lock.write(f);
            k++;
        }
    });
    uint32_t good = 0u;
    for (uint32_t i = 0u; i < 200000u; i++) {
        GpsFix f;
        if (lock.read(f, 64u)) {
            uint32_t k = f.iTowMs;
            TEST_ASSERT_EQUAL_INT32((int32_t)(k * 3u), f.groundSpeedMmps);
            TEST_ASSERT_EQUAL_INT32((int32_t)(k ^ 0x5A5A5A5Au), f.headingMotionE5);
            TEST_ASSERT_EQUAL_UINT32(~k, f.speedAccMmps);
            TEST_ASSERT_EQUAL_UINT32(k * 7u, f.headingAccE5);
            TEST_ASSERT_EQUAL_UINT8((uint8_t)k, f.numSv);
            good++;
        }
    }
    stop.store(true);
    writer.join();
    TEST_ASSERT_TRUE(good > 0u);
}

// ---- BLE GPS block (D-060 item 3) ----------------------------------------------------

static GpsFix sampleFix() {
    GpsFix f;
    f.rxTimeMs = 0x01020304u;
    f.iTowMs = 0x7E7D7C7Bu;       // must not reach the block
    f.groundSpeedMmps = -2;       // 0xFFFFFFFE: the sign survives the copy
    f.headingMotionE5 = 18000000; // 180 deg = 0x0112A880
    f.speedAccMmps = 150u;
    f.headingAccE5 = 80000u;
    f.fixType = BLE_GPS_FIX_TYPE_FIX3D;
    f.numSv = 11u;
    f.gnssFixOk = 1u;
    return f;
}

void test_gps_block_matches_known_good_bytes() {
    const uint8_t expected[26] = {
        0x01, 0x2A,             // version, seq
        0x04, 0x03, 0x02, 0x01, // deviceTimeMs
        0xFE, 0xFF, 0xFF, 0xFF, // groundSpeed -2 mm/s
        0x80, 0xA8, 0x12, 0x01, // headingOfMotion 180.00000 deg
        0x96, 0x00, 0x00, 0x00, // speedAccuracy 150 mm/s
        0x80, 0x38, 0x01, 0x00, // headingAccuracy 0.80000 deg
        0x03, 0x0B,             // fixType 3D, numSv 11
        0x05, 0x00,             // flags, reserved
    };
    uint8_t out[32];
    memset(out, 0xEE, sizeof(out));
    size_t n = packGpsBlock(sampleFix(), 0x2Au, 0x05u, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(26u, n);
    TEST_ASSERT_EQUAL_UINT32(BLE_GPS_TOTAL_BYTES, n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, sizeof(expected));
    TEST_ASSERT_EQUAL_HEX8(0xEE, out[26]); // nothing written past the block
    TEST_ASSERT_FALSE(containsBytes(out, n, (int32_t)0x7E7D7C7B));
}

void test_gps_block_refuses_a_short_buffer() {
    uint8_t out[BLE_GPS_TOTAL_BYTES];
    memset(out, 0xEE, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(0u, packGpsBlock(sampleFix(), 1u, 0u, out, sizeof(out) - 1u));
    TEST_ASSERT_EQUAL_UINT32(0u, packGpsBlock(sampleFix(), 1u, 0u, nullptr, 64u));
    for (size_t i = 0; i < sizeof(out); i++) {
        TEST_ASSERT_EQUAL_HEX8(0xEE, out[i]);
    }
    TEST_ASSERT_EQUAL_UINT32(BLE_GPS_TOTAL_BYTES, packGpsBlock(sampleFix(), 1u, 0u, out, sizeof(out)));
}

void test_gps_block_flags_from_fix_and_counter_changes() {
    GpsFix fix = sampleFix();
    GpsErrorCounters prev = {7u, 3u};
    GpsErrorCounters same = {7u, 3u};
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK, gpsBlockFlags(fix, same, prev));
    fix.gnssFixOk = 0u;
    TEST_ASSERT_EQUAL_HEX8(0x00, gpsBlockFlags(fix, same, prev));

    GpsErrorCounters parse = {8u, 3u};
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_PARSE_ERROR, gpsBlockFlags(fix, parse, prev));
    GpsErrorCounters overflow = {7u, 4u};
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_UART_OVERFLOW, gpsBlockFlags(fix, overflow, prev));

    fix.gnssFixOk = 1u;
    GpsErrorCounters both = {9u, 5u};
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK | BLE_GPS_FLAG_PARSE_ERROR | BLE_GPS_FLAG_UART_OVERFLOW,
                           gpsBlockFlags(fix, both, prev));

    // The counters wrap mod 2^32; a wrapped counter is still a change.
    GpsErrorCounters nearMax = {0xFFFFFFFFu, 0xFFFFFFFFu};
    GpsErrorCounters wrapped = {0u, 0u};
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK | BLE_GPS_FLAG_PARSE_ERROR | BLE_GPS_FLAG_UART_OVERFLOW,
                           gpsBlockFlags(fix, wrapped, nearMax));
}

void test_gps_block_from_a_parsed_nav_pvt_has_no_position() {
    uint8_t frame[120];
    size_t n = navPvt(frame, 13889, -4500000, 3u, 0x01u, 9u);
    GpsCore core;
    core.feed(frame, n, 5000u);
    GpsFix fix;
    TEST_ASSERT_TRUE(core.latest(fix));
    GpsErrorCounters now = {core.checksumErrors() + core.lengthErrors(), core.uartOverflows()};
    uint8_t out[BLE_GPS_TOTAL_BYTES];
    size_t len = packGpsBlock(fix, 0u, gpsBlockFlags(fix, now, now), out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(BLE_GPS_TOTAL_BYTES, len);

    const uint8_t* s = &out[BLE_GPS_GROUND_SPEED_OFFSET];
    const uint8_t* h = &out[BLE_GPS_HEADING_OF_MOTION_OFFSET];
    const uint8_t* t = &out[BLE_GPS_DEVICE_TIME_MS_OFFSET];
    uint32_t speed = (uint32_t)s[0] | ((uint32_t)s[1] << 8u) | ((uint32_t)s[2] << 16u) | ((uint32_t)s[3] << 24u);
    uint32_t head = (uint32_t)h[0] | ((uint32_t)h[1] << 8u) | ((uint32_t)h[2] << 16u) | ((uint32_t)h[3] << 24u);
    uint32_t time = (uint32_t)t[0] | ((uint32_t)t[1] << 8u) | ((uint32_t)t[2] << 16u) | ((uint32_t)t[3] << 24u);
    TEST_ASSERT_EQUAL_INT32(13889, (int32_t)speed);
    TEST_ASSERT_EQUAL_INT32(-4500000, (int32_t)head);
    TEST_ASSERT_EQUAL_UINT32(5000u, time);
    TEST_ASSERT_EQUAL_UINT8(3u, out[BLE_GPS_FIX_TYPE_OFFSET]);
    TEST_ASSERT_EQUAL_UINT8(9u, out[BLE_GPS_NUM_SV_OFFSET]);
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK, out[BLE_GPS_FLAGS_OFFSET]);
    TEST_ASSERT_FALSE(containsBytes(out, len, kLat));
    TEST_ASSERT_FALSE(containsBytes(out, len, kLon));
    TEST_ASSERT_FALSE(containsBytes(out, len, kHeight));
}

// ---- GpsNotifyTracker: when a NAV-PVT becomes a GPS block ----------------------------------

namespace {

const uint16_t kRoomy = 182u;  // blePayloadLimit(185), the MTU moto-mobile requests
const uint16_t kAttDefault = 20u; // blePayloadLimit(23), every connection's first MTU

GpsFix trackerFix(uint8_t fixType = 3u, uint8_t gnssFixOk = 1u) {
    GpsFix fix;
    fix.rxTimeMs = 1000u;
    fix.fixType = fixType;
    fix.gnssFixOk = gnssFixOk;
    fix.numSv = 9u;
    return fix;
}


} // namespace

void test_notify_needs_a_connection_and_a_first_nav_pvt() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    GpsFix fix = trackerFix();
    // Not armed (no connection): nothing, even with new PVTs.
    TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(true, fix, 5u, e, 0u, kRoomy).action);
    t.arm(0u, e);
    // CONN_GPS=0 or unconfirmed pins: hasFix never becomes true, so never a block.
    for (uint32_t ms = 0u; ms < 1000u; ms += 10u) {
        TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(false, GpsFix(), 0u, e, ms, kRoomy).action);
    }
    // The first NAV-PVT after the connection is the first block.
    GpsNotifyDecision d = t.step(true, fix, 1u, e, 1000u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(0u, d.seq);
    // The same NAV-PVT is not sent twice, however many passes follow.
    TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(true, fix, 1u, e, 1100u, kRoomy).action);
}

void test_notify_starts_at_the_pvt_after_the_connection() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    t.arm(40u, e); // 40 PVTs before the phone connected: none of them is sent
    TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(true, trackerFix(), 40u, e, 0u, kRoomy).action);
    GpsNotifyDecision d = t.step(true, trackerFix(), 41u, e, 100u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(0u, d.seq);
}

void test_notify_new_pvt_by_count_and_merged_pvts_leave_a_seq_gap() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    GpsFix fix = trackerFix();
    t.arm(0u, e);
    TEST_ASSERT_EQUAL_UINT8(0u, t.step(true, fix, 1u, e, 0u, kRoomy).seq);
    // The fix time does not decide: a new count with an unchanged rxTimeMs is a new block.
    GpsNotifyDecision d = t.step(true, fix, 2u, e, 100u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(1u, d.seq);
    // The 50 ms mirror merged three PVTs: one block, seq jumps by three (two lost).
    d = t.step(true, fix, 5u, e, 200u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(4u, d.seq);
    TEST_ASSERT_EQUAL_UINT8(5u, t.step(true, fix, 6u, e, 300u, kRoomy).seq);
}

void test_notify_seq_and_pvt_count_wrap() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    GpsFix fix = trackerFix();
    uint32_t count = 0xFFFFFFF0u;
    t.arm(count, e);
    uint32_t ms = 0u;
    for (int i = 0; i < 255; i++) { // seq 0..254
        count++;
        ms += 100u;
        TEST_ASSERT_EQUAL_UINT8((uint8_t)i, t.step(true, fix, count, e, ms, kRoomy).seq);
    }
    // count crosses 2^32 and seq crosses 255 with a gap of 2 (seq 255 and 0 lost): 1.
    count += 3u;
    ms += 100u;
    GpsNotifyDecision d = t.step(true, fix, count, e, ms, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(1u, d.seq);
}

void test_notify_low_mtu_skips_but_advances_seq_and_error_base() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    GpsFix fix = trackerFix();
    t.arm(0u, e);
    // MTU 23 right after connecting: produced, not sent; seq advances.
    GpsNotifyDecision d = t.step(true, fix, 1u, e, 0u, kAttDefault);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SKIP_MTU, d.action);
    TEST_ASSERT_EQUAL_UINT8(0u, d.seq);
    // A checksum error and an overflow during the skipped block...
    GpsErrorCounters e2 = {1u, 1u};
    d = t.step(true, fix, 2u, e2, 100u, kAttDefault);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SKIP_MTU, d.action);
    TEST_ASSERT_EQUAL_UINT8(1u, d.seq);
    // One byte short of the block is still a skip.
    d = t.step(true, fix, 3u, e2, 200u, (uint16_t)(GPS_BLOCK_BYTES - 1u));
    TEST_ASSERT_EQUAL(GpsNotifyAction::SKIP_MTU, d.action);
    // ...after the MTU exchange the first sent block shows the gap (seq 3, 0..2 lost) and
    // does not repeat errors that belong to skipped blocks.
    d = t.step(true, fix, 4u, e2, 300u, (uint16_t)GPS_BLOCK_BYTES);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(3u, d.seq);
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK, d.flags);
}

void test_notify_error_flags_since_connection_and_previous_block() {
    GpsNotifyTracker t;
    GpsFix fix = trackerFix();
    GpsErrorCounters boot = {7u, 3u}; // errors before the phone connected
    t.arm(0u, boot);
    GpsNotifyDecision d = t.step(true, fix, 1u, boot, 0u, kRoomy);
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK, d.flags); // not "since boot"
    GpsErrorCounters parse = {8u, 3u};
    d = t.step(true, fix, 2u, parse, 100u, kRoomy);
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK | BLE_GPS_FLAG_PARSE_ERROR, d.flags);
    d = t.step(true, fix, 3u, parse, 200u, kRoomy); // reported once
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK, d.flags);
    GpsErrorCounters ovf = {8u, 4u};
    d = t.step(true, fix, 4u, ovf, 300u, kRoomy);
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK | BLE_GPS_FLAG_UART_OVERFLOW, d.flags);
}

void test_notify_blocks_without_a_usable_fix_are_sent() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    t.arm(0u, e);
    GpsNotifyDecision d = t.step(true, trackerFix(0u, 0u), 1u, e, 0u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_HEX8(0u, d.flags);
    d = t.step(true, trackerFix(2u, 0u), 2u, e, 100u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_HEX8(0u, d.flags);
}

void test_notify_min_spacing_holds_a_pvt_and_sends_it_later() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    GpsFix fix = trackerFix();
    t.arm(0u, e);
    TEST_ASSERT_EQUAL_UINT8(0u, t.step(true, fix, 1u, e, 1000u, kRoomy).seq);
    // The next PVT arrives early (jitter): held for GPS_BLOCK_MIN_SPACING_MS...
    const uint32_t early = 1000u + GPS_BLOCK_MIN_SPACING_MS - 1u;
    TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(true, fix, 2u, e, 1020u, kRoomy).action);
    TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(true, fix, 2u, e, early, kRoomy).action);
    // ...then sent on the next pass, with the next seq: held, never dropped.
    GpsNotifyDecision d = t.step(true, fix, 2u, e, early + 1u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(1u, d.seq);
    // The spacing is half the PVT period, not a 100 ms send timer: a 10 Hz source with
    // +-40 ms jitter keeps every PVT (consecutive seq) and the delay does not accumulate.
    const int32_t jitter[] = {40, -40, 30, -30, 40, -40, 0, 20, -20, 40};
    uint32_t count = 2u;
    uint8_t seq = 1u;
    for (unsigned i = 0; i < sizeof(jitter) / sizeof(jitter[0]); i++) {
        count++;
        uint32_t pvtMs = (uint32_t)((int32_t)(1100u + 100u * (i + 1u)) + jitter[i]);
        GpsNotifyDecision step = {GpsNotifyAction::NONE, 0u, 0u};
        uint32_t ms = pvtMs;
        while (step.action == GpsNotifyAction::NONE) {
            step = t.step(true, fix, count, e, ms, kRoomy);
            ms += 5u; // loop passes
            TEST_ASSERT_TRUE(ms < pvtMs + 100u); // sent before the next PVT
        }
        seq++;
        TEST_ASSERT_EQUAL_UINT8(seq, step.seq);
    }
}

void test_notify_disconnect_resets_last_block_state() {
    GpsNotifyTracker t;
    GpsErrorCounters e = {0u, 0u};
    GpsFix fix = trackerFix();
    t.arm(0u, e);
    TEST_ASSERT_EQUAL_UINT8(0u, t.step(true, fix, 1u, e, 1000u, kRoomy).seq);
    t.disarm();
    TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(true, fix, 2u, e, 1100u, kRoomy).action);
    // Reconnect 10 ms after the last block: no spacing hold from the old connection, the
    // error base is the counters at reconnection, PVTs in between are not replayed, and seq
    // keeps rolling.
    GpsErrorCounters e2 = {5u, 5u};
    t.arm(3u, e2);
    TEST_ASSERT_EQUAL(GpsNotifyAction::NONE, t.step(true, fix, 3u, e2, 1010u, kRoomy).action);
    GpsNotifyDecision d = t.step(true, fix, 4u, e2, 1010u, kRoomy);
    TEST_ASSERT_EQUAL(GpsNotifyAction::SEND, d.action);
    TEST_ASSERT_EQUAL_UINT8(1u, d.seq);
    TEST_ASSERT_EQUAL_HEX8(BLE_GPS_FLAG_GNSS_FIX_OK, d.flags);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_cfg_frames_match_known_good_bytes);
    RUN_TEST(test_cfg_prt_is_ubx_only_at_the_requested_baud);
    RUN_TEST(test_cfg_builder_refuses_a_short_buffer);
    RUN_TEST(test_nav_pvt_decodes_motion_fields);
    RUN_TEST(test_negative_speed_and_flags_bits_other_than_fix_ok);
    RUN_TEST(test_no_position_leaves_the_parser);
    RUN_TEST(test_bytes_split_across_feeds_and_back_to_back_frames);
    RUN_TEST(test_bad_checksum_is_counted_and_the_next_frame_parses);
    RUN_TEST(test_corrupt_payload_byte_fails_the_checksum);
    RUN_TEST(test_oversized_length_is_counted_and_skipped);
    RUN_TEST(test_nav_pvt_with_the_wrong_length_is_refused);
    RUN_TEST(test_resync_after_garbage_nmea_and_a_lone_sync_byte);
    RUN_TEST(test_header_lookalike_in_garbage_costs_at_most_the_next_frame);
    RUN_TEST(test_other_messages_are_ignored_but_valid);
    RUN_TEST(test_zero_length_message);
    RUN_TEST(test_configure_warm_receiver_already_at_38400);
    RUN_TEST(test_configure_cold_receiver_moves_from_9600);
    RUN_TEST(test_configure_without_receiver_reports_it_and_is_bounded);
    RUN_TEST(test_core_publishes_fix_with_node_time_and_counts_overflow);
    RUN_TEST(test_step_detects_silence_and_reconfigures);
    RUN_TEST(test_seqlock_read_before_write_and_round_trip);
    RUN_TEST(test_seqlock_reader_never_sees_a_torn_copy);
    RUN_TEST(test_gps_block_matches_known_good_bytes);
    RUN_TEST(test_gps_block_refuses_a_short_buffer);
    RUN_TEST(test_gps_block_flags_from_fix_and_counter_changes);
    RUN_TEST(test_gps_block_from_a_parsed_nav_pvt_has_no_position);
    RUN_TEST(test_notify_needs_a_connection_and_a_first_nav_pvt);
    RUN_TEST(test_notify_starts_at_the_pvt_after_the_connection);
    RUN_TEST(test_notify_new_pvt_by_count_and_merged_pvts_leave_a_seq_gap);
    RUN_TEST(test_notify_seq_and_pvt_count_wrap);
    RUN_TEST(test_notify_low_mtu_skips_but_advances_seq_and_error_base);
    RUN_TEST(test_notify_error_flags_since_connection_and_previous_block);
    RUN_TEST(test_notify_blocks_without_a_usable_fix_are_sent);
    RUN_TEST(test_notify_min_spacing_holds_a_pvt_and_sends_it_later);
    RUN_TEST(test_notify_disconnect_resets_last_block_state);
    return UNITY_END();
}
