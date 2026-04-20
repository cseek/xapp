#include "binary_test_utils.h"
#include "novatel/rawimub.h"

// ── Helper: build a RAWIMU binary data block (40 bytes) ─────────────────────
static std::vector<uint8_t> makeRawImuData(
    uint32_t week,
    double   seconds,
    uint32_t imuStatus,
    int32_t  zAccel,
    int32_t  negYAccel,
    int32_t  xAccel,
    int32_t  zGyro,
    int32_t  negYGyro,
    int32_t  xGyro) {
    std::vector<uint8_t> d(40, 0);
    writeU32LE(d.data() + 0, week);
    writeF64LE(d.data() + 4, seconds);
    writeU32LE(d.data() + 12, imuStatus);
    writeI32LE(d.data() + 16, zAccel);
    writeI32LE(d.data() + 20, negYAccel);
    writeI32LE(d.data() + 24, xAccel);
    writeI32LE(d.data() + 28, zGyro);
    writeI32LE(d.data() + 32, negYGyro);
    writeI32LE(d.data() + 36, xGyro);
    return d;
}

// ── Test 1: valid single frame, all fields ───────────────────────────────────
bool testRawImuBSingleFrame() {
    auto data  = makeRawImuData(2344, 404999.998, 0x00000000, -1234567, 2345678, -345678, 98765, -87654, 123456);
    auto frame = buildBinaryFrame(268, 2344, 404999998u, data.data(), (uint16_t)data.size());

    bool                   rawOk = false;
    Novatel::RawImuB       captured{};
    Novatel::RawImuBParser parser(
        [&](const Novatel::RawImuB &f) {
            captured = f;
        },
        [&](const uint8_t *buf, uint32_t len) {
            rawOk = (len == frame.size() && buf[0] == 0xAA);
        });
    parser.Parse(frame.data(), (uint32_t)frame.size());

    return rawOk
           && captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 404999.998)
           && captured.imuStatus == 0u
           && captured.zAccel == -1234567
           && captured.negYAccel == 2345678
           && captured.xAccel == -345678
           && captured.zGyro == 98765
           && captured.negYGyro == -87654
           && captured.xGyro == 123456;
}

// ── Test 2: imuStatus non-zero ───────────────────────────────────────────────
bool testRawImuBImuStatus() {
    auto                   data  = makeRawImuData(2344, 405000.010, 0x0A1B2C3Du, 1000000, -2000000, 300000, -10000, 20000, -30000);
    auto                   frame = buildBinaryFrame(268, 2344, 405000010u, data.data(), (uint16_t)data.size());
    Novatel::RawImuB       captured{};
    Novatel::RawImuBParser parser(
        [&](const Novatel::RawImuB &f) {
            captured = f;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return captured.imuStatus == 0x0A1B2C3Du
           && captured.zAccel == 1000000
           && captured.xGyro == -30000;
}

// ── Test 3: bad CRC — frame callback must NOT fire ───────────────────────────
bool testRawImuBBadCrc() {
    auto                   data     = makeRawImuData(2344, 405000.0, 0, 0, 0, 0, 0, 0, 0);
    auto                   frame    = corruptCrc(buildBinaryFrame(268, 2344, 405000000u, data.data(), (uint16_t)data.size()));
    bool                   rawFired = false, frameFired = false;
    Novatel::RawImuBParser parser(
        [&](const Novatel::RawImuB &) {
            frameFired = true;
        },
        [&](const uint8_t *, uint32_t) {
            rawFired = true;
        });
    parser.Parse(frame.data(), (uint32_t)frame.size());
    // raw fires (delivery of raw bytes happens before CRC check in this parser),
    // but frame callback must NOT fire.
    return !frameFired;
}

// ── Test 4: wrong message ID silently ignored ────────────────────────────────
bool testRawImuBWrongMsgId() {
    auto                   data     = makeRawImuData(2344, 405000.0, 0, 0, 0, 0, 0, 0, 0);
    auto                   frame    = buildBinaryFrame(999 /*wrong*/, 2344, 405000000u, data.data(), (uint16_t)data.size());
    bool                   rawFired = false, frameFired = false;
    Novatel::RawImuBParser parser(
        [&](const Novatel::RawImuB &) {
            frameFired = true;
        },
        [&](const uint8_t *, uint32_t) {
            rawFired = true;
        });
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !rawFired && !frameFired;
}

// ── Test 5: two consecutive frames ──────────────────────────────────────────
bool testRawImuBTwoFrames() {
    auto                 d1 = makeRawImuData(2344, 405000.0, 0, 100, 200, 300, 400, 500, 600);
    auto                 d2 = makeRawImuData(2344, 405001.0, 1, -100, -200, -300, -400, -500, -600);
    auto                 f1 = buildBinaryFrame(268, 2344, 405000000u, d1.data(), (uint16_t)d1.size());
    auto                 f2 = buildBinaryFrame(268, 2344, 405001000u, d2.data(), (uint16_t)d2.size());
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());

    int                    count = 0;
    Novatel::RawImuBParser parser(
        [&](const Novatel::RawImuB &) {
            count++;
        },
        nullptr);
    parser.Parse(stream.data(), (uint32_t)stream.size());
    return count == 2;
}

// ── main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_rawimub ===\n\n";
    CHECK(testRawImuBSingleFrame());
    CHECK(testRawImuBImuStatus());
    CHECK(testRawImuBBadCrc());
    CHECK(testRawImuBWrongMsgId());
    CHECK(testRawImuBTwoFrames());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
