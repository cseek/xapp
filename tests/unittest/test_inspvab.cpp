#include "binary_test_utils.h"
#include "novatel/inspvab.h"

// ── Helper: build INSPVA data block (88 bytes) ──────────────────────────────
static std::vector<uint8_t> makeInsPvaData(
    uint32_t week,
    double   seconds,
    double   lat,
    double   lon,
    double   height,
    double   northVel,
    double   eastVel,
    double   upVel,
    double   roll,
    double   pitch,
    double   azimuth,
    uint32_t status) {
    std::vector<uint8_t> d(88, 0);
    writeU32LE(d.data() + 0, week);
    writeF64LE(d.data() + 4, seconds);
    writeF64LE(d.data() + 12, lat);
    writeF64LE(d.data() + 20, lon);
    writeF64LE(d.data() + 28, height);
    writeF64LE(d.data() + 36, northVel);
    writeF64LE(d.data() + 44, eastVel);
    writeF64LE(d.data() + 52, upVel);
    writeF64LE(d.data() + 60, roll);
    writeF64LE(d.data() + 68, pitch);
    writeF64LE(d.data() + 76, azimuth);
    writeU32LE(d.data() + 84, status);
    return d;
}

// ── Test 1: valid single frame ───────────────────────────────────────────────
bool testInsPvaB_SingleFrame() {
    auto data  = makeInsPvaData(2344, 405000.000, 31.12345678, 121.34567890, 45.678, 0.123, -0.456, 0.789, 0.1, -0.2, 95.5, 3);
    auto frame = buildBinaryFrame(507, 2344, 405000000u, data.data(), (uint16_t)data.size());

    bool                   rawOk = false;
    Novatel::InsPvaB       captured{};
    Novatel::InsPvaBParser parser(
        [&](const Novatel::InsPvaB &f) {
            captured = f;
        },
        [&](const uint8_t *, uint32_t len) {
            rawOk = (len == frame.size());
        });
    parser.Parse(frame.data(), (uint32_t)frame.size());

    return rawOk
           && captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 405000.0)
           && fEq(captured.lat, 31.12345678, 1e-8)
           && fEq(captured.lon, 121.34567890, 1e-8)
           && fEq(captured.height, 45.678, 1e-3)
           && fEq(captured.northVel, 0.123, 1e-6)
           && fEq(captured.eastVel, -0.456, 1e-6)
           && fEq(captured.upVel, 0.789, 1e-6)
           && fEq(captured.roll, 0.1, 1e-6)
           && fEq(captured.pitch, -0.2, 1e-6)
           && fEq(captured.azimuth, 95.5, 1e-6)
           && captured.status == 3u;
}

// ── Test 2: velocity and attitude values ─────────────────────────────────────
bool testInsPvaB_VelAttitude() {
    auto                   data  = makeInsPvaData(2000, 100.0, 0.0, 0.0, 0.0, 10.5, -3.2, 0.1, 5.0, -2.0, 270.0, 5);
    auto                   frame = buildBinaryFrame(507, 2000, 100000u, data.data(), (uint16_t)data.size());
    Novatel::InsPvaB       captured{};
    Novatel::InsPvaBParser parser(
        [&](const Novatel::InsPvaB &f) {
            captured = f;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return fEq(captured.northVel, 10.5, 1e-6)
           && fEq(captured.eastVel, -3.2, 1e-6)
           && fEq(captured.roll, 5.0, 1e-6)
           && fEq(captured.azimuth, 270.0, 1e-6)
           && captured.status == 5u;
}

// ── Test 3: bad CRC ──────────────────────────────────────────────────────────
bool testInsPvaB_BadCrc() {
    auto                   data  = makeInsPvaData(2344, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                   frame = corruptCrc(buildBinaryFrame(507, 2344, 0, data.data(), (uint16_t)data.size()));
    bool                   fired = false;
    Novatel::InsPvaBParser parser(
        [&](const Novatel::InsPvaB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 4: wrong message ID ─────────────────────────────────────────────────
bool testInsPvaB_WrongId() {
    auto                   data  = makeInsPvaData(2344, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                   frame = buildBinaryFrame(1, 2344, 0, data.data(), (uint16_t)data.size());
    bool                   fired = false;
    Novatel::InsPvaBParser parser(
        [&](const Novatel::InsPvaB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 5: two consecutive frames ──────────────────────────────────────────
bool testInsPvaB_TwoFrames() {
    auto                 d1 = makeInsPvaData(2344, 405000.0, 31.0, 121.0, 50.0, 1.0, 2.0, 3.0, 0, 0, 90.0, 3);
    auto                 d2 = makeInsPvaData(2344, 405001.0, 31.1, 121.1, 51.0, 1.1, 2.1, 3.1, 0, 0, 91.0, 3);
    auto                 f1 = buildBinaryFrame(507, 2344, 405000000u, d1.data(), (uint16_t)d1.size());
    auto                 f2 = buildBinaryFrame(507, 2344, 405001000u, d2.data(), (uint16_t)d2.size());
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());
    int                    count = 0;
    Novatel::InsPvaBParser parser(
        [&](const Novatel::InsPvaB &) {
            count++;
        },
        nullptr);
    parser.Parse(stream.data(), (uint32_t)stream.size());
    return count == 2;
}

int main() {
    std::cout << "\n=== test_inspvab ===\n\n";
    CHECK(testInsPvaB_SingleFrame());
    CHECK(testInsPvaB_VelAttitude());
    CHECK(testInsPvaB_BadCrc());
    CHECK(testInsPvaB_WrongId());
    CHECK(testInsPvaB_TwoFrames());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
