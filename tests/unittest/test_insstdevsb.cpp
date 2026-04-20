#include "binary_test_utils.h"
#include "novatel/insstdevsb.h"

// ── Helper: build INSSTDEV data block (52 bytes) ────────────────────────────
static std::vector<uint8_t> makeInsStdevsData(
    float    latSig,
    float    lonSig,
    float    hgtSig,
    float    nVelSig,
    float    eVelSig,
    float    uVelSig,
    float    rollSig,
    float    pitchSig,
    float    azSig,
    uint32_t extSolStat,
    uint16_t timeSinceUpdate,
    uint16_t r1,
    uint32_t r2,
    uint32_t r3) {
    std::vector<uint8_t> d(52, 0);
    writeF32LE(d.data() + 0, latSig);
    writeF32LE(d.data() + 4, lonSig);
    writeF32LE(d.data() + 8, hgtSig);
    writeF32LE(d.data() + 12, nVelSig);
    writeF32LE(d.data() + 16, eVelSig);
    writeF32LE(d.data() + 20, uVelSig);
    writeF32LE(d.data() + 24, rollSig);
    writeF32LE(d.data() + 28, pitchSig);
    writeF32LE(d.data() + 32, azSig);
    writeU32LE(d.data() + 36, extSolStat);
    writeU16LE(d.data() + 40, timeSinceUpdate);
    writeU16LE(d.data() + 42, r1);
    writeU32LE(d.data() + 44, r2);
    writeU32LE(d.data() + 48, r3);
    return d;
}

// ── Test 1: valid single frame ───────────────────────────────────────────────
bool testInsStdevsB_SingleFrame() {
    auto                      data  = makeInsStdevsData(0.001f, 0.002f, 0.005f, 0.01f, 0.02f, 0.05f, 0.1f, 0.2f, 0.3f, 0x00000001u, 5, 0, 0, 0);
    auto                      frame = buildBinaryFrame(2051, 2344, 405000000u, data.data(), (uint16_t)data.size());
    bool                      rawOk = false;
    Novatel::InsStdevsB       captured{};
    Novatel::InsStdevsBParser parser(
        [&](const Novatel::InsStdevsB &f) {
            captured = f;
        },
        [&](const uint8_t *, uint32_t len) {
            rawOk = (len == frame.size());
        });
    parser.Parse(frame.data(), (uint32_t)frame.size());

    return rawOk
           && fEq(captured.latSigma, 0.001f, 1e-5f)
           && fEq(captured.lonSigma, 0.002f, 1e-5f)
           && fEq(captured.hgtSigma, 0.005f, 1e-5f)
           && fEq(captured.northVelSigma, 0.01f, 1e-5f)
           && fEq(captured.eastVelSigma, 0.02f, 1e-5f)
           && fEq(captured.upVelSigma, 0.05f, 1e-5f)
           && fEq(captured.rollSigma, 0.1f, 1e-5f)
           && fEq(captured.pitchSigma, 0.2f, 1e-5f)
           && fEq(captured.azimuthSigma, 0.3f, 1e-5f)
           && captured.extSolStat == 0x00000001u
           && captured.timeSinceUpdate == 5
           && captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 405000.0);
}

// ── Test 2: all sigma fields non-trivial ─────────────────────────────────────
bool testInsStdevsB_AllSigmas() {
    auto                      data  = makeInsStdevsData(1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f, 9.9f, 0, 100, 0, 0, 0);
    auto                      frame = buildBinaryFrame(2051, 2100, 50000u, data.data(), (uint16_t)data.size());
    Novatel::InsStdevsB       captured{};
    Novatel::InsStdevsBParser parser(
        [&](const Novatel::InsStdevsB &f) {
            captured = f;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return fEq(captured.latSigma, 1.1f, 1e-5f)
           && fEq(captured.rollSigma, 7.7f, 1e-5f)
           && fEq(captured.azimuthSigma, 9.9f, 1e-5f)
           && captured.timeSinceUpdate == 100;
}

// ── Test 3: bad CRC ──────────────────────────────────────────────────────────
bool testInsStdevsB_BadCrc() {
    auto                      data  = makeInsStdevsData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                      frame = corruptCrc(buildBinaryFrame(2051, 2344, 0, data.data(), (uint16_t)data.size()));
    bool                      fired = false;
    Novatel::InsStdevsBParser parser(
        [&](const Novatel::InsStdevsB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 4: wrong message ID ─────────────────────────────────────────────────
bool testInsStdevsB_WrongId() {
    auto                      data  = makeInsStdevsData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                      frame = buildBinaryFrame(1, 2344, 0, data.data(), (uint16_t)data.size());
    bool                      fired = false;
    Novatel::InsStdevsBParser parser(
        [&](const Novatel::InsStdevsB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 5: two consecutive frames ──────────────────────────────────────────
bool testInsStdevsB_TwoFrames() {
    auto                 d1 = makeInsStdevsData(0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0, 0, 0, 0, 0);
    auto                 d2 = makeInsStdevsData(0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 0.2f, 1, 0, 0, 0, 0);
    auto                 f1 = buildBinaryFrame(2051, 2344, 1000u, d1.data(), (uint16_t)d1.size());
    auto                 f2 = buildBinaryFrame(2051, 2344, 2000u, d2.data(), (uint16_t)d2.size());
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());
    int                       count = 0;
    Novatel::InsStdevsBParser parser(
        [&](const Novatel::InsStdevsB &) {
            count++;
        },
        nullptr);
    parser.Parse(stream.data(), (uint32_t)stream.size());
    return count == 2;
}

int main() {
    std::cout << "\n=== test_insstdevsb ===\n\n";
    CHECK(testInsStdevsB_SingleFrame());
    CHECK(testInsStdevsB_AllSigmas());
    CHECK(testInsStdevsB_BadCrc());
    CHECK(testInsStdevsB_WrongId());
    CHECK(testInsStdevsB_TwoFrames());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
