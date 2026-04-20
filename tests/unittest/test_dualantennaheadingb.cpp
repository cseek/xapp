#include "binary_test_utils.h"
#include "novatel/dualantennaheadingb.h"

// ── Helper: build DUALANTENNAHEADING data block (44 bytes) ──────────────────
static std::vector<uint8_t> makeDualAntData(
    uint32_t   solStatus,
    uint32_t   posType,
    float      length,
    float      heading,
    float      pitch,
    float      hdgStdDev,
    float      ptchStdDev,
    const char stnId[4],
    uint8_t    numSvs,
    uint8_t    numSolnSvs,
    uint8_t    numObs,
    uint8_t    numMulti,
    uint8_t    solSource,
    uint8_t    extSolStat,
    uint8_t    galBdsSigMask,
    uint8_t    gpsGloSigMask) {
    std::vector<uint8_t> d(44, 0);
    writeU32LE(d.data() + 0, solStatus);
    writeU32LE(d.data() + 4, posType);
    writeF32LE(d.data() + 8, length);
    writeF32LE(d.data() + 12, heading);
    writeF32LE(d.data() + 16, pitch);
    // d[20:24] reserved float = 0
    writeF32LE(d.data() + 24, hdgStdDev);
    writeF32LE(d.data() + 28, ptchStdDev);
    memcpy(d.data() + 32, stnId, 4);
    d[36] = numSvs;
    d[37] = numSolnSvs;
    d[38] = numObs;
    d[39] = numMulti;
    d[40] = solSource;
    d[41] = extSolStat;
    d[42] = galBdsSigMask;
    d[43] = gpsGloSigMask;
    return d;
}

// ── Test 1: valid single frame ───────────────────────────────────────────────
bool testDualAntHeadingB_SingleFrame() {
    const char stn[4] = {'0', '0', '0', '0'};
    auto       data   = makeDualAntData(0, 50, 2.456f, 123.456f, -1.23f, 0.5f, 0.3f, stn, 12, 10, 10, 8, 1, 0, 0, 0);
    auto       frame  = buildBinaryFrame(971, 2344, 405000000u, data.data(), (uint16_t)data.size());

    bool                               rawOk = false;
    Novatel::DualAntennaHeadingB       captured{};
    Novatel::DualAntennaHeadingBParser parser(
        [&](const Novatel::DualAntennaHeadingB &f) {
            captured = f;
        },
        [&](const uint8_t *, uint32_t len) {
            rawOk = (len == frame.size());
        });
    parser.Parse(frame.data(), (uint32_t)frame.size());

    return rawOk
           && captured.solStatus == 0u
           && captured.posType == 50u
           && fEq(captured.length, 2.456f, 1e-3f)
           && fEq(captured.heading, 123.456f, 1e-3f)
           && fEq(captured.pitch, -1.23f, 1e-3f)
           && fEq(captured.hdgStdDev, 0.5f, 1e-4f)
           && fEq(captured.ptchStdDev, 0.3f, 1e-4f)
           && captured.numSvs == 12
           && captured.numSolnSvs == 10
           && captured.solSource == 1
           && captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 405000.0);
}

// ── Test 2: heading wraps to 359 ─────────────────────────────────────────────
bool testDualAntHeadingB_HeadingFields() {
    const char                         stn[4] = {0, 0, 0, 0};
    auto                               data   = makeDualAntData(0, 50, -1.0f, 359.999f, 0.0f, 0.1f, 0.2f, stn, 8, 8, 8, 6, 0, 2, 3, 4);
    auto                               frame  = buildBinaryFrame(971, 2000, 50000u, data.data(), (uint16_t)data.size());
    Novatel::DualAntennaHeadingB       captured{};
    Novatel::DualAntennaHeadingBParser parser(
        [&](const Novatel::DualAntennaHeadingB &f) {
            captured = f;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return fEq(captured.heading, 359.999f, 1e-3f)
           && captured.extSolStat == 2
           && captured.galBdsSigMask == 3
           && captured.gpsGloSigMask == 4;
}

// ── Test 3: bad CRC ──────────────────────────────────────────────────────────
bool testDualAntHeadingB_BadCrc() {
    const char                         stn[4] = {0, 0, 0, 0};
    auto                               data   = makeDualAntData(0, 0, 0, 0, 0, 0, 0, stn, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                               frame  = corruptCrc(buildBinaryFrame(971, 2344, 0, data.data(), (uint16_t)data.size()));
    bool                               fired  = false;
    Novatel::DualAntennaHeadingBParser parser(
        [&](const Novatel::DualAntennaHeadingB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 4: wrong message ID ─────────────────────────────────────────────────
bool testDualAntHeadingB_WrongId() {
    const char                         stn[4] = {0, 0, 0, 0};
    auto                               data   = makeDualAntData(0, 0, 0, 0, 0, 0, 0, stn, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                               frame  = buildBinaryFrame(1, 2344, 0, data.data(), (uint16_t)data.size());
    bool                               fired  = false;
    Novatel::DualAntennaHeadingBParser parser(
        [&](const Novatel::DualAntennaHeadingB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 5: two consecutive frames ──────────────────────────────────────────
bool testDualAntHeadingB_TwoFrames() {
    const char           stn[4] = {0, 0, 0, 0};
    auto                 d1     = makeDualAntData(0, 50, 2.0f, 90.0f, 1.0f, 0.5f, 0.3f, stn, 12, 10, 10, 8, 1, 0, 0, 0);
    auto                 d2     = makeDualAntData(0, 50, 2.0f, 180.0f, 2.0f, 0.4f, 0.2f, stn, 10, 8, 8, 6, 1, 0, 0, 0);
    auto                 f1     = buildBinaryFrame(971, 2344, 1000u, d1.data(), (uint16_t)d1.size());
    auto                 f2     = buildBinaryFrame(971, 2344, 2000u, d2.data(), (uint16_t)d2.size());
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());
    int                                count = 0;
    Novatel::DualAntennaHeadingBParser parser(
        [&](const Novatel::DualAntennaHeadingB &) {
            count++;
        },
        nullptr);
    parser.Parse(stream.data(), (uint32_t)stream.size());
    return count == 2;
}

int main() {
    std::cout << "\n=== test_dualantennaheadingb ===\n\n";
    CHECK(testDualAntHeadingB_SingleFrame());
    CHECK(testDualAntHeadingB_HeadingFields());
    CHECK(testDualAntHeadingB_BadCrc());
    CHECK(testDualAntHeadingB_WrongId());
    CHECK(testDualAntHeadingB_TwoFrames());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
