#include "binary_test_utils.h"
#include "novatel/bestgnssposb.h"

// ── Helper: build BESTGNSSPOS data block (72 bytes) ─────────────────────────
static std::vector<uint8_t> makeBestGnssPosData(
    uint32_t   solStatus,
    uint32_t   posType,
    double     lat,
    double     lon,
    double     hgt,
    float      undulation,
    uint32_t   datumId,
    float      latStd,
    float      lonStd,
    float      hgtStd,
    const char stnId[4],
    float      diffAge,
    float      solAge,
    uint8_t    numSvs,
    uint8_t    numSolSvs,
    uint8_t    numGgL1,
    uint8_t    numSolMultiSvs,
    uint8_t    extSolStat,
    uint8_t    galBdsSigMask,
    uint8_t    gpsGloSigMask) {
    std::vector<uint8_t> d(72, 0);
    writeU32LE(d.data() + 0, solStatus);
    writeU32LE(d.data() + 4, posType);
    writeF64LE(d.data() + 8, lat);
    writeF64LE(d.data() + 16, lon);
    writeF64LE(d.data() + 24, hgt);
    writeF32LE(d.data() + 32, undulation);
    writeU32LE(d.data() + 36, datumId);
    writeF32LE(d.data() + 40, latStd);
    writeF32LE(d.data() + 44, lonStd);
    writeF32LE(d.data() + 48, hgtStd);
    memcpy(d.data() + 52, stnId, 4);
    writeF32LE(d.data() + 56, diffAge);
    writeF32LE(d.data() + 60, solAge);
    d[64] = numSvs;
    d[65] = numSolSvs;
    d[66] = numGgL1;
    d[67] = numSolMultiSvs;
    d[68] = 0;
    d[69] = extSolStat;
    d[70] = galBdsSigMask;
    d[71] = gpsGloSigMask;
    return d;
}

// ── Test 1: valid single frame ───────────────────────────────────────────────
bool testBestGnssPosB_SingleFrame() {
    const char stn[4] = {'0', '0', '0', '0'};
    auto       data   = makeBestGnssPosData(0, 8, 31.12345678, 121.34567890, 45.678, -10.5f, 61, 0.002f, 0.003f, 0.005f, stn, 0.0f, 0.5f, 16, 14, 12, 10, 0, 0, 0);
    auto       frame  = buildBinaryFrame(1429, 2344, 405000000u, data.data(), (uint16_t)data.size());

    bool                        rawOk = false;
    Novatel::BestGnssPosB       captured{};
    Novatel::BestGnssPosBParser parser(
        [&](const Novatel::BestGnssPosB &f) {
            captured = f;
        },
        [&](const uint8_t *buf, uint32_t len) {
            rawOk = (len == frame.size());
        });
    parser.Parse(frame.data(), (uint32_t)frame.size());

    return rawOk
           && captured.solStatus == 0u
           && captured.posType == 8u
           && fEq(captured.lat, 31.12345678, 1e-8)
           && fEq(captured.lon, 121.34567890, 1e-8)
           && fEq(captured.hgt, 45.678, 1e-3)
           && fEq(captured.undulation, -10.5f, 1e-3f)
           && captured.numSvs == 16
           && captured.numSolSvs == 14
           && captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 405000.0);
}

// ── Test 2: std-dev fields ───────────────────────────────────────────────────
bool testBestGnssPosB_StdDev() {
    const char                  stn[4] = {'A', 'B', 'C', 'D'};
    auto                        data   = makeBestGnssPosData(0, 8, 0.0, 0.0, 0.0, 0.0f, 61, 0.012f, 0.013f, 0.025f, stn, 1.5f, 2.0f, 8, 8, 6, 4, 0x01, 0x02, 0x03);
    auto                        frame  = buildBinaryFrame(1429, 2100, 100000u, data.data(), (uint16_t)data.size());
    Novatel::BestGnssPosB       captured{};
    Novatel::BestGnssPosBParser parser(
        [&](const Novatel::BestGnssPosB &f) {
            captured = f;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return fEq(captured.latStd, 0.012f, 1e-5f)
           && fEq(captured.lonStd, 0.013f, 1e-5f)
           && fEq(captured.hgtStd, 0.025f, 1e-5f)
           && fEq(captured.diffAge, 1.5f, 1e-5f)
           && captured.extSolStat == 0x01
           && captured.galBdsSigMask == 0x02
           && captured.gpsGloSigMask == 0x03;
}

// ── Test 3: bad CRC ──────────────────────────────────────────────────────────
bool testBestGnssPosB_BadCrc() {
    const char                  stn[4] = {0, 0, 0, 0};
    auto                        data   = makeBestGnssPosData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, stn, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                        frame  = corruptCrc(buildBinaryFrame(1429, 2344, 0, data.data(), (uint16_t)data.size()));
    bool                        fired  = false;
    Novatel::BestGnssPosBParser parser(
        [&](const Novatel::BestGnssPosB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 4: wrong message ID ignored ────────────────────────────────────────
bool testBestGnssPosB_WrongId() {
    const char                  stn[4] = {0, 0, 0, 0};
    auto                        data   = makeBestGnssPosData(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, stn, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    auto                        frame  = buildBinaryFrame(1, 2344, 0, data.data(), (uint16_t)data.size());
    bool                        fired  = false;
    Novatel::BestGnssPosBParser parser(
        [&](const Novatel::BestGnssPosB &) {
            fired = true;
        },
        nullptr);
    parser.Parse(frame.data(), (uint32_t)frame.size());
    return !fired;
}

// ── Test 5: two consecutive frames ──────────────────────────────────────────
bool testBestGnssPosB_TwoFrames() {
    const char           stn[4] = {0, 0, 0, 0};
    auto                 d1     = makeBestGnssPosData(0, 8, 10.0, 20.0, 100.0, 0, 61, 0, 0, 0, stn, 0, 0, 12, 10, 8, 6, 0, 0, 0);
    auto                 d2     = makeBestGnssPosData(0, 8, 11.0, 21.0, 101.0, 0, 61, 0, 0, 0, stn, 0, 0, 14, 12, 8, 6, 0, 0, 0);
    auto                 f1     = buildBinaryFrame(1429, 2344, 1000u, d1.data(), (uint16_t)d1.size());
    auto                 f2     = buildBinaryFrame(1429, 2344, 2000u, d2.data(), (uint16_t)d2.size());
    std::vector<uint8_t> stream;
    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());
    int                         count = 0;
    Novatel::BestGnssPosBParser parser(
        [&](const Novatel::BestGnssPosB &) {
            count++;
        },
        nullptr);
    parser.Parse(stream.data(), (uint32_t)stream.size());
    return count == 2;
}

int main() {
    std::cout << "\n=== test_bestgnssposb ===\n\n";
    CHECK(testBestGnssPosB_SingleFrame());
    CHECK(testBestGnssPosB_StdDev());
    CHECK(testBestGnssPosB_BadCrc());
    CHECK(testBestGnssPosB_WrongId());
    CHECK(testBestGnssPosB_TwoFrames());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
