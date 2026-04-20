#include "test_utils.h"
#include "novatel/rawimua.h"
#include "nmea/gga.h"
#include "nmea/gsv.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────
static const char RAWIMU_FRAME1[] = "#RAWIMUA,COM1,0,92.5,FINESTEERING,2344,405000.000,02000020,e5e1,32;"
                                    "2344,404999.998,00000000,-1234567,2345678,-345678,98765,-87654,123456*3599623f\r\n";
static const char RAWIMU_FRAME2[] = "#RAWIMUA,COM1,1,91.0,FINESTEERING,2344,405000.010,02000020,e5e1,32;"
                                    "2344,405000.010,00000001,1000000,-2000000,300000,-10000,20000,-30000*029cff6a\r\n";

static const char BDGGA_FRAME1[] = "$BDGGA,092725.00,3114.51480,N,12129.55470,E,1,08,1.2,50.3,M,-8.5,M,0.0,0000*70\r\n";
static const char BDGGA_FRAME2[] = "$BDGGA,093000.00,3114.52000,N,12129.56000,E,2,12,0.8,51.0,M,-8.5,M,1.5,0002*79\r\n";
static const char GPGGA_FRAME[]  = "$GPGGA,092725.00,3114.51480,N,12129.55470,E,1,09,1.0,50.3,M,-8.5,M,0.0,0000*62\r\n";
static const char GLGGA_FRAME[]  = "$GLGGA,092725.00,3114.51480,N,12129.55470,E,1,06,1.5,50.3,M,-8.5,M,0.0,0000*74\r\n";
static const char GAGGA_FRAME[]  = "$GAGGA,092725.00,3114.51480,N,12129.55470,E,1,04,1.8,50.3,M,-8.5,M,0.0,0000*76\r\n";
static const char GNGGA_FRAME[]  = "$GNGGA,092725.00,3114.51480,N,12129.55470,E,2,16,0.6,50.3,M,-8.5,M,1.2,0001*74\r\n";

static const char GPGSV_FRAME1[] = "$GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75\r\n";
static const char GPGSV_FRAME2[] = "$GPGSV,2,2,08,15,07,135,42,16,24,319,38,25,30,088,44,26,12,179,40*7A\r\n";

// ── RawImu 测试 ───────────────────────────────────────────────────────────────

bool testRawImuSingleFrame() {
    Novatel::RawImuA       captured{};
    bool                   rawOk = false, frameOk = false;
    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 8) && (memcmp(data, "#RAWIMUA", 8) == 0);
        });
    auto v = toVec(RAWIMU_FRAME1);
    parser.Parse(v.data(), v.size());
    return rawOk && frameOk
           && captured.gpsWeek == 2344 && fEq(captured.gpsSeconds, 404999.998)
           && captured.imuStatus == 0x00000000u
           && captured.zAccel == -1234567 && captured.negYAccel == 2345678
           && captured.xAccel == -345678
           && captured.zGyro == 98765 && captured.negYGyro == -87654
           && captured.xGyro == 123456;
}

bool testRawImuStickyPacket() {
    int                    count = 0;
    Novatel::RawImuAParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });
    const uint8_t        noise[] = {0xFF, 0xAA, 0x01, 0x02};
    std::vector<uint8_t> buf     = toVec(RAWIMU_FRAME1);
    rawAppend(buf, noise, sizeof(noise));
    vecAppend(buf, toVec(RAWIMU_FRAME2));
    parser.Parse(buf.data(), buf.size());
    return count == 2;
}

bool testRawImuSplitPacketByte() {
    bool                   rawOk = false;
    Novatel::RawImuAParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });
    for(uint8_t b : toVec(RAWIMU_FRAME1)) {
        parser.ParseByteStream(b);
    }
    return rawOk;
}

bool testRawImuSplitPacketChunks() {
    bool                   rawOk = false;
    Novatel::RawImuAParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });
    auto     v  = toVec(RAWIMU_FRAME1);
    uint32_t c1 = static_cast<uint32_t>(v.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(v.size() * 2 / 3);
    parser.Parse(v.data(), c1);
    parser.Parse(v.data() + c1, c2 - c1);
    parser.Parse(v.data() + c2, static_cast<uint32_t>(v.size()) - c2);
    return rawOk;
}

// ── GGA 测试 ──────────────────────────────────────────────────────────────────

bool testGgaSingleFrameBd() {
    Nmea::Gga       captured{};
    bool            rawOk = false, frameOk = false;
    Nmea::GgaParser parser(
        [&](const Nmea::Gga &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 6) && (memcmp(data, "$BDGGA", 6) == 0);
        });
    auto v = toVec(BDGGA_FRAME1);
    parser.Parse(v.data(), v.size());
    return rawOk && frameOk
           && strcmp(captured.talkerId, "BD") == 0
           && fEq(captured.utcTime, 92725.00)
           && fEq(captured.latitude, 3114.51480) && captured.latDir == 'N'
           && fEq(captured.longitude, 12129.55470) && captured.lonDir == 'E'
           && captured.quality == 1 && captured.numSatellites == 8
           && fEq(captured.hdop, 1.2) && fEq(captured.altitude, 50.3)
           && fEq(captured.geoidSep, -8.5) && fEq(captured.dgpsAge, 0.0)
           && captured.dgpsStationId == 0;
}

bool testGgaMultipleTalkerIds() {
    struct Expected {
        const char *frame;
        const char *id;
        int         satellites;
    };
    constexpr Expected cases[] = {
        {BDGGA_FRAME1, "BD", 8 },
        {GPGGA_FRAME,  "GP", 9 },
        {GLGGA_FRAME,  "GL", 6 },
        {GAGGA_FRAME,  "GA", 4 },
        {GNGGA_FRAME,  "GN", 16},
    };
    int             idx   = 0;
    bool            allOk = true;
    Nmea::GgaParser parser(
        [&](const Nmea::Gga &f) {
            if(idx < 5) {
                allOk = allOk && strcmp(f.talkerId, cases[idx].id) == 0
                        && f.numSatellites == cases[idx].satellites;
                idx++;
            }
        },
        nullptr);
    for(auto &c : cases) {
        auto v = toVec(c.frame);
        parser.Parse(v.data(), v.size());
    }
    return allOk && idx == 5;
}

bool testGgaStickyPacket() {
    int             count = 0;
    Nmea::GgaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });
    std::vector<uint8_t> buf;
    for(const char *f : {BDGGA_FRAME1, GPGGA_FRAME, GLGGA_FRAME, GAGGA_FRAME, GNGGA_FRAME}) {
        vecAppend(buf, toVec(f));
    }
    parser.Parse(buf.data(), buf.size());
    return count == 5;
}

bool testGgaSplitPacketChunks() {
    int             count = 0;
    Nmea::GgaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });
    std::vector<uint8_t> buf = toVec(BDGGA_FRAME1);
    vecAppend(buf, toVec(GPGGA_FRAME));
    uint32_t c1 = static_cast<uint32_t>(buf.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(buf.size() * 2 / 3);
    parser.Parse(buf.data(), c1);
    parser.Parse(buf.data() + c1, c2 - c1);
    parser.Parse(buf.data() + c2, static_cast<uint32_t>(buf.size()) - c2);
    return count == 2;
}

// ── GSV 测试 ──────────────────────────────────────────────────────────────────

bool testGsvSingleFrame() {
    Nmea::Gsv       captured{};
    bool            rawOk = false, frameOk = false;
    Nmea::GsvParser parser(
        [&](const Nmea::Gsv &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 6) && (memcmp(data, "$GPGSV", 6) == 0);
        });
    auto v = toVec(GPGSV_FRAME1);
    parser.Parse(v.data(), v.size());
    return rawOk && frameOk
           && strcmp(captured.talkerId, "GP") == 0
           && captured.totalMessages == 2 && captured.messageNumber == 1
           && captured.satellitesInView == 8 && captured.satelliteCount == 4
           && captured.satellites[0].prn == 1 && captured.satellites[0].snr == 46
           && captured.satellites[3].prn == 14 && captured.satellites[3].snr == 45;
}

bool testGsvStickyPacket() {
    int             count = 0;
    Nmea::GsvParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });
    std::vector<uint8_t> buf = toVec(GPGSV_FRAME1);
    vecAppend(buf, toVec(GPGSV_FRAME2));
    parser.Parse(buf.data(), buf.size());
    return count == 2;
}

bool testGsvSplitPacketChunks() {
    int             count = 0;
    Nmea::GsvParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });
    std::vector<uint8_t> buf = toVec(GPGSV_FRAME1);
    vecAppend(buf, toVec(GPGSV_FRAME2));
    uint32_t c1 = static_cast<uint32_t>(buf.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(buf.size() * 2 / 3);
    parser.Parse(buf.data(), c1);
    parser.Parse(buf.data() + c1, c2 - c1);
    parser.Parse(buf.data() + c2, static_cast<uint32_t>(buf.size()) - c2);
    return count == 2;
}

// ── 混合协议测试 ──────────────────────────────────────────────────────────────

bool testMixedProtocol() {
    int                    rawImuCount = 0, ggaCount = 0, gsvCount = 0;
    Novatel::RawImuAParser rawImuParser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawImuCount++;
        });
    Nmea::GgaParser ggaParser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            ggaCount++;
        });
    Nmea::GsvParser gsvParser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            gsvCount++;
        });

    const uint8_t        noise[] = {0xFF, 0x00, '#', 'X', '$', 'Z', 0x01};
    std::vector<uint8_t> stream;
    rawAppend(stream, noise, sizeof(noise));
    vecAppend(stream, toVec(BDGGA_FRAME1));
    vecAppend(stream, toVec(RAWIMU_FRAME1));
    vecAppend(stream, toVec(GPGSV_FRAME1));
    vecAppend(stream, toVec(GPGGA_FRAME));
    vecAppend(stream, toVec(RAWIMU_FRAME2));
    vecAppend(stream, toVec(GPGSV_FRAME2));
    vecAppend(stream, toVec(GNGGA_FRAME));
    rawAppend(stream, noise, sizeof(noise));

    for(uint8_t b : stream) {
        rawImuParser.ParseByteStream(b);
        ggaParser.ParseByteStream(b);
        gsvParser.ParseByteStream(b);
    }
    return rawImuCount == 2 && ggaCount == 3 && gsvCount == 2;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_main ===\n\n";
    // RawImu
    CHECK(testRawImuSingleFrame());
    CHECK(testRawImuStickyPacket());
    CHECK(testRawImuSplitPacketByte());
    CHECK(testRawImuSplitPacketChunks());

    // GGA
    CHECK(testGgaSingleFrameBd());
    CHECK(testGgaMultipleTalkerIds());
    CHECK(testGgaStickyPacket());
    CHECK(testGgaSplitPacketChunks());

    // GSV
    CHECK(testGsvSingleFrame());
    CHECK(testGsvStickyPacket());
    CHECK(testGsvSplitPacketChunks());

    // 混合场景
    CHECK(testMixedProtocol());

    std::cout << "\nAll tests passed.\n";
    return 0;
}
