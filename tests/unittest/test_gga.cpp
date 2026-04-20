#include "test_utils.h"
#include "nmea/gga.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────
static const char BDGGA_FRAME1[] = "$BDGGA,092725.00,3114.51480,N,12129.55470,E,1,08,1.2,50.3,M,-8.5,M,0.0,0000*70\r\n";
static const char BDGGA_FRAME2[] = "$BDGGA,093000.00,3114.52000,N,12129.56000,E,2,12,0.8,51.0,M,-8.5,M,1.5,0002*79\r\n";
static const char GPGGA_FRAME[]  = "$GPGGA,092725.00,3114.51480,N,12129.55470,E,1,09,1.0,50.3,M,-8.5,M,0.0,0000*62\r\n";
static const char GLGGA_FRAME[]  = "$GLGGA,092725.00,3114.51480,N,12129.55470,E,1,06,1.5,50.3,M,-8.5,M,0.0,0000*74\r\n";
static const char GAGGA_FRAME[]  = "$GAGGA,092725.00,3114.51480,N,12129.55470,E,1,04,1.8,50.3,M,-8.5,M,0.0,0000*76\r\n";
static const char GNGGA_FRAME[]  = "$GNGGA,092725.00,3114.51480,N,12129.55470,E,2,16,0.6,50.3,M,-8.5,M,1.2,0001*74\r\n";

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（BDGGA）：验证 talkerId + 所有字段 + 原始回调前缀
bool testGgaSingleFrameBd() {
    Nmea::Gga captured{};
    bool      rawOk = false, frameOk = false;

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
           && fEq(captured.latitude, 3114.51480)
           && captured.latDir == 'N'
           && fEq(captured.longitude, 12129.55470)
           && captured.lonDir == 'E'
           && captured.quality == 1
           && captured.numSatellites == 8
           && fEq(captured.hdop, 1.2)
           && fEq(captured.altitude, 50.3)
           && fEq(captured.geoidSep, -8.5)
           && fEq(captured.dgpsAge, 0.0)
           && captured.dgpsStationId == 0;
}

// 多 Talker ID：同一个 GgaParser 依次接收 BD/GP/GL/GA/GN 五种帧，
// 验证每帧的 talkerId 与 satellite 数均被正确提取
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
                allOk = allOk
                        && strcmp(f.talkerId, cases[idx].id) == 0
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

// 粘包：五种 Talker ID 帧直接拼接，期望触发 5 次回调
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

// 分包（逐字节）
bool testGgaSplitPacketByte() {
    bool            rawOk = false;
    Nmea::GgaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });

    for(uint8_t b : toVec(BDGGA_FRAME1)) {
        parser.ParseByteStream(b);
    }
    return rawOk;
}

// 分包（多帧多片）：BDGGA1+GPGGA 拼为一个流，分三片送入
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

// 垃圾数据：帧前后插入含假头的噪声，仍能识别有效帧
bool testGgaWithGarbage() {
    int             count = 0;
    Nmea::GgaParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    // '$','G','P','G','G',<非'A'> 让状态机行进到 SENTENCE3 后 reset
    const uint8_t        garbage[] = {0xFF, 0x01, '$', 'G', 'P', 'G', 'G', 0x01};
    std::vector<uint8_t> stream;
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(BDGGA_FRAME1));
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GPGGA_FRAME));

    parser.Parse(stream.data(), static_cast<uint32_t>(stream.size()));
    return count == 2;
}

// 空字段：所有数据字段均为空（定位无效时设备可能发出此类帧）
// 验证字段索引不错位，数值字段归零，字符字段为 '\0'
bool testGgaEmptyFields() {
    // $GPGGA,,,,,,,,,,,,,,*AA\r\n  — 全空字段
    static const char EMPTY_FRAME[] = "$GPGGA,,,,,,,,,,,,,,*56\r\n";

    Nmea::Gga       captured{};
    bool            frameOk = false;
    Nmea::GgaParser parser(
        [&](const Nmea::Gga &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVec(EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.talkerId, "GP") == 0
           && fEq(captured.utcTime, 0.0)
           && fEq(captured.latitude, 0.0)
           && captured.latDir == '\0'
           && fEq(captured.longitude, 0.0)
           && captured.lonDir == '\0'
           && captured.quality == 0
           && captured.numSatellites == 0
           && fEq(captured.hdop, 0.0)
           && fEq(captured.altitude, 0.0)
           && fEq(captured.geoidSep, 0.0)
           && fEq(captured.dgpsAge, 0.0)
           && captured.dgpsStationId == 0;
}

// 部分字段为空：utcTime 和 quality 有值，其余为空
// 验证非空字段仍被正确解析，字段索引未被空字段打乱
bool testGgaPartialEmptyFields() {
    // field1=utcTime, field6=quality 有值，其余全空
    static const char PARTIAL_FRAME[] = "$GPGGA,092725.00,,,,,,8,,50.3,M,,,*32\r\n";

    Nmea::Gga       captured{};
    bool            frameOk = false;
    Nmea::GgaParser parser(
        [&](const Nmea::Gga &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVec(PARTIAL_FRAME);
    parser.Parse(v.data(), v.size());

    return frameOk
           && fEq(captured.utcTime, 92725.00)
           && fEq(captured.latitude, 0.0)    // 空字段 → 0
           && captured.latDir == '\0'        // 空字段 → '\0'
           && captured.quality == 0          // field6 空 → 0
           && captured.numSatellites == 8    // field7 = 8
           && fEq(captured.altitude, 50.3);  // field9 = 50.3
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_gga ===\n\n";
    CHECK(testGgaSingleFrameBd());
    CHECK(testGgaMultipleTalkerIds());
    CHECK(testGgaStickyPacket());
    CHECK(testGgaSplitPacketByte());
    CHECK(testGgaSplitPacketChunks());
    CHECK(testGgaWithGarbage());
    CHECK(testGgaEmptyFields());
    CHECK(testGgaPartialEmptyFields());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
