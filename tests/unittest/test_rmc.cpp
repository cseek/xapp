#include "test_utils.h"
#include "nmea/rmc.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────
// 完整帧：GPRMC，含磁偏角和模式指示符（NMEA 2.3+）
static const char GPRMC_FRAME[] = "$GPRMC,092725.00,A,3114.51480,N,12129.55470,E,0.5,270.3,160426,2.3,W,A*2C\r\n";

// GNRMC：不同 Talker ID
static const char GNRMC_FRAME[] = "$GNRMC,092725.00,A,3114.51480,N,12129.55470,E,1.2,90.0,160426,,,A*73\r\n";

// 定位无效（status = V）
static const char GPRMC_VOID_FRAME[] = "$GPRMC,000000.00,V,,,,,,,010101,,,N*7C\r\n";

// 全空字段（最极端情况）
static const char GPRMC_EMPTY_FRAME[] = "$GPRMC,,,,,,,,,,,,*4B\r\n";

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（GPRMC）：验证 talkerId + 所有字段 + 原始回调前缀
bool testRmcSingleFrame() {
    Nmea::Rmc captured{};
    bool      rawOk = false, frameOk = false;

    Nmea::RmcParser parser(
        [&](const Nmea::Rmc &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 6) && (memcmp(data, "$GPRMC", 6) == 0);
        });

    auto v = toVec(GPRMC_FRAME);
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && strcmp(captured.talkerId, "GP") == 0
           && fEq(captured.utcTime, 92725.00)
           && captured.status == 'A'
           && fEq(captured.latitude, 3114.51480)
           && captured.latDir == 'N'
           && fEq(captured.longitude, 12129.55470)
           && captured.lonDir == 'E'
           && fEq(captured.speedKnots, 0.5)
           && fEq(captured.courseDeg, 270.3)
           && captured.date == 160426
           && fEq(captured.magVariation, 2.3)
           && captured.magVarDir == 'W'
           && captured.modeIndicator == 'A';
}

// 多 Talker ID：GP / GN 各一帧，验证 talkerId 正确提取
bool testRmcMultipleTalkerIds() {
    struct Expected {
        const char *frame;
        const char *id;
        char        status;
    };
    constexpr Expected cases[] = {
        {GPRMC_FRAME, "GP", 'A'},
        {GNRMC_FRAME, "GN", 'A'},
    };
    int             idx   = 0;
    bool            allOk = true;
    Nmea::RmcParser parser(
        [&](const Nmea::Rmc &f) {
            if(idx < 2) {
                allOk = allOk
                        && strcmp(f.talkerId, cases[idx].id) == 0
                        && f.status == cases[idx].status;
                idx++;
            }
        },
        nullptr);
    for(auto &c : cases) {
        auto v = toVec(c.frame);
        parser.Parse(v.data(), v.size());
    }
    return allOk && idx == 2;
}

// status=V（定位无效）：验证 status 字段正确为 'V'，坐标字段为空归零
bool testRmcVoidStatus() {
    Nmea::Rmc       captured{};
    Nmea::RmcParser parser(
        [&](const Nmea::Rmc &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPRMC_VOID_FRAME);
    parser.Parse(v.data(), v.size());

    return captured.status == 'V'
           && fEq(captured.latitude, 0.0)
           && fEq(captured.longitude, 0.0)
           && captured.modeIndicator == 'N';
}

// 全空字段：验证字段索引不错位，数值归零，字符字段为 '\0'
bool testRmcEmptyFields() {
    Nmea::Rmc       captured{};
    Nmea::RmcParser parser(
        [&](const Nmea::Rmc &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GPRMC_EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return strcmp(captured.talkerId, "GP") == 0
           && fEq(captured.utcTime, 0.0)
           && captured.status == '\0'
           && fEq(captured.latitude, 0.0)
           && captured.latDir == '\0'
           && fEq(captured.longitude, 0.0)
           && captured.lonDir == '\0'
           && fEq(captured.speedKnots, 0.0)
           && fEq(captured.courseDeg, 0.0)
           && captured.date == 0
           && fEq(captured.magVariation, 0.0)
           && captured.magVarDir == '\0'
           && captured.modeIndicator == '\0';
}

// 磁偏角字段为空（GNRMC 省略 magVar/magVarDir）
bool testRmcNoMagVariation() {
    Nmea::Rmc       captured{};
    Nmea::RmcParser parser(
        [&](const Nmea::Rmc &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(GNRMC_FRAME);
    parser.Parse(v.data(), v.size());

    return strcmp(captured.talkerId, "GN") == 0
           && fEq(captured.speedKnots, 1.2)
           && fEq(captured.courseDeg, 90.0)
           && fEq(captured.magVariation, 0.0)  // 空字段 → 0
           && captured.magVarDir == '\0'       // 空字段 → '\0'
           && captured.modeIndicator == 'A';
}

// 粘包：三帧直接拼接，期望 3 次回调
bool testRmcStickyPacket() {
    int             count = 0;
    Nmea::RmcParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPRMC_FRAME);
    vecAppend(buf, toVec(GNRMC_FRAME));
    vecAppend(buf, toVec(GPRMC_VOID_FRAME));

    parser.Parse(buf.data(), buf.size());
    return count == 3;
}

// 分包（逐字节）
bool testRmcSplitPacketByte() {
    bool            rawOk = false;
    Nmea::RmcParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });

    for(uint8_t b : toVec(GPRMC_FRAME)) {
        parser.ParseByteStream(b);
    }
    return rawOk;
}

// 分包（多片）：GPRMC + GNRMC 拼为一流，分三片送入
bool testRmcSplitPacketChunks() {
    int             count = 0;
    Nmea::RmcParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(GPRMC_FRAME);
    vecAppend(buf, toVec(GNRMC_FRAME));

    uint32_t c1 = static_cast<uint32_t>(buf.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(buf.size() * 2 / 3);
    parser.Parse(buf.data(), c1);
    parser.Parse(buf.data() + c1, c2 - c1);
    parser.Parse(buf.data() + c2, static_cast<uint32_t>(buf.size()) - c2);
    return count == 2;
}

// 垃圾数据：假头 $GPRMX（最后字母不是 C）不会被误识别
bool testRmcWithGarbage() {
    int             count = 0;
    Nmea::RmcParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    // '$','G','P','R','M',<非'C'> → 状态机回退
    const uint8_t        garbage[] = {0xFF, 0x01, '$', 'G', 'P', 'R', 'M', 0x01};
    std::vector<uint8_t> stream;
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GPRMC_FRAME));
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(GNRMC_FRAME));

    parser.Parse(stream.data(), static_cast<uint32_t>(stream.size()));
    return count == 2;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_rmc ===\n\n";
    CHECK(testRmcSingleFrame());
    CHECK(testRmcMultipleTalkerIds());
    CHECK(testRmcVoidStatus());
    CHECK(testRmcEmptyFields());
    CHECK(testRmcNoMagVariation());
    CHECK(testRmcStickyPacket());
    CHECK(testRmcSplitPacketByte());
    CHECK(testRmcSplitPacketChunks());
    CHECK(testRmcWithGarbage());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
