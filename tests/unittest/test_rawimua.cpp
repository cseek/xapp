#include "test_utils.h"
#include "novatel/rawimua.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────
static const char RAWIMU_FRAME1[] = "#RAWIMUA,COM1,0,92.5,FINESTEERING,2344,405000.000,02000020,e5e1,32;"
                                    "2344,404999.998,00000000,-1234567,2345678,-345678,98765,-87654,123456*3599623f\r\n";

static const char RAWIMU_FRAME2[] = "#RAWIMUA,COM1,1,91.0,FINESTEERING,2344,405000.010,02000020,e5e1,32;"
                                    "2344,405000.010,00000001,1000000,-2000000,300000,-10000,20000,-30000*029cff6a\r\n";

// 非零且非平凡的 imuStatus（测试十六进制解析）
static const char RAWIMU_FRAME3[] = "#RAWIMUA,COM1,2,90.0,FINESTEERING,2344,405000.020,02000020,e5e1,32;"
                                    "2344,405000.020,0A1B2C3D,0,0,0,0,0,0*e9ffdbc5\r\n";

// body 全空字段（week/sec/status/6轴数据全部为空）
static const char RAWIMU_EMPTY_FRAME[] = "#RAWIMUA,COM1,3,89.0,FINESTEERING,2344,405000.030,02000020,e5e1,32;"
                                         ",,,,,,,,*659ced3e\r\n";

// body 部分空字段（部分有效、部分为空）
static const char RAWIMU_PARTIAL_FRAME[] = "#RAWIMUA,COM1,4,88.0,FINESTEERING,2344,405000.040,02000020,e5e1,32;"
                                           "2344,405000.040,,,,,,20000,-30000*e49e69c0\r\n";

// 错误 CRC：raw 回调会触发，但 frame 回调不应触发
static const char RAWIMU_BAD_CRC_FRAME[] = "#RAWIMUA,COM1,0,92.5,FINESTEERING,2344,405000.000,02000020,e5e1,32;"
                                           "2344,404999.998,00000000,-1234567,2345678,-345678,98765,-87654,123456*35996230\r\n";

// A 后不是逗号，应被状态机直接拒绝
static const char RAWIMU_A_NOT_COMMA_FRAME[] = "#RAWIMUAX,COM1,0,92.5,FINESTEERING,2344,405000.000,02000020,e5e1,32;"
                                               "2344,404999.998,00000000,-1234567,2345678,-345678,98765,-87654,123456*deadbeef\r\n";

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（FRAME1）：验证原始回调前缀 & 所有结构体字段
bool testRawImuSingleFrame() {
    Novatel::RawImuA captured{};
    bool             rawOk = false, frameOk = false;

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
           && captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 404999.998)
           && captured.imuStatus == 0x00000000u
           && captured.zAccel == -1234567
           && captured.negYAccel == 2345678
           && captured.xAccel == -345678
           && captured.zGyro == 98765
           && captured.negYGyro == -87654
           && captured.xGyro == 123456;
}

// 单帧（FRAME2）：验证正值字段与 imuStatus=0x00000001
bool testRawImuFrame2Fields() {
    Novatel::RawImuA       captured{};
    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(RAWIMU_FRAME2);
    parser.Parse(v.data(), v.size());

    return captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 405000.010)
           && captured.imuStatus == 0x00000001u
           && captured.zAccel == 1000000
           && captured.negYAccel == -2000000
           && captured.xAccel == 300000
           && captured.zGyro == -10000
           && captured.negYGyro == 20000
           && captured.xGyro == -30000;
}

// imuStatus 十六进制解析：0A1B2C3D
bool testRawImuImuStatusHex() {
    Novatel::RawImuA       captured{};
    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(RAWIMU_FRAME3);
    parser.Parse(v.data(), v.size());

    return captured.imuStatus == 0x0A1B2C3Du;
}

// body 全空字段：数值字段应保持默认值 0（不会发生字段错位）
bool testRawImuEmptyFields() {
    Novatel::RawImuA       captured{};
    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(RAWIMU_EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return captured.gpsWeek == 0
           && fEq(captured.gpsSeconds, 0.0)
           && captured.imuStatus == 0u
           && captured.zAccel == 0
           && captured.negYAccel == 0
           && captured.xAccel == 0
           && captured.zGyro == 0
           && captured.negYGyro == 0
           && captured.xGyro == 0;
}

// body 部分空字段：week/sec + 最后两个 gyro 有值，其余为空
bool testRawImuPartialEmptyFields() {
    Novatel::RawImuA       captured{};
    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &f) {
            captured = f;
        },
        nullptr);

    auto v = toVec(RAWIMU_PARTIAL_FRAME);
    parser.Parse(v.data(), v.size());

    return captured.gpsWeek == 2344
           && fEq(captured.gpsSeconds, 405000.040)
           && captured.imuStatus == 0u  // 空字段
           && captured.zAccel == 0      // 空字段
           && captured.negYAccel == 0   // 空字段
           && captured.xAccel == 0      // 空字段
           && captured.zGyro == 0       // 空字段
           && captured.negYGyro == 20000
           && captured.xGyro == -30000;
}

// 多帧帧回调：两帧依次送入，验证两次回调数据各自正确
bool testRawImuMultipleFrames() {
    Novatel::RawImuA       frames[2]{};
    int                    idx = 0;
    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &f) {
            if(idx < 2) {
                frames[idx++] = f;
            }
        },
        nullptr);

    auto v1 = toVec(RAWIMU_FRAME1);
    auto v2 = toVec(RAWIMU_FRAME2);
    parser.Parse(v1.data(), v1.size());
    parser.Parse(v2.data(), v2.size());

    return idx == 2
           && frames[0].zAccel == -1234567
           && frames[0].imuStatus == 0x00000000u
           && frames[1].zAccel == 1000000
           && frames[1].imuStatus == 0x00000001u;
}

// 粘包（原始回调计数）：两帧 + 中间噪声，期望 2 次原始回调
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

// 粘包（帧回调内容）：三帧拼接，验证每帧字段互不干扰
bool testRawImuStickyPacketFrameData() {
    Novatel::RawImuA       frames[3]{};
    int                    idx = 0;
    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &f) {
            if(idx < 3) {
                frames[idx++] = f;
            }
        },
        nullptr);

    std::vector<uint8_t> buf = toVec(RAWIMU_FRAME1);
    vecAppend(buf, toVec(RAWIMU_FRAME2));
    vecAppend(buf, toVec(RAWIMU_FRAME3));
    parser.Parse(buf.data(), buf.size());

    return idx == 3
           && frames[0].zAccel == -1234567
           && frames[1].zAccel == 1000000
           && frames[2].imuStatus == 0x0A1B2C3Du
           && frames[2].zAccel == 0;
}

// 垃圾数据：假头 #RAWIMUB（最后字母不是 A） + 部分头截断，不触发误识别
bool testRawImuWithGarbage() {
    int                    count = 0;
    Novatel::RawImuAParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    // '#','R','A','W','I','M','U',<非'A'> → 状态机回退
    const uint8_t        garbage[] = {0xFF, 0x01, '#', 'R', 'A', 'W', 'I', 'M', 'U', 0x42};
    std::vector<uint8_t> stream;
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(RAWIMU_FRAME1));
    rawAppend(stream, garbage, sizeof(garbage));
    vecAppend(stream, toVec(RAWIMU_FRAME2));

    parser.Parse(stream.data(), static_cast<uint32_t>(stream.size()));
    return count == 2;
}

// 分包（逐字节）
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

// 分包（三段切割）
bool testRawImuSplitPacketChunks() {
    int                    count = 0;
    Novatel::RawImuAParser parser(
        nullptr,
        [&](uint8_t *, uint32_t) {
            count++;
        });

    std::vector<uint8_t> buf = toVec(RAWIMU_FRAME1);
    vecAppend(buf, toVec(RAWIMU_FRAME2));
    uint32_t c1 = static_cast<uint32_t>(buf.size() / 3);
    uint32_t c2 = static_cast<uint32_t>(buf.size() * 2 / 3);
    parser.Parse(buf.data(), c1);
    parser.Parse(buf.data() + c1, c2 - c1);
    parser.Parse(buf.data() + c2, static_cast<uint32_t>(buf.size()) - c2);
    return count == 2;
}

// CRC 错误：rawCallback 触发，frameCallback 不触发
bool testRawImuBadCrcRejectFrame() {
    bool rawOk   = false;
    bool frameOk = false;

    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &) {
            frameOk = true;
        },
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });

    auto v = toVec(RAWIMU_BAD_CRC_FRAME);
    parser.Parse(v.data(), v.size());

    return rawOk && !frameOk;
}

// A 后非逗号：raw/frame 回调都不应触发
bool testRawImuRequireCommaAfterA() {
    bool rawOk   = false;
    bool frameOk = false;

    Novatel::RawImuAParser parser(
        [&](const Novatel::RawImuA &) {
            frameOk = true;
        },
        [&](uint8_t *, uint32_t) {
            rawOk = true;
        });

    auto v = toVec(RAWIMU_A_NOT_COMMA_FRAME);
    parser.Parse(v.data(), v.size());

    return !rawOk && !frameOk;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    std::cout << "\n=== test_rawimua ===\n\n";
    CHECK(testRawImuSingleFrame());
    CHECK(testRawImuFrame2Fields());
    CHECK(testRawImuImuStatusHex());
    CHECK(testRawImuEmptyFields());
    CHECK(testRawImuPartialEmptyFields());
    CHECK(testRawImuMultipleFrames());
    CHECK(testRawImuStickyPacket());
    CHECK(testRawImuStickyPacketFrameData());
    CHECK(testRawImuWithGarbage());
    CHECK(testRawImuSplitPacketByte());
    CHECK(testRawImuSplitPacketChunks());
    CHECK(testRawImuBadCrcRejectFrame());
    CHECK(testRawImuRequireCommaAfterA());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
