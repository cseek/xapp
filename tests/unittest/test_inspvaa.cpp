#include "test_utils.h"
#include "novatel/inspvaa.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────

// 官方文档示例帧
static const char INSPVAA_FRAME1[] = "#INSPVAA,USB1,0,67.5,FINESTEERING,2209,490558.000,02000020,18bc,16809;"
                                     "2209,490558.000000000,51.15043714042,-114.03067871718,1080.3548,"
                                     "0.0051,-0.0014,-0.0012,-0.296402993,0.311887972,157.992156267,"
                                     "INS_SOLUTION_GOOD*cc698020\r\n";

// 第二帧：不同位置/速度/姿态
static const char INSPVAA_FRAME2[] = "#INSPVAA,COM1,1,72.0,FINESTEERING,2344,405000.000,02000020,18bc,16809;"
                                     "2344,405000.000000000,31.22819476000,121.48917843000,20.1230,"
                                     "1.2345,-0.5678,0.1234,2.345678900,-1.234567890,270.123456789,"
                                     "INS_ALIGNMENT_COMPLETE*22ab1e35\r\n";

// INS 未对准帧
static const char INSPVAA_FRAME3[] = "#INSPVAA,COM1,2,80.0,FINESTEERING,2344,405001.000,02000020,18bc,16809;"
                                     "2344,405001.000000000,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,"
                                     "INS_INACTIVE*d284da0a\r\n";

// 空字段帧（northVel/eastVel 为空）
static const char INSPVAA_EMPTY_FRAME[] = "#INSPVAA,COM1,3,70.0,FINESTEERING,2344,405002.000,02000020,18bc,16809;"
                                          "2344,405002.000000000,31.0,121.0,10.0,,,0.5,1.0,-0.5,90.0,"
                                          "INS_SOLUTION_GOOD*3cbc351a\r\n";

// ── 测试函数 ─────────────────────────────────────────────────────────────────

// 单帧（FRAME1）：原始回调前缀校验 & 所有字段
bool testINSPVAASingleFrame() {
    Novatel::InsPvaA captured{};
    bool             rawOk = false, frameOk = false;

    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 8) && (memcmp(data, "#INSPVAA", 8) == 0);
        });

    auto v = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1),
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1) + strlen(INSPVAA_FRAME1));
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && captured.gpsWeek == 2209
           && fEq(captured.gpsSeconds, 490558.0, 1e-3)
           && fEq(captured.lat, 51.15043714042, 1e-8)
           && fEq(captured.lon, -114.03067871718, 1e-8)
           && fEq(captured.height, 1080.3548, 1e-3)
           && fEq(captured.northVel, 0.0051, 1e-5)
           && fEq(captured.eastVel, -0.0014, 1e-5)
           && fEq(captured.upVel, -0.0012, 1e-5)
           && fEq(captured.roll, -0.296402993, 1e-7)
           && fEq(captured.pitch, 0.311887972, 1e-7)
           && fEq(captured.azimuth, 157.992156267, 1e-6)
           && strcmp(captured.status, "INS_SOLUTION_GOOD") == 0;
}

// 第二帧：位置/速度/姿态均不同
bool testINSPVAAFrame2() {
    Novatel::InsPvaA captured{};
    bool             frameOk = false;

    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME2),
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME2) + strlen(INSPVAA_FRAME2));
    parser.Parse(v.data(), v.size());

    return frameOk
           && captured.gpsWeek == 2344
           && fEq(captured.lat, 31.22819476, 1e-7)
           && fEq(captured.lon, 121.48917843, 1e-7)
           && fEq(captured.northVel, 1.2345, 1e-4)
           && fEq(captured.eastVel, -0.5678, 1e-4)
           && fEq(captured.azimuth, 270.123456789, 1e-6)
           && strcmp(captured.status, "INS_ALIGNMENT_COMPLETE") == 0;
}

// INS_INACTIVE 帧：frameCallback 触发，数值全零
bool testINSPVAAInactive() {
    Novatel::InsPvaA captured{};
    bool             frameOk = false;

    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME3),
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME3) + strlen(INSPVAA_FRAME3));
    parser.Parse(v.data(), v.size());

    return frameOk
           && strcmp(captured.status, "INS_INACTIVE") == 0
           && fEq(captured.lat, 0.0, 1e-9)
           && fEq(captured.azimuth, 0.0, 1e-9);
}

// 逐字节解析：状态机流式能力
bool testINSPVAAByteByByte() {
    Novatel::InsPvaA captured{};
    bool             frameOk = false;

    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    for(size_t i = 0; i < strlen(INSPVAA_FRAME1); i++) {
        parser.ParseByteStream(static_cast<uint8_t>(INSPVAA_FRAME1[i]));
    }

    return frameOk
           && fEq(captured.lat, 51.15043714042, 1e-8)
           && strcmp(captured.status, "INS_SOLUTION_GOOD") == 0;
}

// 连续两帧：各自独立触发
bool testINSPVAAMultipleFrames() {
    int              callCount = 0;
    Novatel::InsPvaA frames[2]{};

    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &f) {
            if(callCount < 2) {
                frames[callCount] = f;
            }
            callCount++;
        },
        nullptr);

    auto v1 = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1),
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1) + strlen(INSPVAA_FRAME1));
    auto v2 = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME2),
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME2) + strlen(INSPVAA_FRAME2));
    parser.Parse(v1.data(), v1.size());
    parser.Parse(v2.data(), v2.size());

    return callCount == 2
           && strcmp(frames[0].status, "INS_SOLUTION_GOOD") == 0
           && strcmp(frames[1].status, "INS_ALIGNMENT_COMPLETE") == 0;
}

// 垃圾数据 + 有效帧：状态机正确过滤
bool testINSPVAAWithGarbage() {
    Novatel::InsPvaA captured{};
    bool             frameOk = false;

    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    const char     garbage[] = "#INS##INSPVA_GARBAGE\r\n";
    const uint8_t *gj        = reinterpret_cast<const uint8_t *>(garbage);
    parser.Parse(gj, strlen(garbage));

    auto v = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1),
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1) + strlen(INSPVAA_FRAME1));
    parser.Parse(v.data(), v.size());

    return frameOk && fEq(captured.lat, 51.15043714042, 1e-8);
}

// 空字段：northVel/eastVel 为空应解析为 0
bool testINSPVAAEmptyFields() {
    Novatel::InsPvaA captured{};
    bool             frameOk = false;

    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_EMPTY_FRAME),
        reinterpret_cast<const uint8_t *>(INSPVAA_EMPTY_FRAME) + strlen(INSPVAA_EMPTY_FRAME));
    parser.Parse(v.data(), v.size());

    return frameOk
           && fEq(captured.lat, 31.0, 1e-6)
           && fEq(captured.northVel, 0.0, 1e-9)  // empty → 0
           && fEq(captured.eastVel, 0.0, 1e-9)   // empty → 0
           && fEq(captured.upVel, 0.5, 1e-5)
           && fEq(captured.azimuth, 90.0, 1e-6)
           && strcmp(captured.status, "INS_SOLUTION_GOOD") == 0;
}

// 仅原始回调，无 frame 回调：不崩溃
bool testINSPVAARawCallbackOnly() {
    bool                   rawOk = false;
    Novatel::InsPvaAParser parser(
        nullptr,
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 0);
        });

    auto v = std::vector<uint8_t>(
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1),
        reinterpret_cast<const uint8_t *>(INSPVAA_FRAME1) + strlen(INSPVAA_FRAME1));
    parser.Parse(v.data(), v.size());
    return rawOk;
}

// 粘包：两帧连续送入同一字节流
bool testINSPVAAStickyPacket() {
    int                    callCount = 0;
    Novatel::InsPvaAParser parser(
        [&](const Novatel::InsPvaA &) {
            callCount++;
        },
        nullptr);

    std::vector<uint8_t> combined;
    for(char c : std::string(INSPVAA_FRAME1)) {
        combined.push_back(c);
    }
    for(char c : std::string(INSPVAA_FRAME2)) {
        combined.push_back(c);
    }
    parser.Parse(combined.data(), combined.size());

    return callCount == 2;
}

// ── main ─────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "\n=== test_inspvaa ===\n\n";
    CHECK(testINSPVAASingleFrame());
    CHECK(testINSPVAAFrame2());
    CHECK(testINSPVAAInactive());
    CHECK(testINSPVAAByteByByte());
    CHECK(testINSPVAAMultipleFrames());
    CHECK(testINSPVAAWithGarbage());
    CHECK(testINSPVAAEmptyFields());
    CHECK(testINSPVAARawCallbackOnly());
    CHECK(testINSPVAAStickyPacket());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
