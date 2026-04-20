#include "test_utils.h"
#include "novatel/insstdevsa.h"

// ── 测试帧 ───────────────────────────────────────────────────────────────────

static const char INSSTDEVSA_FRAME1[] = "%INSSTDEVSA,2209,491032.000;0.1812,0.1812,0.1805,0.0018,0.0018,0.0017,"
                                        "0.0291,0.0291,0.0575,13000045,0,0,7fd1bf,0*b6d40807\r\n";

static const char INSSTDEVSA_FRAME2[] = "%INSSTDEVSA,2344,405000.000;0.5000,0.4000,0.3000,0.0200,0.0100,0.0050,"
                                        "0.1000,0.0900,0.1200,0F000001,3,1,00ABCDEF,2*e2564471\r\n";

static const char INSSTDEVSA_FRAME3[] = "%INSSTDEVSA,2344,405001.000;0,0,0,0,0,0,0,0,0,00000000,0,0,00000000,0*a7a9da9e\r\n";

static const char INSSTDEVSA_EMPTY_FRAME[] = "%INSSTDEVSA,2344,405002.000;0.1,,0.3,,,0.6,0.7,0.8,0.9,01020304,1,0,00000001,0*c593a889\r\n";

static std::vector<uint8_t> toVecISD(const char *s) {
    const uint8_t *p = reinterpret_cast<const uint8_t *>(s);
    return {p, p + strlen(s)};
}

// ── 测试函数 ─────────────────────────────────────────────────────────────────

bool testInsStdevsSingleFrame() {
    Novatel::InsStdevsA captured{};
    bool                rawOk = false, frameOk = false;

    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &f) {
            frameOk  = true;
            captured = f;
        },
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 11) && (memcmp(data, "%INSSTDEVSA", 11) == 0);
        });

    auto v = toVecISD(INSSTDEVSA_FRAME1);
    parser.Parse(v.data(), v.size());

    return rawOk && frameOk
           && captured.gpsWeek == 2209
           && fEq(captured.gpsSeconds, 491032.0, 1e-3)
           && fEq(captured.latSigma, 0.1812f, 1e-4f)
           && fEq(captured.lonSigma, 0.1812f, 1e-4f)
           && fEq(captured.hgtSigma, 0.1805f, 1e-4f)
           && fEq(captured.northVelSigma, 0.0018f, 1e-4f)
           && fEq(captured.eastVelSigma, 0.0018f, 1e-4f)
           && fEq(captured.upVelSigma, 0.0017f, 1e-4f)
           && fEq(captured.rollSigma, 0.0291f, 1e-4f)
           && fEq(captured.pitchSigma, 0.0291f, 1e-4f)
           && fEq(captured.azimuthSigma, 0.0575f, 1e-4f)
           && captured.extSolStat == 0x13000045u
           && captured.timeSinceUpdate == 0
           && captured.reserved1 == 0
           && captured.reserved2 == 0x7fd1bfu
           && captured.reserved3 == 0;
}

bool testInsStdevsFrame2() {
    Novatel::InsStdevsA       captured{};
    bool                      frameOk = false;
    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecISD(INSSTDEVSA_FRAME2);
    parser.Parse(v.data(), v.size());

    return frameOk
           && captured.gpsWeek == 2344
           && fEq(captured.latSigma, 0.5f, 1e-4f)
           && fEq(captured.eastVelSigma, 0.01f, 1e-4f)
           && fEq(captured.azimuthSigma, 0.12f, 1e-4f)
           && captured.extSolStat == 0x0F000001u
           && captured.timeSinceUpdate == 3
           && captured.reserved1 == 1
           && captured.reserved2 == 0x00ABCDEFu
           && captured.reserved3 == 2;
}

bool testInsStdevsZeroFrame() {
    Novatel::InsStdevsA captured{};
    bool                frameOk = false;

    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecISD(INSSTDEVSA_FRAME3);
    parser.Parse(v.data(), v.size());

    return frameOk
           && fEq(captured.latSigma, 0.0f, 1e-9f)
           && fEq(captured.azimuthSigma, 0.0f, 1e-9f)
           && captured.extSolStat == 0u
           && captured.reserved2 == 0u;
}

bool testInsStdevsByteByByte() {
    Novatel::InsStdevsA captured{};
    bool                frameOk = false;

    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    for(size_t i = 0; i < strlen(INSSTDEVSA_FRAME1); i++) {
        parser.ParseByteStream(static_cast<uint8_t>(INSSTDEVSA_FRAME1[i]));
    }

    return frameOk
           && captured.gpsWeek == 2209
           && fEq(captured.latSigma, 0.1812f, 1e-4f);
}

bool testInsStdevsMultipleFrames() {
    int                 callCount = 0;
    Novatel::InsStdevsA frames[2]{};

    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &f) {
            if(callCount < 2) {
                frames[callCount] = f;
            }
            callCount++;
        },
        nullptr);

    auto v1 = toVecISD(INSSTDEVSA_FRAME1);
    auto v2 = toVecISD(INSSTDEVSA_FRAME2);
    parser.Parse(v1.data(), v1.size());
    parser.Parse(v2.data(), v2.size());

    return callCount == 2
           && frames[0].gpsWeek == 2209
           && frames[1].gpsWeek == 2344;
}

bool testInsStdevsWithGarbage() {
    Novatel::InsStdevsA captured{};
    bool                frameOk = false;

    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    const char garbage[] = "RANDOM_NOISE#NOT_INS\r\n";
    parser.Parse(reinterpret_cast<const uint8_t *>(garbage), strlen(garbage));

    auto v = toVecISD(INSSTDEVSA_FRAME1);
    parser.Parse(v.data(), v.size());

    return frameOk && captured.gpsWeek == 2209;
}

bool testInsStdevsEmptyFields() {
    Novatel::InsStdevsA captured{};
    bool                frameOk = false;

    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &f) {
            frameOk  = true;
            captured = f;
        },
        nullptr);

    auto v = toVecISD(INSSTDEVSA_EMPTY_FRAME);
    parser.Parse(v.data(), v.size());

    return frameOk
           && fEq(captured.latSigma, 0.1f, 1e-4f)
           && fEq(captured.lonSigma, 0.0f, 1e-9f)
           && fEq(captured.northVelSigma, 0.0f, 1e-9f)
           && fEq(captured.eastVelSigma, 0.0f, 1e-9f)
           && fEq(captured.upVelSigma, 0.6f, 1e-4f)
           && captured.extSolStat == 0x01020304u;
}

bool testInsStdevsRawCallbackOnly() {
    bool                      rawOk = false;
    Novatel::InsStdevsAParser parser(
        nullptr,
        [&](uint8_t *data, uint32_t len) {
            rawOk = (len > 0) && (data[0] == '%');
        });

    auto v = toVecISD(INSSTDEVSA_FRAME1);
    parser.Parse(v.data(), v.size());

    return rawOk;
}

bool testInsStdevsStickyPacket() {
    int callCount = 0;

    Novatel::InsStdevsAParser parser(
        [&](const Novatel::InsStdevsA &) {
            callCount++;
        },
        nullptr);

    std::vector<uint8_t> combined;
    for(char c : std::string(INSSTDEVSA_FRAME1)) {
        combined.push_back(static_cast<uint8_t>(c));
    }
    for(char c : std::string(INSSTDEVSA_FRAME2)) {
        combined.push_back(static_cast<uint8_t>(c));
    }
    parser.Parse(combined.data(), combined.size());

    return callCount == 2;
}

int main() {
    std::cout << "\n=== test_insstdevsa ===\n\n";
    CHECK(testInsStdevsSingleFrame());
    CHECK(testInsStdevsFrame2());
    CHECK(testInsStdevsZeroFrame());
    CHECK(testInsStdevsByteByByte());
    CHECK(testInsStdevsMultipleFrames());
    CHECK(testInsStdevsWithGarbage());
    CHECK(testInsStdevsEmptyFields());
    CHECK(testInsStdevsRawCallbackOnly());
    CHECK(testInsStdevsStickyPacket());
    std::cout << "\nAll tests passed.\n";
    return 0;
}
