// pio test -e native -f test_mt6701_decode
#include <unity.h>

#include "comm/GcodeParser.h"
#include "drivers/Mt6701Decode.h"

using namespace hin;
using namespace hin::mt6701;

void setUp() {}
void tearDown() {}

void test_decode_angle() {
    TEST_ASSERT_EQUAL_UINT16(0, decodeAngle(0x00, 0x00));
    TEST_ASSERT_EQUAL_UINT16(16383, decodeAngle(0xFF, 0xFC));
    TEST_ASSERT_EQUAL_UINT16(8192, decodeAngle(0x80, 0x00));
    TEST_ASSERT_EQUAL_UINT16(1, decodeAngle(0x00, 0x04));
    TEST_ASSERT_EQUAL_UINT16(1, decodeAngle(0x00, 0x07));  // 2 bit thấp bị bỏ
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 180.0f, ticksToDeg(8192));
}

void test_circular_delta_wraps() {
    TEST_ASSERT_EQUAL_INT32(10, circularDelta(110, 100));
    TEST_ASSERT_EQUAL_INT32(20, circularDelta(10, 16374));   // qua 0 chiều dương
    TEST_ASSERT_EQUAL_INT32(-20, circularDelta(16374, 10));  // qua 0 chiều âm
}

void test_turn_counter_multi_turn() {
    TurnCounter tc;
    tc.reset(16000);
    // Quay thuận 3 vòng, bước 1000 tick
    int32_t raw = 16000;
    for (int i = 0; i < 3 * 16384 / 1000; ++i) {
        raw = (raw + 1000) % kTicksPerRev;
        tc.update(static_cast<uint16_t>(raw));
    }
    TEST_ASSERT_EQUAL_INT32(16000 + (3 * 16384 / 1000) * 1000, tc.totalTicks());
    // Quay ngược về
    for (int i = 0; i < 3 * 16384 / 1000; ++i) {
        raw = (raw - 1000 + kTicksPerRev) % kTicksPerRev;
        tc.update(static_cast<uint16_t>(raw));
    }
    TEST_ASSERT_EQUAL_INT32(16000, tc.totalTicks());
    tc.zero();
    TEST_ASSERT_EQUAL_INT32(0, tc.totalTicks());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, tc.turns());
}

void test_jump_filter() {
    JumpFilter f;
    f.setMaxStep(100);
    TEST_ASSERT_TRUE(f.feed(1000));    // mẫu đầu luôn nhận
    TEST_ASSERT_TRUE(f.feed(1080));
    TEST_ASSERT_FALSE(f.feed(5000));   // nhảy vô lý → bỏ
    TEST_ASSERT_EQUAL_UINT16(1080, f.last());
    TEST_ASSERT_TRUE(f.feed(1000));
    f.reset();
    TEST_ASSERT_TRUE(f.feed(9000));
    f.feed(16350);                     // bị loại (cách 9000 quá xa)
    TEST_ASSERT_EQUAL_UINT16(9000, f.last());
}

void test_jump_filter_across_zero() {
    JumpFilter f;
    f.setMaxStep(50);
    TEST_ASSERT_TRUE(f.feed(16370));
    TEST_ASSERT_TRUE(f.feed(5));  // qua 0: delta = 19
}

// Parser G-code (thuần C++) — đặt chung bộ test giải mã
void test_gcode_parse() {
    GcodeLine g;
    TEST_ASSERT_EQUAL(0, static_cast<int>(parseGcode("G0 J1=100 J2=45 J3=-30 J4=0 F50", g)));
    TEST_ASSERT_EQUAL_CHAR('G', g.letter);
    TEST_ASSERT_EQUAL_INT(0, g.code);
    TEST_ASSERT_EQUAL_UINT8(0x0F, g.jointMask);
    TEST_ASSERT_EQUAL_FLOAT(-30.0f, g.joints[2]);
    TEST_ASSERT_TRUE(g.has(GcodeLine::kF));

    TEST_ASSERT_EQUAL(0, static_cast<int>(parseGcode("g1 x200 Y50 Z40 A0 ; chu thich", g)));
    TEST_ASSERT_EQUAL_INT(1, g.code);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, g.x);
    TEST_ASSERT_TRUE(g.has(GcodeLine::kA));
    TEST_ASSERT_FALSE(g.has(GcodeLine::kF));

    TEST_ASSERT_EQUAL(0, static_cast<int>(parseGcode("M50 L", g)));
    TEST_ASSERT_EQUAL_CHAR('L', g.flag);

    TEST_ASSERT_EQUAL(0, static_cast<int>(parseGcode("   ", g)));
    TEST_ASSERT_EQUAL_CHAR(0, g.letter);

    TEST_ASSERT_NOT_EQUAL(0, static_cast<int>(parseGcode("X10", g)));
    TEST_ASSERT_NOT_EQUAL(0, static_cast<int>(parseGcode("G0 J5=1", g)));
    TEST_ASSERT_NOT_EQUAL(0, static_cast<int>(parseGcode("G1 Q5", g)));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_decode_angle);
    RUN_TEST(test_circular_delta_wraps);
    RUN_TEST(test_turn_counter_multi_turn);
    RUN_TEST(test_jump_filter);
    RUN_TEST(test_jump_filter_across_zero);
    RUN_TEST(test_gcode_parse);
    return UNITY_END();
}
