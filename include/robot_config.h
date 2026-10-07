#pragma once
#include <HinRobot.h>

// Cấu hình robot HIN — mọi tham số đặc thù phần cứng nằm ở đây.
// Pin map: README mục 4. Hình học: README mục 5.4.

using hin::JointType::Prismatic;
using hin::JointType::Revolute;
using hin::EncoderMount::Screw;
using hin::EncoderMount::Output;

constexpr hin::JointConfig JOINTS[] = {
// name       type       step dir ch  µstep ratio       encMount invert  min   max   vmax  amax
//                                          (T: mm/vòng
//                                           R: tỉ số đai)                (mm | °)  (/s)  (/s²)
  {"z",        Prismatic, 25,  26, 0,  16,   8.0f,       Screw,   false,  0,    200,  40,   200},
  {"shoulder", Revolute,  27,  14, 1,  16,  10.0f,       Output,  false, -135,  135,  90,   180},
  {"elbow",    Revolute,  32,  33, 2,  16,   8.0f,       Output,  false, -145,  145,  120,  240},
  {"wrist",    Revolute,  13,  23, 3,  16,   3.0f,       Output,  false, -180,  180,  180,  360},
};

// Hình học SCARA (mm)
constexpr hin::ScaraGeometry GEOMETRY = {
    .a2 = 150.0f,   // J2 → J3
    .a3 = 120.0f,   // J3 → J4
    .d4 =  60.0f,   // mặt bích J4 → đầu tay kẹp (hướng xuống)
    .z0 =  80.0f,   // độ cao cánh tay khi d1 = 0
};

constexpr hin::HomingConfig HOMING = {
    .joint      = 0,       // J1
    .limitPin   = 34,
    .direction  = -1,      // chạy xuống
    .fastMmS    = 10.0f,
    .slowMmS    = 2.0f,
    .backoffMm  = 3.0f,
    .timeoutMs  = 30000,
};

constexpr hin::RobotConfig ROBOT_CONFIG = {
    .joints            = JOINTS,
    .jointCount        = 4,
    .geometry          = GEOMETRY,
    .homing            = HOMING,
    .enablePin         = 4,
    .encoderBus        = {.port = 0, .sda = 21, .scl = 22, .hz = 400'000,
                          .muxAddr = 0x70, .muxResetPin = 19},   // ch = kênh TCA9548A
    .gripperPin        = 18,
    .estopPin          = 39,
    .followingErrorMm  = 0.5f,   // J1 — vượt ngưỡng → Fault::StallDetected
    .followingErrorDeg = 2.0f,   // J2–J4
    .limitZMaxPin      = 35,     // tùy chọn, -1 nếu không lắp
};
