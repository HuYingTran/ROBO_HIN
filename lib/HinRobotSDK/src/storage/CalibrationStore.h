#pragma once
#include <cstdint>

#include "../common/Status.h"
#include "../common/Types.h"

namespace hin {

struct CalibrationData {
    static constexpr uint32_t kMagic = 0x48494E31;  // "HIN1"
    uint32_t magic = kMagic;
    float encoderOffset[kMaxJoints] = {};  // đơn vị khớp
    uint8_t elbow = static_cast<uint8_t>(ElbowConfig::Nearest);
};

// Lưu / đọc hiệu chuẩn trên NVS (namespace "hin").
class CalibrationStore {
public:
    Status load(CalibrationData& out);
    Status save(const CalibrationData& in);
    Status erase();
};

}  // namespace hin
