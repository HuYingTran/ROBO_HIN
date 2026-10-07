#pragma once
#include <cstdint>

#include "../common/Status.h"

namespace hin {

class IEncoder {
public:
    virtual ~IEncoder() = default;
    // Góc trong 1 vòng [0, 2π).
    virtual Status read(float& angleRad) = 0;
    // Góc thô (tick) của lần đọc thành công gần nhất. Dùng để đếm vòng.
    virtual Status readRaw(uint16_t& ticks) = 0;
    virtual uint32_t ticksPerRev() const = 0;
    virtual FieldStatus fieldStatus() const = 0;
    // Đặt bước nhảy tối đa hợp lệ giữa 2 mẫu (tick), 0 = tắt kiểm tra.
    virtual void setMaxStep(uint16_t ticks) = 0;
    // Quên mẫu trước (sau khi xóa lỗi / gỡ treo bus).
    virtual void resetFilter() = 0;
};

}  // namespace hin
