#pragma once
#include "IEncoder.h"
#include "Mt6701Decode.h"

namespace hin {

class Tca9548a;

// MT6701 đọc qua I2C, đằng sau một kênh TCA9548A.
// I2C không có CRC → tự thử lại khi NACK, bỏ mẫu nhảy vô lý.
class MT6701I2c : public IEncoder {
public:
    MT6701I2c() = default;
    MT6701I2c(Tca9548a& mux, uint8_t channel) { attach(mux, channel); }

    void attach(Tca9548a& mux, uint8_t channel);

    Status read(float& angleRad) override;
    Status readRaw(uint16_t& ticks) override;
    uint32_t ticksPerRev() const override { return mt6701::kTicksPerRev; }
    FieldStatus fieldStatus() const override;
    void setMaxStep(uint16_t ticks) override;
    void resetFilter() override { filter_.reset(); }

    uint8_t channel() const { return channel_; }
    uint16_t errorStreak() const { return errStreak_; }

private:
    Status readOnce(uint16_t& raw);

    Tca9548a* mux_ = nullptr;
    uint8_t channel_ = 0;
    mt6701::JumpFilter filter_;
    uint16_t errStreak_ = 0;
    bool everOk_ = false;
};

}  // namespace hin
