#pragma once

namespace hin {

class IGripper {
public:
    virtual ~IGripper() = default;
    virtual void open() { setPosition(0.0f); }
    virtual void close() { setPosition(100.0f); }
    // 0 = mở hết, 100 = kẹp hết.
    virtual void setPosition(float percent) = 0;
    virtual float position() const = 0;
};

}  // namespace hin
