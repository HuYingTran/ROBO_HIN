#if defined(ARDUINO)
#include "CalibrationStore.h"

#include <Preferences.h>

namespace hin {

namespace {
constexpr const char* kNs = "hin";
constexpr const char* kKey = "cal";
}  // namespace

Status CalibrationStore::load(CalibrationData& out) {
    Preferences p;
    if (!p.begin(kNs, true)) return Status::StorageError;
    CalibrationData tmp;
    const size_t n = p.getBytes(kKey, &tmp, sizeof(tmp));
    p.end();
    if (n != sizeof(tmp) || tmp.magic != CalibrationData::kMagic) return Status::StorageError;
    out = tmp;
    return Status::Ok;
}

Status CalibrationStore::save(const CalibrationData& in) {
    Preferences p;
    if (!p.begin(kNs, false)) return Status::StorageError;
    const size_t n = p.putBytes(kKey, &in, sizeof(in));
    p.end();
    return n == sizeof(in) ? Status::Ok : Status::StorageError;
}

Status CalibrationStore::erase() {
    Preferences p;
    if (!p.begin(kNs, false)) return Status::StorageError;
    const bool ok = p.remove(kKey);
    p.end();
    return ok ? Status::Ok : Status::StorageError;
}

}  // namespace hin
#endif
