#include "Status.h"

namespace hin {

const char* toString(Status s) {
    switch (s) {
        case Status::Ok: return "Ok";
        case Status::Error: return "Error";
        case Status::InvalidArgument: return "InvalidArgument";
        case Status::OutOfRange: return "OutOfRange";
        case Status::OutOfReach: return "OutOfReach";
        case Status::JointLimit: return "JointLimit";
        case Status::NotReady: return "NotReady";
        case Status::NotHomed: return "NotHomed";
        case Status::Busy: return "Busy";
        case Status::Timeout: return "Timeout";
        case Status::BusError: return "BusError";
        case Status::Implausible: return "Implausible";
        case Status::Faulted: return "Faulted";
        case Status::QueueFull: return "QueueFull";
        case Status::Unsupported: return "Unsupported";
        case Status::StorageError: return "StorageError";
    }
    return "?";
}

const char* toString(Fault f) {
    switch (f) {
        case Fault::None: return "None";
        case Fault::EStop: return "EStop";
        case Fault::StallDetected: return "StallDetected";
        case Fault::EncoderError: return "EncoderError";
        case Fault::HomingTimeout: return "HomingTimeout";
        case Fault::HomingFailed: return "HomingFailed";
        case Fault::LimitHit: return "LimitHit";
        case Fault::Watchdog: return "Watchdog";
        case Fault::Internal: return "Internal";
    }
    return "?";
}

const char* toString(RobotState s) {
    switch (s) {
        case RobotState::Disabled: return "Disabled";
        case RobotState::Unhomed: return "Unhomed";
        case RobotState::Idle: return "Idle";
        case RobotState::Homing: return "Homing";
        case RobotState::Moving: return "Moving";
        case RobotState::Fault: return "Fault";
        case RobotState::EStop: return "EStop";
    }
    return "?";
}

const char* toString(FieldStatus f) {
    switch (f) {
        case FieldStatus::Unknown: return "Unknown";
        case FieldStatus::Ok: return "OK";
        case FieldStatus::Lost: return "LOST";
    }
    return "?";
}

}  // namespace hin
