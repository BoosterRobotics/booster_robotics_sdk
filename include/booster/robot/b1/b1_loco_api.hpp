#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__

#include <string>
#include <booster/third_party/nlohmann_json/json.hpp>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/common/robot_mode.hpp>
#include <booster/robot/common/entities.hpp>

namespace booster {
namespace robot {
namespace b1 {

/* service name */
const std::string LOCO_SERVICE_NAME = "loco";

/*API version*/
const std::string LOCO_API_VERSION = "1.0.0.1";

/*API ID */
enum class LocoApiId {
    kChangeMode = 2000,
    kMove = 2001,
    kRotateHead = 2004,
    kWaveHand = 2005,
    kRotateHeadWithDirection = 2006,
    kLieDown = 2007,
    kGetUp = 2008,
    kMoveHandEndEffector = 2009,
    kControlGripper = 2010
};

class RotateHeadParameter {
public:
    RotateHeadParameter() = default;
    RotateHeadParameter(float pitch, float yaw) :
        pitch_(pitch),
        yaw_(yaw) {
    }

public:
    void FromJson(nlohmann::json &json) {
        pitch_ = json["pitch"];
        yaw_ = json["yaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["pitch"] = pitch_;
        json["yaw"] = yaw_;
        return json;
    }

public:
    float pitch_;
    float yaw_;
};

class ChangeModeParameter {
public:
    ChangeModeParameter() = default;
    ChangeModeParameter(booster::robot::RobotMode mode) :
        mode_(mode) {
    }

public:
    void FromJson(nlohmann::json &json) {
        mode_ = static_cast<booster::robot::RobotMode>(json["mode"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["mode"] = static_cast<int>(mode_);
        return json;
    }

public:
    booster::robot::RobotMode mode_;
};

class MoveParameter {
public:
    MoveParameter() = default;
    MoveParameter(float vx, float vy, float vyaw) :
        vx_(vx),
        vy_(vy),
        vyaw_(vyaw) {
    }

public:
    void FromJson(nlohmann::json &json) {
        vx_ = json["vx"];
        vy_ = json["vy"];
        vyaw_ = json["vyaw"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["vx"] = vx_;
        json["vy"] = vy_;
        json["vyaw"] = vyaw_;
        return json;
    }

public:
    float vx_;
    float vy_;
    float vyaw_;
};

class RotateHeadWithDirectionParameter {
public:
    RotateHeadWithDirectionParameter() = default;
    RotateHeadWithDirectionParameter(int pitch_direction, int yaw_direction) :
        pitch_direction_(pitch_direction),
        yaw_direction_(yaw_direction) {
    }

public:
    void FromJson(nlohmann::json &json) {
        pitch_direction_ = json["pitch_direction"];
        yaw_direction_ = json["yaw_direction"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["pitch_direction"] = pitch_direction_;
        json["yaw_direction"] = yaw_direction_;
        return json;
    }

public:
    int pitch_direction_;
    int yaw_direction_;
};

class WaveHandParameter {
public:
    WaveHandParameter() = default;
    WaveHandParameter(HandIndex hand_index, HandAction hand_action) :
        hand_action_(hand_action), hand_index_(hand_index) {
    }

public:
    void FromJson(nlohmann::json &json) {
        hand_index_ = static_cast<HandIndex>(json["hand_index"]);
        hand_action_ = static_cast<HandAction>(json["hand_action"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["hand_index"] = static_cast<int>(hand_index_);
        json["hand_action"] = static_cast<int>(hand_action_);
        return json;
    }

public:
    HandIndex hand_index_;
    HandAction hand_action_;
};

class MoveHandEndEffectorParameter {
public:
    MoveHandEndEffectorParameter() = default;
    MoveHandEndEffectorParameter(
        const Posture &target_posture,
        int time_millis,
        HandIndex hand_index) :
        target_posture_(target_posture),
        time_millis_(time_millis),
        hand_index_(hand_index) {
        has_aux_ = false;
    }
    MoveHandEndEffectorParameter(
        const Posture &target_posture,
        const Posture &aux_posture,
        int time_millis,
        HandIndex hand_index) :
        target_posture_(target_posture),
        aux_posture_(aux_posture),
        time_millis_(time_millis),
        hand_index_(hand_index) {
        has_aux_ = true;
    }

public:
    void FromJson(nlohmann::json &json) {
        target_posture_.FromJson(json["target_posture"]);
        has_aux_ = json["has_aux"];
        if (has_aux_) {
            aux_posture_.FromJson(json["aux_posture"]);
        }
        time_millis_ = json["time_millis"];
        hand_index_ = static_cast<HandIndex>(json["hand_index"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["target_posture"] = target_posture_.ToJson();
        if (has_aux_) {
            json["aux_posture"] = aux_posture_.ToJson();
        }
        json["time_millis"] = time_millis_;
        json["hand_index"] = static_cast<int>(hand_index_);
        json["has_aux"] = has_aux_;
        return json;
    }

public:
    Posture target_posture_;
    Posture aux_posture_;
    int time_millis_ = 1000;
    HandIndex hand_index_;
    bool has_aux_ = false;
};

enum class GripperControlMode {
    // position mode, the gripper will stop when it reaches the target position,
    // or when it experiences a specific reaction force, this reaction force
    // specified in the motion parameter
    kPosition = 0,
    // torque mode, if the gripper not reaches the target position, the gripper will
    // continue to move with the torque specified in the motion parameter
    kTorque = 1
};

/**
 *  This class definition represents a gripper motion parameter. Different grippers
 *  have different motion parameters.
 *
 *  The following parameters all represent a scaling factor. When using them, you
 *  need to convert them into the corresponding coefficients based on the specifications
 *  of the gripper you are using.
 *
 *  For the Inspire EG2-4C2 Gripper:
 *  - Maximum position: 77mm, Therefore, a position value of (0 ~ 1000) corresponds to (0 ~ 77 mm)
 *  - Maximum force: 2kg, Therefore, a force value of (0 ~ 1000) corresponds to (0 ~ 2 kg)
 *  - Speed unit: Not specified, The Inspire EG2-4C2 Gripper does not provide a unit, range, 
 *    or dimension for speed. The speed can only be adjusted using values from 0 to 1000, which 
 *    do not correspond to an absolute value
 *
 *  position: represents the gripper's opening value, ranging from 0 to 1000
 *  force: represents the gripper's force control value, ranging from 50 to 1000
 *  speed represents the gripper's opening and closing speed ranging from 1 to 1000
 */
class GripperMotionParameter {
public:
    GripperMotionParameter() = default;
    GripperMotionParameter(const int32_t position, const int32_t force, const int32_t speed) :
        position_(position), force_(force), speed_(speed) {
    }

    void FromJson(nlohmann::json &json) {
        position_ = json["position"];
        force_ = json["force"];
        speed_ = json["speed"];
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["position"] = position_;
        json["force"] = force_;
        json["speed"] = speed_;
        return json;
    }

public:
    int32_t position_ = 0;
    int32_t force_ = 0;
    int32_t speed_ = 0;
};

class ControlGripperParameter {
public:
    ControlGripperParameter() = default;
    ControlGripperParameter(const GripperMotionParameter &motion_param, GripperControlMode mode, HandIndex hand_index) :
        motion_param_(motion_param), mode_(mode), hand_index_(hand_index) {
    }

public:
    void FromJson(nlohmann::json &json) {
        motion_param_.FromJson(json["motion_param"]);
        mode_ = static_cast<GripperControlMode>(json["mode"]);
        hand_index_ = static_cast<HandIndex>(json["hand_index"]);
    }

    nlohmann::json ToJson() const {
        nlohmann::json json;
        json["motion_param"] = motion_param_.ToJson();
        json["mode"] = static_cast<int>(mode_);
        json["hand_index"] = static_cast<int>(hand_index_);
        return json;
    }

public:
    GripperMotionParameter motion_param_;
    GripperControlMode mode_;
    HandIndex hand_index_;
};

}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_API_HPP__