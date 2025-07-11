#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include "booster/robot/b1/b1_loco_client.hpp"
#include "booster/robot/b1/b1_api_const.hpp"
#include "booster/robot/b1/b1_loco_api.hpp"
#include "booster/idl/b1/ImuState.h"
#include "booster/idl/b1/LowState.h"
#include "booster/idl/b1/MotorState.h"
#include "booster/idl/b1/LowCmd.h"
#include "booster/idl/b1/MotorCmd.h"
#include "booster/idl/b1/Odometer.h"
#include "booster/robot/common/robot_shared.hpp"
#include "booster/robot/common/entities.hpp"
#include "booster/robot/channel/channel_factory.hpp"
#include "booster/idl/b1/HandReplyData.h"
#include "booster/idl/b1/HandReplyParam.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
namespace robot = booster::robot;
using booster_interface::msg::LowState;
using booster_interface::msg::MotorState;
using booster_interface::msg::LowCmd;
using booster_interface::msg::MotorCmd;
using booster_interface::msg::Odometer;
using booster_interface::msg::ImuState;
using booster_interface::msg::CmdType;
using booster_interface::msg::HandReplyData;
using booster_interface::msg::HandReplyParam;

namespace booster::robot::b1 {
class __attribute__((visibility("hidden"))) B1LowStateSubscriber : public std::enable_shared_from_this<B1LowStateSubscriber> {
public:
    B1LowStateSubscriber(const py::function &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        py::gil_scoped_release release;
        auto weak_this = std::weak_ptr<B1LowStateSubscriber>(shared_from_this());
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<LowState>(channel_name_, [weak_this](const void *msg) {
            if (auto shared_this = weak_this.lock()) {
                {
                    py::gil_scoped_acquire acquire;
                    const LowState *low_state_msg = static_cast<const LowState *>(msg);
                    shared_this->py_handler_(low_state_msg);
                }
            }
        });
    }

    void CloseChannel() {
        py::gil_scoped_release release;
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<booster_interface::msg::LowState> channel_ptr_;
    py::function py_handler_;
    const std::string channel_name_ = kTopicLowState;
};

class __attribute__((visibility("hidden"))) B1LowHandDataScriber : public std::enable_shared_from_this<B1LowHandDataScriber> {
public:
    B1LowHandDataScriber(const py::function &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        py::gil_scoped_release release;
        auto weak_this = std::weak_ptr<B1LowHandDataScriber>(shared_from_this());
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<HandReplyData>(channel_name_, [weak_this](const void *msg) {
            if (auto shared_this = weak_this.lock()) {
                {
                    py::gil_scoped_acquire acquire;
                    const HandReplyData *hand_data = static_cast<const HandReplyData *>(msg);
                    shared_this->py_handler_(hand_data);
                }
            }
        });
    }

    void CloseChannel() {
        py::gil_scoped_release release;
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<booster_interface::msg::HandReplyData> channel_ptr_;
    py::function py_handler_;
    const std::string channel_name_ = "rt/booster_hand_data";
};

class __attribute__((visibility("hidden"))) B1LowCmdPublisher {
public:
    explicit B1LowCmdPublisher() :
        channel_name_(kTopicJointCtrl) {
    }

    void InitChannel() {
        py::gil_scoped_release release;
        channel_ptr_ = ChannelFactory::Instance()->CreateSendChannel<LowCmd>(channel_name_);
    }

    bool Write(LowCmd *msg) {
        if (channel_ptr_) {
            return channel_ptr_->Write(msg);
        }
        return false;
    }

    void CloseChannel() {
        py::gil_scoped_release release;
        if (channel_ptr_) {
            ChannelFactory::Instance()->CloseWriter(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    std::string channel_name_;
    ChannelPtr<LowCmd> channel_ptr_;
};

class __attribute__((visibility("hidden"))) B1OdometerStateSubscriber : public std::enable_shared_from_this<B1OdometerStateSubscriber> {
public:
    B1OdometerStateSubscriber(const py::function &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        py::gil_scoped_release release;
        auto weak_this = std::weak_ptr<B1OdometerStateSubscriber>(shared_from_this());
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<Odometer>(channel_name_, [weak_this](const void *msg) {
            if (auto shared_this = weak_this.lock()) {
                {
                    py::gil_scoped_acquire acquire;
                    const Odometer *low_state_msg = static_cast<const Odometer *>(msg);
                    shared_this->py_handler_(low_state_msg);
                }
            }
        });
    }

    void CloseChannel() {
        py::gil_scoped_release release;
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<Odometer> channel_ptr_;
    py::function py_handler_;
    const std::string channel_name_ = kTopicOdometerState;
};
} // namespace booster::robot::b1

PYBIND11_MODULE(booster_robotics_sdk_python, m) {
    m.doc() = R"pbdoc(
        python binding of booster robotics sdk
        -----------------------
    )pbdoc";

    py::class_<robot::ChannelFactory>(m, "ChannelFactory")
        .def_static("Instance", &robot::ChannelFactory::Instance, py::return_value_policy::reference,
                    R"pbdoc(
                        Get the singleton instance of the channel factory.

                        Note: The returned instance is managed internally and should not be deleted or modified.
                    )pbdoc")
        .def("Init", py::overload_cast<int32_t, const std::string &>(&robot::ChannelFactory::Init), py::arg("domain_id"), py::arg("network_interface") = "",
             R"pbdoc(
                domain_id: domain id of DDS
                network_interface: network interface of DDS, default empty string
            )pbdoc");

    py::enum_<robot::RobotMode>(m, "RobotMode")
        .value("kUnknown", robot::RobotMode::kUnknown)
        .value("kDamping", robot::RobotMode::kDamping)
        .value("kPrepare", robot::RobotMode::kPrepare)
        .value("kWalking", robot::RobotMode::kWalking)
        .value("kCustom", robot::RobotMode::kCustom)
        .export_values();

    py::enum_<robot::b1::JointIndex>(m, "B1JointIndex")
        .value("kHeadYaw", robot::b1::JointIndex::kHeadYaw)
        .value("kHeadPitch", robot::b1::JointIndex::kHeadPitch)
        .value("kLeftShoulderPitch", robot::b1::JointIndex::kLeftShoulderPitch)
        .value("kLeftShoulderRoll", robot::b1::JointIndex::kLeftShoulderRoll)
        .value("kLeftElbowPitch", robot::b1::JointIndex::kLeftElbowPitch)
        .value("kLeftElbowYaw", robot::b1::JointIndex::kLeftElbowYaw)
        .value("kRightShoulderPitch", robot::b1::JointIndex::kRightShoulderPitch)
        .value("kRightShoulderRoll", robot::b1::JointIndex::kRightShoulderRoll)
        .value("kRightElbowPitch", robot::b1::JointIndex::kRightElbowPitch)
        .value("kRightElbowYaw", robot::b1::JointIndex::kRightElbowYaw)
        .value("kWaist", robot::b1::JointIndex::kWaist)
        .value("kLeftHipPitch", robot::b1::JointIndex::kLeftHipPitch)
        .value("kLeftHipRoll", robot::b1::JointIndex::kLeftHipRoll)
        .value("kLeftHipYaw", robot::b1::JointIndex::kLeftHipYaw)
        .value("kLeftKneePitch", robot::b1::JointIndex::kLeftKneePitch)
        .value("kCrankUpLeft", robot::b1::JointIndex::kCrankUpLeft)
        .value("kCrankDownLeft", robot::b1::JointIndex::kCrankDownLeft)
        .value("kRightHipPitch", robot::b1::JointIndex::kRightHipPitch)
        .value("kRightHipRoll", robot::b1::JointIndex::kRightHipRoll)
        .value("kRightHipYaw", robot::b1::JointIndex::kRightHipYaw)
        .value("kRightKneePitch", robot::b1::JointIndex::kRightKneePitch)
        .value("kCrankUpRight", robot::b1::JointIndex::kCrankUpRight)
        .value("kCrankDownRight", robot::b1::JointIndex::kCrankDownRight)
        .export_values();

    m.attr("B1JointCnt") = robot::b1::kJointCnt;

    py::enum_<robot::b1::LocoApiId>(m, "B1LocoApiId")
        .value("kChangeMode", robot::b1::LocoApiId::kChangeMode)
        .value("kMove", robot::b1::LocoApiId::kMove)
        .value("kRotateHead", robot::b1::LocoApiId::kRotateHead)
        .export_values();

    py::enum_<robot::b1::HandAction>(m, "B1HandAction")
        .value("kHandOpen", robot::b1::HandAction::kHandOpen)
        .value("kHandClose", robot::b1::HandAction::kHandClose)
        .export_values();

    py::enum_<robot::b1::HandIndex>(m, "B1HandIndex")
        .value("kLeftHand", robot::b1::HandIndex::kLeftHand)
        .value("kRightHand", robot::b1::HandIndex::kRightHand)
        .export_values();

    py::enum_<robot::b1::GripperControlMode>(m, "GripperControlMode")
        .value("kPosition", robot::b1::GripperControlMode::kPosition,
               "Position mode: stops at target position or specified reaction force")
        .value("kForce", robot::b1::GripperControlMode::kForce,
               "Force mode: continues to move with specified force if target position is not reached")
        .export_values();

    py::enum_<robot::Frame>(m, "Frame")
        .value("kUnknown", robot::Frame::kUnknown)
        .value("kBody", robot::Frame::kBody)
        .value("kHead", robot::Frame::kHead)
        .value("kLeftHand", robot::Frame::kLeftHand)
        .value("kRightHand", robot::Frame::kRightHand)
        .value("kLeftFoot", robot::Frame::kLeftFoot)
        .value("kRightFoot", robot::Frame::kRightFoot)
        .export_values();

    // Bind Position class
    py::class_<Position>(m, "Position")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &Position::x_)
        .def_readwrite("y", &Position::y_)
        .def_readwrite("z", &Position::z_);

    // Bind Orientation class
    py::class_<Orientation>(m, "Orientation")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("roll"), py::arg("pitch"), py::arg("yaw"))
        .def_readwrite("roll", &Orientation::roll_)
        .def_readwrite("pitch", &Orientation::pitch_)
        .def_readwrite("yaw", &Orientation::yaw_);

    // Bind Posture class
    py::class_<Posture>(m, "Posture")
        .def(py::init<>())
        .def(py::init<const Position &, const Orientation &>(), py::arg("position"), py::arg("orientation"))
        .def_readwrite("position", &Posture::position_)
        .def_readwrite("orientation", &Posture::orientation_);

    // Bind Quaternion class
    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<float, float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"), py::arg("w"))
        .def_readwrite("x", &Quaternion::x_)
        .def_readwrite("y", &Quaternion::y_)
        .def_readwrite("z", &Quaternion::z_)
        .def_readwrite("w", &Quaternion::w_);

    // Bind Transform class
    py::class_<Transform>(m, "Transform")
        .def(py::init<>())
        .def(py::init<const Position &, const Quaternion &>(), py::arg("position"), py::arg("orientation"))
        .def_readwrite("position", &Transform::position_)
        .def_readwrite("orientation", &Transform::orientation_);

    py::class_<robot::b1::GripperMotionParameter>(m, "GripperMotionParameter")
        .def(py::init<>())                          // Default constructor
        .def(py::init<int32_t, int32_t, int32_t>(), // constructor with parameters
             py::arg("position"), py::arg("force"), py::arg("speed"))
        .def_readwrite("position", &robot::b1::GripperMotionParameter::position_)
        .def_readwrite("force", &robot::b1::GripperMotionParameter::force_)
        .def_readwrite("speed", &robot::b1::GripperMotionParameter::speed_);

    py::class_<robot::b1::GetModeResponse>(m, "GetModeResponse")
        .def(py::init<>())
        .def_readwrite("mode", &robot::b1::GetModeResponse::mode_);

    py::class_<robot::b1::DexterousFingerParameter>(m, "DexterousFingerParameter")
        .def(py::init<>())
        .def(py::init<int32_t, int32_t, int32_t, int32_t>(),
             py::arg("seq"), py::arg("angle"), py::arg("force"), py::arg("speed"))
        .def_readwrite("seq", &robot::b1::DexterousFingerParameter::seq_)
        .def_readwrite("angle", &robot::b1::DexterousFingerParameter::angle_)
        .def_readwrite("force", &robot::b1::DexterousFingerParameter::force_)
        .def_readwrite("speed", &robot::b1::DexterousFingerParameter::speed_);

    py::class_<robot::b1::B1LocoClient>(m, "B1LocoClient", R"pbdoc(
        B1LocoClient is a client interface for controlling the B1 robot's locomotion and other high-level functionalities.
        It provides methods to send API requests, change robot modes, move the robot, control its head and hands, and more.
        .def("Init", py::overload_cast<const std::string &>(&robot::b1::B1LocoClient::Init), py::arg("robot_name"), R"pbdoc(
                /**
                 * @brief Initialize the B1LocoClient with a specific robot name.
                 * 
                 * @param robot_name The name of the robot to initialize the client for.
                 */
            )pbdoc")
        .def(py::init<>())
        .def(
            "Init", [](robot::b1::B1LocoClient &client) {
                py::gil_scoped_release release;
                return client.Init();
            },
            "Init")
        .def(
            "Init", [](robot::b1::B1LocoClient &client, const std::string &robot_name) {
                py::gil_scoped_release release;
                return client.Init(robot_name);
            },
            "Init with robot name")
        .def("SendApiRequest", &robot::b1::B1LocoClient::SendApiRequest, py::arg("api_id"), py::arg("param"),
             R"pbdoc(
                /**
                 * @brief Send API request to B1 robot
                 * 
                 * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
                 * @param param API parameter
                 * 
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("ChangeMode", &robot::b1::B1LocoClient::ChangeMode, py::arg("mode"),
             R"pbdoc(
                /**
                 * @brief Change robot mode
                 * 
                 * @param mode robot mode, options are: kDamping, kPrepare, kWalking
                 * 
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("GetMode", &robot::b1::B1LocoClient::GetMode, py::arg("get_mode_response"),
             R"pbdoc(
                /**
                 * @brief Get current robot mode
                 *
                 * @param[out] get_mode_response Reference to store the response data, including:
                 *              - current_mode (RobotMode enum value)
                 *
                 * @return 0 if success, otherwise return error code
                 * @see ChangeMode() for mode switching API
                 * @see RobotMode enum for available mode definitions
                 */
               )pbdoc")
        .def("Move", &robot::b1::B1LocoClient::Move, py::arg("vx"), py::arg("vy"), py::arg("vyaw"),
             R"pbdoc(
                /**
                 * @brief Move robot
                 * 
                 * @param vx linear velocity in x direction, unit: m/s
                 * @param vy linear velocity in y direction, unit: m/s
                 * @param vyaw angular velocity, unit: rad/s
                 * 
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("RotateHead", &robot::b1::B1LocoClient::RotateHead, py::arg("pitch"), py::arg("yaw"),
             R"pbdoc(
                 /**
                 * @brief Robot rotates its head
                 *
                 * @param pitch pitch angle, unit: rad
                 * @param yaw yaw angle, unit: rad
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("RotateHeadWithDirection", &robot::b1::B1LocoClient::RotateHeadWithDirection, py::arg("pitch_direction"), py::arg("yaw_direction"),
             R"pbdoc(
                 /**
                 * @brief Robot rotates its head with direction
                 *
                 * @param pitch_direction pitch direction, unit: rad
                 * @param yaw_direction yaw direction, unit: rad
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("WaveHand", &robot::b1::B1LocoClient::WaveHand, py::arg("action"),
             R"pbdoc(
                 /**
                 * @brief Robot waves hand
                 *
                 * @param action hand action, options are: kHandOpen, kHandClose
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("Handshake", &robot::b1::B1LocoClient::Handshake, py::arg("action"),
             R"pbdoc(
                 /**
                 * @brief Handshake
                 *
                 * @param action whether to start handshake action, options are: kHandOpen, kHandClose
                 *
                 * @return 0 if success, otherwise return error code
                 */
            )pbdoc")
        .def("MoveHandEndEffectorWithAux", &robot::b1::B1LocoClient::MoveHandEndEffectorWithAux, py::arg("target_posture"), py::arg("aux_posture"), py::arg("time_millis"), py::arg("hand_index"),
             R"pbdoc(
                /**
                 *  @brief Move hand end-effector to a target posture(position & orientation) with an auxiliary point
                 *
                 *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach. 
                 *  It contains position & orientation.
                 *  @param aux_posture Represents the auxiliary point on the end-effector's motion arc trajectory
                 *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
                 *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
                 *
                 *  @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("MoveHandEndEffector", &robot::b1::B1LocoClient::MoveHandEndEffector, py::arg("target_posture"), py::arg("time_millis"), py::arg("hand_index"),
             R"pbdoc(
                /**
                 *  @brief Move hand end-effector with a target posture(position & orientation)
                 *  @deprecated **This API is deprecated and will be removed in future versions.**
                 *              Please use the new API `MoveHandEndEffectorV2` instead.
                 *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach. 
                 *                        It contains position & orientation. 
                 *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
                 *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
                 *
                 *  @return 0 if success, otherwise return error code
                 * 
                 *  @details
                 *  **Reason for deprecation**: This API is deprecated due to an implicit rotational offset (rot) being applied to the target orientation. 
                 *  The final orientation is calculated as orientation = rot * offset, which contradicts the parameter description of `target_posture`.
                 */
                )pbdoc")
        .def("MoveHandEndEffectorV2", &robot::b1::B1LocoClient::MoveHandEndEffectorV2, py::arg("target_posture"), py::arg("time_millis"), py::arg("hand_index"),
             R"pbdoc(
                /**
                 *  @brief Move hand end-effector with a target posture(position & orientation)
                 *
                 *  @param target_posture Represents the target posture in base frame (torso frame) that the hand end-effector should reach. It contains position & orientation. 
                 *  @param time_mills Specifies the duration, in milliseconds, for completing the movement.
                 *  @param hand_index Identifies which hand the parameter refers to (for instance, left hand or right hand).
                 *
                 *  @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("ControlGripper", &robot::b1::B1LocoClient::ControlGripper, py::arg("motion_param"), py::arg("mode"), py::arg("hand_index"),
             R"pbdoc(
                /**
                 * @brief Control gripper
                 *
                 * @param motion_param motion parameter, include position, force, velocity, see `GripperMotionParameter`
                 * @param mode gripper control mode, options are: kPosition, kForce, see `GripperControlMode`
                 * @param hand_index hand index, options are: kLeftHand, kRightHand
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("GetFrameTransform", &robot::b1::B1LocoClient::GetFrameTransform, py::arg("src"), py::arg("dst"), py::arg("transform"),
             R"pbdoc(
                /**
                 * @brief Get frame transform
                 *
                 * @param src source frame
                 * @param dst destination frame
                 * @param transform [out] calculated transform
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("SwitchHandEndEffectorControlMode", &robot::b1::B1LocoClient::SwitchHandEndEffectorControlMode, py::arg("switch_on"),
             R"pbdoc(
                /**
                 * @brief Switch hand end-effector control mode
                 * 
                 * @param switch_on true to switch on, false to switch off
                 * 
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("ControlDexterousHand", &robot::b1::B1LocoClient::ControlDexterousHand, py::arg("finger_params"), py::arg("hand_index"),
             R"pbdoc(
                /**
                 * @brief Control dexterous hand
                 *
                 * @param finger_params finger parameters, include position, force, speed, see `DexterousFingerParameter`
                 * @param hand_index hand index, options are: kLeftHand, kRightHand
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("GetUp", &robot::b1::B1LocoClient::GetUp,
             R"pbdoc(
                /**
                 * @brief Stand up
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc")
        .def("LieDown", &robot::b1::B1LocoClient::LieDown,
             R"pbdoc(
                /**
                 * @brief Lie down
                 *
                 * @return 0 if success, otherwise return error code
                 */
                )pbdoc");

    py::class_<ImuState>(m, "ImuState")
        .def(py::init<>())
        .def(py::init<const ImuState &>())
        .def_property("rpy",
                      (const std::array<float, 3> &(ImuState::*)() const) & ImuState::rpy,
                      (void(ImuState::*)(const std::array<float, 3> &)) & ImuState::rpy)
        .def_property("gyro",
                      (const std::array<float, 3> &(ImuState::*)() const) & ImuState::gyro,
                      (void(ImuState::*)(const std::array<float, 3> &)) & ImuState::gyro)
        .def_property("acc",
                      (const std::array<float, 3> &(ImuState::*)() const) & ImuState::acc,
                      (void(ImuState::*)(const std::array<float, 3> &)) & ImuState::acc)
        .def("__eq__", &ImuState::operator==)
        .def("__ne__", &ImuState::operator!=);

    py::class_<MotorState>(m, "MotorState")
        .def(py::init<>())
        .def(py::init<const MotorState &>())
        .def_property("mode",
                      (uint8_t(MotorState::*)() const) & MotorState::mode,
                      (uint8_t & (MotorState::*)()) & MotorState::mode)
        .def_property("q",
                      (float(MotorState::*)() const) & MotorState::q,
                      (float &(MotorState::*)()) & MotorState::q)
        .def_property("dq",
                      (float(MotorState::*)() const) & MotorState::dq,
                      (float &(MotorState::*)()) & MotorState::dq)
        .def_property("ddq",
                      (float(MotorState::*)() const) & MotorState::ddq,
                      (float &(MotorState::*)()) & MotorState::ddq)
        .def_property("tau_est",
                      (float(MotorState::*)() const) & MotorState::tau_est,
                      (float &(MotorState::*)()) & MotorState::tau_est)
        .def_property("temperature",
                      (uint8_t(MotorState::*)() const) & MotorState::temperature,
                      (uint8_t & (MotorState::*)()) & MotorState::temperature)
        .def_property("lost",
                      (uint32_t(MotorState::*)() const) & MotorState::lost,
                      (uint32_t & (MotorState::*)()) & MotorState::lost)
        .def_property("reserve",
                      (const std::array<uint32_t, 2> &(MotorState::*)() const) & MotorState::reserve,
                      (std::array<uint32_t, 2> & (MotorState::*)()) & MotorState::reserve)
        .def("__eq__", &MotorState::operator==)
        .def("__ne__", &MotorState::operator!=);

    py::class_<LowState>(m, "LowState")
        .def(py::init<>())
        .def(py::init<const LowState &>())
        .def_property("imu_state",
                      (const ImuState &(LowState::*)() const) & LowState::imu_state,
                      (void(LowState::*)(const ImuState &)) & LowState::imu_state)
        .def_property("motor_state_parallel",
                      (const std::vector<MotorState> &(LowState::*)() const) & LowState::motor_state_parallel,
                      (void(LowState::*)(const std::vector<MotorState> &)) & LowState::motor_state_parallel)
        .def_property("motor_state_serial",
                      (const std::vector<MotorState> &(LowState::*)() const) & LowState::motor_state_serial,
                      (void(LowState::*)(const std::vector<MotorState> &)) & LowState::motor_state_serial)
        .def("__eq__", &LowState::operator==)
        .def("__ne__", &LowState::operator!=);

    py::class_<MotorCmd>(m, "MotorCmd")
        .def(py::init<>())
        .def(py::init<const MotorCmd &>())
        .def_property("mode",
                      (uint8_t(MotorCmd::*)() const) & MotorCmd::mode,
                      (void(MotorCmd::*)(uint8_t)) & MotorCmd::mode)
        .def_property("q",
                      (float(MotorCmd::*)() const) & MotorCmd::q,
                      (void(MotorCmd::*)(float)) & MotorCmd::q)
        .def_property("dq",
                      (float(MotorCmd::*)() const) & MotorCmd::dq,
                      (void(MotorCmd::*)(float)) & MotorCmd::dq)
        .def_property("tau",
                      (float(MotorCmd::*)() const) & MotorCmd::tau,
                      (void(MotorCmd::*)(float)) & MotorCmd::tau)
        .def_property("kp",
                      (float(MotorCmd::*)() const) & MotorCmd::kp,
                      (void(MotorCmd::*)(float)) & MotorCmd::kp)
        .def_property("kd",
                      (float(MotorCmd::*)() const) & MotorCmd::kd,
                      (void(MotorCmd::*)(float)) & MotorCmd::kd)
        .def_property("weight",
                      (float(MotorCmd::*)() const) & MotorCmd::weight,
                      (void(MotorCmd::*)(float)) & MotorCmd::weight)
        .def("__eq__", &MotorCmd::operator==)
        .def("__ne__", &MotorCmd::operator!=);

    py::enum_<CmdType>(m, "LowCmdType")
        .value("PARALLEL", CmdType::PARALLEL)
        .value("SERIAL", CmdType::SERIAL)
        .export_values();

    py::class_<LowCmd>(m, "LowCmd")
        .def(py::init<>())
        .def(py::init<const LowCmd &>())
        .def_property("cmd_type",
                      (CmdType(LowCmd::*)() const) & LowCmd::cmd_type,
                      (void(LowCmd::*)(CmdType)) & LowCmd::cmd_type)
        .def_property("motor_cmd",
                      (const std::vector<MotorCmd> &(LowCmd::*)() const) & LowCmd::motor_cmd,
                      (void(LowCmd::*)(const std::vector<MotorCmd> &)) & LowCmd::motor_cmd)
        .def("__eq__", &LowCmd::operator==)
        .def("__ne__", &LowCmd::operator!=);

    py::class_<robot::b1::B1LowStateSubscriber, std::shared_ptr<robot::b1::B1LowStateSubscriber>>(m, "B1LowStateSubscriber")
        .def(py::init<const py::function &>(), py::arg("handler"), R"pbdoc(
                 /**
                 * @brief init low state subscriber with callback handler
                 *
                 * @param handler callback handler of low state, the handler should accept one parameter of type LowState
                 *
                 */
            )pbdoc")
        .def("InitChannel", &robot::b1::B1LowStateSubscriber::InitChannel, "Init low state subscription channel")
        .def("CloseChannel", &robot::b1::B1LowStateSubscriber::CloseChannel, "Close low state subscription channel")
        .def("GetChannelName", &robot::b1::B1LowStateSubscriber::GetChannelName, "Get low state subscription channel name");

    py::class_<robot::b1::B1LowHandDataScriber, std::shared_ptr<robot::b1::B1LowHandDataScriber>>(m, "B1LowHandDataScriber")
        .def(py::init<const py::function &>(), py::arg("handler"), R"pbdoc(
                 /**
                 * @brief init hand data subscriber with callback handler
                 *
                 * @param handler callback handler of hand data, the handler should accept one parameter of type LowState
                 *
                 */
            )pbdoc")
        .def("InitChannel", &robot::b1::B1LowHandDataScriber::InitChannel, "Init low state subscription channel")
        .def("CloseChannel", &robot::b1::B1LowHandDataScriber::CloseChannel, "Close low state subscription channel")
        .def("GetChannelName", &robot::b1::B1LowHandDataScriber::GetChannelName, "Get low state subscription channel name");

    py::class_<robot::b1::B1LowCmdPublisher>(m, "B1LowCmdPublisher")
        .def(py::init<>())
        .def("InitChannel", &robot::b1::B1LowCmdPublisher::InitChannel, "Init low cmd publication channel")
        .def("Write", &robot::b1::B1LowCmdPublisher::Write, py::arg("msg"), R"pbdoc(
                 /**
                 * @brief write low cmd message into channel, i.e. publish low cmd message
                 *
                 * @param msg LowCmd
                 *
                 */
            )pbdoc")
        .def("CloseChannel", &robot::b1::B1LowCmdPublisher::CloseChannel, "Close low cmd publication channel")
        .def("GetChannelName", &robot::b1::B1LowCmdPublisher::GetChannelName, "Get low cmd publication channel name");

    py::class_<Odometer>(m, "Odometer")
        .def(py::init<>())
        .def_property("x",
                      (float(Odometer::*)() const) & Odometer::x,
                      (void(Odometer::*)(float)) & Odometer::x)
        .def_property("y",
                      (float(Odometer::*)() const) & Odometer::y,
                      (void(Odometer::*)(float)) & Odometer::y)
        .def_property("theta",
                      (float(Odometer::*)() const) & Odometer::theta,
                      (void(Odometer::*)(float)) & Odometer::theta);

    py::class_<robot::b1::B1OdometerStateSubscriber, std::shared_ptr<robot::b1::B1OdometerStateSubscriber>>(m, "B1OdometerStateSubscriber")
        .def(py::init<const py::function &>(), py::arg("handler"), R"pbdoc(
                 /**
                 * @brief init odometer state subscriber with callback handler
                 *
                 * @param handler callback handler of odom state, the handler should accept one parameter of type Odometer
                 *
                 */
            )pbdoc")
        .def("InitChannel", &robot::b1::B1OdometerStateSubscriber::InitChannel, "Init odometer subscription channel")
        .def("CloseChannel", &robot::b1::B1OdometerStateSubscriber::CloseChannel, "Close odometer subscription channel")
        .def("GetChannelName", &robot::b1::B1OdometerStateSubscriber::GetChannelName, "Get odometer subscription channel name");

    py::class_<HandReplyParam>(m, "HandReplyParam")
        .def(py::init<>())
        .def(py::init<const HandReplyParam &>())
        .def_property("angle",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::angle,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::angle)
        .def_property("force",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::force,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::force)
        .def_property("current",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::current,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::current)
        .def_property("error",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::error,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::error)
        .def_property("status",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::status,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::status)
        .def_property("temp",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::temp,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::temp)
        .def_property("seq",
                      (int32_t(HandReplyParam::*)() const) & HandReplyParam::seq,
                      (int32_t & (HandReplyParam::*)()) & HandReplyParam::seq)
        .def("__eq__", &HandReplyParam::operator==)
        .def("__ne__", &HandReplyParam::operator!=);

    py::class_<HandReplyData>(m, "HandReplyData")
        .def(py::init<>())
        .def(py::init<const HandReplyData &>())
        .def_property("hand_index",
                      (int32_t(HandReplyData::*)() const) & HandReplyData::hand_index,
                      (int32_t & (HandReplyData::*)()) & HandReplyData::hand_index)
        .def_property("hand_type",
                      (int32_t(HandReplyData::*)() const) & HandReplyData::hand_type,
                      (int32_t & (HandReplyData::*)()) & HandReplyData::hand_type)
        .def_property("hand_data",
                      (const std::vector<HandReplyParam> &(HandReplyData::*)() const) & HandReplyData::hand_data,
                      (void(HandReplyData::*)(const std::vector<HandReplyParam> &)) & HandReplyData::hand_data)
        .def("__eq__", &HandReplyData::operator==)
        .def("__ne__", &HandReplyData::operator!=);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
