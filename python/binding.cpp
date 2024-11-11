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
#include "booster/robot/common/robot_mode.hpp"
#include "booster/robot/channel/channel_factory.hpp"
#include "booster/robot/rpc/msg/bs_rpc_resp_msg.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
namespace robot = booster::robot;

namespace booster::robot::b1 {
class B1LowStateSubscriber {
public:
    B1LowStateSubscriber(const py::function &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<booster_interface::msg::LowState>(channel_name_, [this](const void *msg) {
            py::gil_scoped_acquire acquire;
            const booster_interface::msg::LowState *low_state_msg = static_cast<const booster_interface::msg::LowState *>(msg);
            py_handler_(low_state_msg);
        });
    }

    void CloseChannel() {
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

class B1LowCmdPublisher {
public:
    explicit B1LowCmdPublisher() {
        channel_name_ = kTopicJointCtrl;
    }

    void InitChannel() {
        channel_ptr_ = ChannelFactory::Instance()->CreateSendChannel<booster_interface::msg::LowCmd>(channel_name_);
    }

    bool Write(booster_interface::msg::LowCmd *msg) {
        if (channel_ptr_) {
            return channel_ptr_->Write(msg);
        }
        return false;
    }

    void CloseChannel() {
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
    ChannelPtr<booster_interface::msg::LowCmd> channel_ptr_;
};

class B1OdometerStateSubscriber {
public:
    B1OdometerStateSubscriber(const py::function &py_handler) :
        py_handler_(py_handler) {
    }

    void InitChannel() {
        channel_ptr_ = booster::robot::ChannelFactory::Instance()->CreateRecvChannel<booster_interface::msg::Odometer>(channel_name_, [this](const void *msg) {
            py::gil_scoped_acquire acquire;
            const booster_interface::msg::Odometer *low_state_msg = static_cast<const booster_interface::msg::Odometer *>(msg);
            py_handler_(low_state_msg);
        });
    }

    void CloseChannel() {
        if (channel_ptr_) {
            booster::robot::ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    ChannelPtr<booster_interface::msg::Odometer> channel_ptr_;
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
        .def_static("Instance", &robot::ChannelFactory::Instance, py::return_value_policy::reference, "get the singleton instance of channel factory")
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

    py::class_<robot::b1::B1LocoClient>(m, "B1LocoClient")
        .def(py::init<>())
        .def("Init", py::overload_cast<>(&robot::b1::B1LocoClient::Init))
        .def("Init", py::overload_cast<const std::string &>(&robot::b1::B1LocoClient::Init))
        .def("SendApiRequest", &robot::b1::B1LocoClient::SendApiRequest, py::arg("api_id"), py::arg("param"))
        .def("ChangeMode", &robot::b1::B1LocoClient::ChangeMode, py::arg("mode"))
        .def("Move", &robot::b1::B1LocoClient::Move, py::arg("vx"), py::arg("vy"), py::arg("vyaw"))
        .def("RotateHead", &robot::b1::B1LocoClient::RotateHead, py::arg("pitch"), py::arg("yaw"))
        .def("WaveHand", &robot::b1::B1LocoClient::WaveHand);

    py::class_<booster_interface::msg::ImuState>(m, "ImuState")
        .def(py::init<>())
        .def(py::init<const booster_interface::msg::ImuState &>())
        .def_property("rpy",
                      (const std::array<float, 3> &(booster_interface::msg::ImuState::*)() const) & booster_interface::msg::ImuState::rpy,
                      (void(booster_interface::msg::ImuState::*)(const std::array<float, 3> &)) & booster_interface::msg::ImuState::rpy)
        .def_property("gyro",
                      (const std::array<float, 3> &(booster_interface::msg::ImuState::*)() const) & booster_interface::msg::ImuState::gyro,
                      (void(booster_interface::msg::ImuState::*)(const std::array<float, 3> &)) & booster_interface::msg::ImuState::gyro)
        .def_property("acc",
                      (const std::array<float, 3> &(booster_interface::msg::ImuState::*)() const) & booster_interface::msg::ImuState::acc,
                      (void(booster_interface::msg::ImuState::*)(const std::array<float, 3> &)) & booster_interface::msg::ImuState::acc)
        .def("__eq__", &booster_interface::msg::ImuState::operator==)
        .def("__ne__", &booster_interface::msg::ImuState::operator!=);

    py::class_<booster_interface::msg::MotorState>(m, "MotorState")
        .def(py::init<>())
        .def(py::init<const booster_interface::msg::MotorState &>())
        .def_property("mode",
                      (uint8_t(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::mode,
                      (uint8_t & (booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::mode)
        .def_property("q",
                      (float(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::q,
                      (float &(booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::q)
        .def_property("dq",
                      (float(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::dq,
                      (float &(booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::dq)
        .def_property("ddq",
                      (float(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::ddq,
                      (float &(booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::ddq)
        .def_property("tau_est",
                      (float(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::tau_est,
                      (float &(booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::tau_est)
        .def_property("temperature",
                      (uint8_t(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::temperature,
                      (uint8_t & (booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::temperature)
        .def_property("lost",
                      (uint32_t(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::lost,
                      (uint32_t & (booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::lost)
        .def_property("reserve",
                      (const std::array<uint32_t, 2> &(booster_interface::msg::MotorState::*)() const) & booster_interface::msg::MotorState::reserve,
                      (std::array<uint32_t, 2> & (booster_interface::msg::MotorState::*)()) & booster_interface::msg::MotorState::reserve)
        .def("__eq__", &booster_interface::msg::MotorState::operator==)
        .def("__ne__", &booster_interface::msg::MotorState::operator!=);

    py::class_<booster_interface::msg::LowState>(m, "LowState")
        .def(py::init<>())
        .def(py::init<const booster_interface::msg::LowState &>())
        .def_property("imu_state",
                      (const booster_interface::msg::ImuState &(booster_interface::msg::LowState::*)() const) & booster_interface::msg::LowState::imu_state,
                      (void(booster_interface::msg::LowState::*)(const booster_interface::msg::ImuState &)) & booster_interface::msg::LowState::imu_state)
        .def_property("motor_state_parallel",
                      (const std::vector<booster_interface::msg::MotorState> &(booster_interface::msg::LowState::*)() const) & booster_interface::msg::LowState::motor_state_parallel,
                      (void(booster_interface::msg::LowState::*)(const std::vector<booster_interface::msg::MotorState> &)) & booster_interface::msg::LowState::motor_state_parallel)
        .def_property("motor_state_serial",
                      (const std::vector<booster_interface::msg::MotorState> &(booster_interface::msg::LowState::*)() const) & booster_interface::msg::LowState::motor_state_serial,
                      (void(booster_interface::msg::LowState::*)(const std::vector<booster_interface::msg::MotorState> &)) & booster_interface::msg::LowState::motor_state_serial)
        .def("__eq__", &booster_interface::msg::LowState::operator==)
        .def("__ne__", &booster_interface::msg::LowState::operator!=);

    py::class_<booster_interface::msg::MotorCmd>(m, "MotorCmd")
        .def(py::init<>())
        .def(py::init<const booster_interface::msg::MotorCmd &>())
        .def_property("mode",
                      (uint8_t(booster_interface::msg::MotorCmd::*)() const) & booster_interface::msg::MotorCmd::mode,
                      (void(booster_interface::msg::MotorCmd::*)(uint8_t)) & booster_interface::msg::MotorCmd::mode)
        .def_property("q",
                      (float(booster_interface::msg::MotorCmd::*)() const) & booster_interface::msg::MotorCmd::q,
                      (void(booster_interface::msg::MotorCmd::*)(float)) & booster_interface::msg::MotorCmd::q)
        .def_property("dq",
                      (float(booster_interface::msg::MotorCmd::*)() const) & booster_interface::msg::MotorCmd::dq,
                      (void(booster_interface::msg::MotorCmd::*)(float)) & booster_interface::msg::MotorCmd::dq)
        .def_property("tau",
                      (float(booster_interface::msg::MotorCmd::*)() const) & booster_interface::msg::MotorCmd::tau,
                      (void(booster_interface::msg::MotorCmd::*)(float)) & booster_interface::msg::MotorCmd::tau)
        .def_property("kp",
                      (float(booster_interface::msg::MotorCmd::*)() const) & booster_interface::msg::MotorCmd::kp,
                      (void(booster_interface::msg::MotorCmd::*)(float)) & booster_interface::msg::MotorCmd::kp)
        .def_property("kd",
                      (float(booster_interface::msg::MotorCmd::*)() const) & booster_interface::msg::MotorCmd::kd,
                      (void(booster_interface::msg::MotorCmd::*)(float)) & booster_interface::msg::MotorCmd::kd)
        .def_property("weight",
                      (float(booster_interface::msg::MotorCmd::*)() const) & booster_interface::msg::MotorCmd::weight,
                      (void(booster_interface::msg::MotorCmd::*)(float)) & booster_interface::msg::MotorCmd::weight)
        .def("__eq__", &booster_interface::msg::MotorCmd::operator==)
        .def("__ne__", &booster_interface::msg::MotorCmd::operator!=);

    py::enum_<booster_interface::msg::CmdType>(m, "LowCmdType")
        .value("PARALLEL", booster_interface::msg::CmdType::PARALLEL)
        .value("SERIAL", booster_interface::msg::CmdType::SERIAL)
        .export_values();

    py::class_<booster_interface::msg::LowCmd>(m, "LowCmd")
        .def(py::init<>())
        .def(py::init<const booster_interface::msg::LowCmd &>())
        .def_property("cmd_type",
                      (booster_interface::msg::CmdType(booster_interface::msg::LowCmd::*)() const) & booster_interface::msg::LowCmd::cmd_type,
                      (void(booster_interface::msg::LowCmd::*)(booster_interface::msg::CmdType)) & booster_interface::msg::LowCmd::cmd_type)
        .def_property("motor_cmd",
                      (const std::vector<booster_interface::msg::MotorCmd> &(booster_interface::msg::LowCmd::*)() const) & booster_interface::msg::LowCmd::motor_cmd,
                      (void(booster_interface::msg::LowCmd::*)(const std::vector<booster_interface::msg::MotorCmd> &)) & booster_interface::msg::LowCmd::motor_cmd)
        .def("__eq__", &booster_interface::msg::LowCmd::operator==)
        .def("__ne__", &booster_interface::msg::LowCmd::operator!=);

    py::class_<robot::b1::B1LowStateSubscriber>(m, "B1LowStateSubscriber")
        .def(py::init<const py::function &>())
        .def("InitChannel", &robot::b1::B1LowStateSubscriber::InitChannel)
        .def("CloseChannel", &robot::b1::B1LowStateSubscriber::CloseChannel)
        .def("GetChannelName", &robot::b1::B1LowStateSubscriber::GetChannelName);

    py::class_<robot::b1::B1LowCmdPublisher>(m, "B1LowCmdPublisher")
        .def(py::init<>())
        .def("InitChannel", &robot::b1::B1LowCmdPublisher::InitChannel)
        .def("Write", &robot::b1::B1LowCmdPublisher::Write, py::arg("msg"))
        .def("CloseChannel", &robot::b1::B1LowCmdPublisher::CloseChannel)
        .def("GetChannelName", &robot::b1::B1LowCmdPublisher::GetChannelName);

    py::class_<booster_interface::msg::Odometer>(m, "Odometer")
        .def(py::init<>())
        .def_property("x",
                      (float(booster_interface::msg::Odometer::*)() const) & booster_interface::msg::Odometer::x,
                      (void(booster_interface::msg::Odometer::*)(float)) & booster_interface::msg::Odometer::x)
        .def_property("y",
                      (float(booster_interface::msg::Odometer::*)() const) & booster_interface::msg::Odometer::y,
                      (void(booster_interface::msg::Odometer::*)(float)) & booster_interface::msg::Odometer::y)
        .def_property("theta",
                        (float(booster_interface::msg::Odometer::*)() const) & booster_interface::msg::Odometer::theta,
                        (void(booster_interface::msg::Odometer::*)(float)) & booster_interface::msg::Odometer::theta);
    
    py::class_<robot::b1::B1OdometerStateSubscriber>(m, "B1OdometerStateSubscriber")
        .def(py::init<const py::function &>())
        .def("InitChannel", &robot::b1::B1OdometerStateSubscriber::InitChannel)
        .def("CloseChannel", &robot::b1::B1OdometerStateSubscriber::CloseChannel)
        .def("GetChannelName", &robot::b1::B1OdometerStateSubscriber::GetChannelName);


#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}