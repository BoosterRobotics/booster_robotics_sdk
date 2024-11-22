#include <array>
#include <chrono>
#include <iostream>
#include <thread>


#include <booster/idl/b1/LowCmd.h>
#include <booster/idl/b1/MotorCmd.h>
#include <booster/robot/b1/b1_api_const.hpp>
#include <booster/robot/channel/channel_publisher.hpp>


static const std::string kTopicArmSDK = booster::robot::b1::kTopicJointCtrl;
using namespace booster::robot::b1;

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  booster::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  booster::robot::ChannelPublisherPtr< booster_interface::msg::LowCmd>
      arm_sdk_publisher;
  booster_interface::msg::LowCmd msg;

  arm_sdk_publisher.reset(
      new booster::robot::ChannelPublisher<booster_interface::msg::LowCmd>(
          kTopicArmSDK));
  arm_sdk_publisher->InitChannel();

  std::array<JointIndex, 8> arm_joints = {
      JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftElbowPitch,    JointIndex::kLeftElbowYaw,
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightElbowPitch,   JointIndex::kRightElbowYaw};

  float weight = 0.f;
  float weight_rate = 0.2f;

  float kp = 60.f;
  float kd = 1.5f;
  float dq = 0.f;
  float tau_ff = 0.f;

  float control_dt = 0.02f;
  float max_joint_velocity = 0.5f;

  float weight_margin = weight_rate * control_dt;
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time =
      std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  std::array<float, 8> init_pos{};

  std::array<float, 8> target_pos = {0.1f, -1.5,  0.0, -0.2,
                                     0.1f, 1.5, 0.f, 0.2};

  // wait for init
  std::cout << "Press ENTER to init arms ...";
  std::cin.get();

  // set init pos
  std::cout << "Initailizing arms ...";
  float init_time = 5.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  for (size_t i = 0; i < booster::robot::b1::kJointCnt; i++) {
      booster_interface::msg::MotorCmd motor_cmd;
      msg.motor_cmd().push_back(motor_cmd);
  }

  for (int i = 0; i < init_time_steps; ++i) {
    // increase weight
    weight += weight_margin;
    weight = std::clamp(weight, 0.f, 0.5f);
    std::cout << weight << std::endl;

    // set control joints
    for (int j = 0; j < init_pos.size(); ++j) {
      msg.motor_cmd().at(arm_joints.at(j)).q(init_pos.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
      msg.motor_cmd().at(arm_joints.at(j)).weight(weight);
    }

    // send dds msg
    arm_sdk_publisher->Write(&msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done!" << std::endl;

  // wait for control
  std::cout << "Press ENTER to start arm ctrl ..." << std::endl;
  std::cin.get();

  // start control
  std::cout << "Start arm ctrl!" << std::endl;
  float period = 5.f;
  int num_time_steps = static_cast<int>(period / control_dt);

  std::array<float, 8> current_jpos_des{};

  // lift arms up
  for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
    for (int j = 0; j < init_pos.size(); ++j) {
      current_jpos_des.at(j) +=
          std::clamp(target_pos.at(j) - current_jpos_des.at(j),
                     -max_joint_delta, max_joint_delta);
    }

    // set control joints
    for (int j = 0; j < init_pos.size(); ++j) {
      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    // send dds msg
    arm_sdk_publisher->Write(&msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  // put arms down
  for (int i = 0; i < num_time_steps; ++i) {
    // update jpos des
    for (int j = 0; j < init_pos.size(); ++j) {
      current_jpos_des.at(j) +=
          std::clamp(init_pos.at(j) - current_jpos_des.at(j), -max_joint_delta,
                     max_joint_delta);
    }

    // set control joints
    for (int j = 0; j < init_pos.size(); ++j) {
      msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
      msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
      msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
      msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
      msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
    }

    // send dds msg
    arm_sdk_publisher->Write(&msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  // stop control
  std::cout << "Stoping arm ctrl ...";
  float stop_time = 2.0f;
  int stop_time_steps = static_cast<int>(stop_time / control_dt);

  for (int i = 0; i < stop_time_steps; ++i) {
    // increase weight
    weight -= weight_margin;
    weight = std::clamp(weight, 0.f, 1.f);

    // send dds msg
    arm_sdk_publisher->Write(&msg);

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  std::cout << "Done!" << std::endl;

  return 0;
}
