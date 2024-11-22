#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/b1/LowState.h>

#include <thread>
#include <chrono>
#include <iostream>

#define TOPIC "rt/low_state"

using namespace booster::robot;
using namespace booster::common;
using namespace booster_interface::msg;

void Handler(const void *msg) {
    const LowState *low_state_msg = static_cast<const LowState *>(msg);
    std::cout << "Received message: " << std::endl
              << "  serial motor count: " << low_state_msg->motor_state_serial().size() << std::endl
              << "  parallel motor count: " << low_state_msg->motor_state_parallel().size() << std::endl
              << "  imu: "
              << low_state_msg->imu_state().rpy()[0] << ", "
              << low_state_msg->imu_state().rpy()[1] << ", "
              << low_state_msg->imu_state().rpy()[2] << ", "
              << low_state_msg->imu_state().gyro()[0] << ", "
              << low_state_msg->imu_state().gyro()[1] << ", "
              << low_state_msg->imu_state().gyro()[2] << ", "
              << low_state_msg->imu_state().acc()[0] << ", "
              << low_state_msg->imu_state().acc()[1] << ", "
              << low_state_msg->imu_state().acc()[2] << std::endl;
    for(int i = 0; i < low_state_msg->motor_state_serial().size(); i++) {
        std::cout << "  serial motor " << i << ": "
                  << low_state_msg->motor_state_serial()[i].dq() << ", "
                  << low_state_msg->motor_state_serial()[i].ddq() << ", "
                  << low_state_msg->motor_state_serial()[i].tau_est() << ", " << std::endl;
    }
    for(int i = 0; i < low_state_msg->motor_state_parallel().size(); i++) {
        std::cout << "  parallel motor " << i << ": "
                  << low_state_msg->motor_state_parallel()[i].dq() << ", "
                  << low_state_msg->motor_state_parallel()[i].ddq() << ", "
                  << low_state_msg->motor_state_parallel()[i].tau_est() << ", " << std::endl;
    }
}

int main() {
    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<LowState> channel_subscriber(TOPIC, Handler);
    channel_subscriber.InitChannel();
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}