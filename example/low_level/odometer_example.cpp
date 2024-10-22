#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/b1/Odometer.h>
#include <booster/robot/b1/b1_api_const.hpp>

#include <thread>
#include <chrono>
#include <iostream>

using namespace booster::robot;
using namespace booster::common;
using namespace booster::msg;

/*************  ✨ Codeium Command ⭐  *************/
/**
 * @brief Callback function for odometer state subscriber.
 *        This function will be called when a new odometer state message is received.
 *        The message will be printed to console.
 *
 * @param[in] msg The address of the received Odometer message.
 */
/******  c9762cd1-8fe5-4927-b76a-d530bab85865  *******/
void Handler(const void *msg) {
    const Odometer *odom_state_msg = static_cast<const Odometer *>(msg);
    std::cout << "Received message: " << std::endl
              << "x: " << odom_state_msg->x() << ", "
              << "y: " << odom_state_msg->y() << ", "
              << "theta: " << odom_state_msg->theta() << std::endl;
}

int main() {
    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<Odometer> channel_subscriber(booster::robot::b1::kTopicOdometerState, Handler);
    channel_subscriber.InitChannel();
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}