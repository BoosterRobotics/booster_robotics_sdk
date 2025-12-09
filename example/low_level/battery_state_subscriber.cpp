#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/b1/BatteryState.h>

#include <thread>
#include <chrono>
#include <iostream>

#define TOPIC "rt/battery_state"

using namespace booster::robot;
using namespace booster::common;
using namespace booster_interface::msg;

void Handler(const void *msg)
{
    const BatteryState* bat = static_cast<const BatteryState*>(msg);

    std::cout << "------- Battery State -------" << std::endl;
    std::cout << "  Voltage        : " << bat->voltage() << " V" << std::endl;
    std::cout << "  Current        : " << bat->current() << " A" << std::endl;
    std::cout << "  SOC            : " << bat->soc() << " %" << std::endl;
    std::cout << "  Avg Voltage    : " << bat->average_voltage() << " V" << std::endl;
    std::cout << "------------------------------" << std::endl;
}

int main()
{
    ChannelFactory::Instance()->Init(0);

    ChannelSubscriber<BatteryState> sub(TOPIC, Handler);
    sub.InitChannel();

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
