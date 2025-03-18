#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/idl/b1/HandReplyData.h>

#include <thread>
#include <chrono>
#include <iostream>

#define TOPIC "rt/booster_hand_data"

using namespace booster::robot;
using namespace booster::common;
using namespace booster_interface::msg;

void Handler(const void *msg) {
    const HandReplyData *hand_data = static_cast<const HandReplyData *>(msg);

    for (auto data : hand_data->hand_data())
        std::cout << "seq " << data.seq() << " angle:" <<  data.angle() << " force:" <<  data.force() << " current:" <<  data.current() << " status:" <<  data.status() << " temp:" <<  data.temp() << " error:" <<  data.error() << std::endl;
    std::cout << "hand_data->hand_index:" << hand_data->hand_index() << " hand_data->hand_type:" << hand_data->hand_type() << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;

}

int main() {
    ChannelFactory::Instance()->Init(0);
    ChannelSubscriber<HandReplyData> channel_subscriber(TOPIC, Handler);
    channel_subscriber.InitChannel();
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
