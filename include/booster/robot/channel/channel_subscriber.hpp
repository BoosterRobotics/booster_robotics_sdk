#ifndef __BOOSTER_ROBOTICS_SDK_CHANNEL_SUBSCRIBER_HPP__
#define __BOOSTER_ROBOTICS_SDK_CHANNEL_SUBSCRIBER_HPP__

#include <iostream>

#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/rpc/response.hpp>

namespace booster {
namespace robot {

template <typename MSG>
class ChannelSubscriber {
public:
    explicit ChannelSubscriber(const std::string &channel_name, bool reliable = false) :
        channel_name_(channel_name),
        reliable_(reliable) {
    }

    template <class F,
              std::enable_if_t<
                  std::is_invocable_r_v<void, F, const void *>, int> = 0>
    explicit ChannelSubscriber(const std::string &channel_name,
                               F &&handler,
                               bool reliable = false) :
        channel_name_(channel_name),
        handler_(std::forward<F>(handler)),
        reliable_(reliable) {
    }

    void InitChannel(const std::function<void(const void *)> &handler) {
        handler_ = handler;
        InitChannel();
    }

    void InitChannel() {
        if (handler_) {
            std::cout << "ChannelSubscriber::InitChannel: setting reliability: " << reliable_ << std::endl;
            channel_ptr_ = ChannelFactory::Instance()->CreateRecvChannel<MSG>(channel_name_, handler_, reliable_);
        } else {
            std::cerr << "ChannelSubscriber::InitChannel: handler is not set" << std::endl;
        }
    }

    void CloseChannel() {
        if (channel_ptr_) {
            ChannelFactory::Instance()->CloseReader(channel_name_);
            channel_ptr_.reset();
        }
    }

    const std::string &GetChannelName() const {
        return channel_name_;
    }

private:
    std::string channel_name_;
    ChannelPtr<MSG> channel_ptr_;
    std::function<void(const void *)> handler_;
    bool reliable_{false};
};

}
} // namespace booster::robot

#endif