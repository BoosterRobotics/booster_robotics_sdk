#ifndef __BOOSTER_ROBOTICS_SDK_B1_CLIENT_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_CLIENT_HPP__

#include <memory>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <unordered_map>

#include <booster/robot/channel/channel_publisher.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster/robot/rpc/response.hpp>
#include <booster/robot/rpc/request.hpp>
#include <booster/robot/rpc/msg/bs_rpc_req_msg.h>
#include <booster/robot/rpc/msg/bs_rpc_resp_msg.h>

namespace booster {
namespace robot {

class RpcClient {
public:
    RpcClient() = default;
    ~RpcClient() = default;

    void Init(const std::string &channel_name);
    Response SendApiRequest(const Request &req, int64_t timeout_ms = 1000);

    void Stop();

    std::string GenUuid();

private:
    void DdsSubMsgHandler(const void *msg);

    std::mutex mutex_;
    std::unordered_map<std::string, std::pair<Response, std::unique_ptr<std::condition_variable>>> resp_map_;

    std::shared_ptr<ChannelPublisher<BsRpcReqMsg>> channel_publisher_;
    std::shared_ptr<ChannelSubscriber<BsRpcRespMsg>> channel_subscriber_;
};

}
} // namespace booster::robot

#endif // __BOOSTER_ROBOTICS_SDK_B1_CLIENT_HPP__