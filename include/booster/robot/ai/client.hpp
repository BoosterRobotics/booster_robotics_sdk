#pragma once

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>

#include "api.hpp"

using namespace booster::robot;

namespace booster {
namespace robot {

class AiClient {
public:
    AiClient() = default;
    ~AiClient() = default;

    void Init();

    void Init(const std::string &robot_name);
    /**
     * @brief Send API request to B1 robot
     *
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequest(AiApiId api_id, const std::string &param);

    /**
     * @brief Send API request to B1 robot with response
     *
     * @param api_id API_ID, you can find the API_ID in b1_api_const.hpp
     * @param param API parameter
     * @param resp [out] A reference to a Response object where the API's response will be stored.
     * This parameter is modified by the function to contain the result of the API call
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t SendApiRequestWithResponse(AiApiId api_id, const std::string &param, Response &resp);

    /**
     * @brief Start AI chat session with the robot
     *
     * This function sends a request to initiate an AI chat session on the robot.
     *
     * @param param The parameters required to start the AI chat, serialized to JSON.
     *
     * @return 0 if the request is successful, otherwise returns an error code.
     */
    int32_t StartAiChat(const StartAiChatParameter &param) {
        std::string body = param.ToJson().dump();
        return SendApiRequest(AiApiId::kStartAiChat, body);
    }

    /**
     * @brief Stop ai chat
     *
     * @return 0 if success, otherwise return error code
     */

    int32_t StopAiChat() {
        return SendApiRequest(AiApiId::kStopAiChat, "");
    }

    /**
     * @brief Send a speech command to the robot
     *
     * @param param The parameter object containing speech details
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t Speak(const SpeakParameter &param) {
        std::string body = param.ToJson().dump();
        return SendApiRequest(AiApiId::kSpeak, body);
    }

    /**
     * @brief Start face tracking
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t StartFaceTracking() {
        return SendApiRequest(AiApiId::kStartFaceTracking, "");
    }

    /**
     * @brief Start face tracking
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t StopFaceTracking() {
        return SendApiRequest(AiApiId::kStopFaceTracking, "");
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
} // namespace booster::robot
