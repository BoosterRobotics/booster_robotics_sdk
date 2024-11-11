#ifndef __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__
#define __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__

#include <memory>

#include <booster/robot/rpc/rpc_client.hpp>

#include "b1_loco_api.hpp"

using namespace booster::robot;

namespace booster {
namespace robot {
namespace b1 {

class B1LocoClient {
public:
    B1LocoClient() = default;
    ~B1LocoClient() = default;

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
    int32_t SendApiRequest(LocoApiId api_id, const std::string &param);

    /**
     * @brief Change robot mode
     * 
     * @param mode robot mode, options are: kDamping, kPrepare, kWalking
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t ChangeMode(RobotMode mode) {
        ChangeModeParameter change_mode(mode);
        std::string param = change_mode.ToJson().dump();
        return SendApiRequest(LocoApiId::kChangeMode, param);
    }

    /**
     * @brief Move robot
     * 
     * @param vx linear velocity in x direction, unit: m/s
     * @param vy linear velocity in y direction, unit: m/s
     * @param vyaw angular velocity, unit: rad/s
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t Move(float vx, float vy, float vyaw) {
        MoveParameter move(vx, vy, vyaw);
        std::string param = move.ToJson().dump();
        return SendApiRequest(LocoApiId::kMove, param);
    }

    /**
     * @brief Robot rotates its head
     *
     * @param pitch pitch angle, unit: rad
     * @param yaw yaw angle, unit: rad
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t RotateHead(float pitch, float yaw) {
        RotateHeadParameter head_ctrl(pitch, yaw);
        std::string param = head_ctrl.ToJson().dump();
        return SendApiRequest(LocoApiId::kRotateHead, param);
    }

    /**
     * @brief Robot waves its hand
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t WaveHand(HandAction action) {
        WaveHandParameter wave_hand(kRightHand, action);
        std::string param = wave_hand.ToJson().dump();
        return SendApiRequest(LocoApiId::kWaveHand, param);
    }

    /**
     * @brief The robot rotates its head at an appropriate speed, 
     * and the head does not rotate beyond its limit
     *
     * @param pitch_direction pitch direction, options are: 1(left), 0(no rotation), -1(right)
     * @param yaw_direction yaw direction, options are: 1(down), 0(no rotation), -1(up)
     *
     * @return 0 if success, otherwise return error code
     */
    int32_t RotateHeadWithDirection(int pitch_direction, int yaw_direction) {
        RotateHeadWithDirectionParameter head_ctrl(pitch_direction, yaw_direction);
        std::string param = head_ctrl.ToJson().dump();
        return SendApiRequest(LocoApiId::kRotateHeadWithDirection, param);
    }

    /**
     * @brief The robot lies down on its back
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t LieDown() {
        return SendApiRequest(LocoApiId::kLieDown, "");
    }

    /**
     * @brief The robot gets up from a position lying on its back
     * 
     * @return 0 if success, otherwise return error code
     */
    int32_t GetUp() {
        return SendApiRequest(LocoApiId::kGetUp, "");
    }

private:
    std::shared_ptr<RpcClient> rpc_client_;
};

}
}
} // namespace booster::robot::b1

#endif // __BOOSTER_ROBOTICS_SDK_B1_LOCO_CLIENT_HPP__