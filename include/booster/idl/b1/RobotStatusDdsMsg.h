// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file DeviceGateway.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool fastddsgen.
 */

#ifndef _FAST_DDS_GENERATED_BOOSTER_MSG_DEVICEGATEWAY_H_
#define _FAST_DDS_GENERATED_BOOSTER_MSG_DEVICEGATEWAY_H_

#include <array>
#include <bitset>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include <fastcdr/cdr/fixed_size_string.hpp>
#include <fastcdr/xcdr/external.hpp>
#include <fastcdr/xcdr/optional.hpp>

#include "RobotDdsBatteryStatus.h"
#include "RobotDdsImuStatus.h"
#include "RobotDdsJointStatus.h"

/*--------------------pub sub ------------------*/
#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/rtps/common/InstanceHandle.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastrtps/utils/md5.h>

#if !defined(GEN_API_VER) || (GEN_API_VER != 2)
#error \
    Generated DeviceGateway is not compatible with current installed Fast DDS. Please, regenerate it with fastddsgen.
#endif  // GEN_API_VER
/*--------------------pub sub ------------------*/

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(DEVICEGATEWAY_SOURCE)
#define DEVICEGATEWAY_DllAPI __declspec( dllexport )
#else
#define DEVICEGATEWAY_DllAPI __declspec( dllimport )
#endif // DEVICEGATEWAY_SOURCE
#else
#define DEVICEGATEWAY_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define DEVICEGATEWAY_DllAPI
#endif // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
class CdrSizeCalculator;
} // namespace fastcdr
} // namespace eprosima



namespace booster {

namespace msg {





/*!
 * @brief This class represents the structure RobotStatusDdsMsg defined by the user in the IDL file.
 * @ingroup DeviceGateway
 */
class RobotStatusDdsMsg  : public eprosima::fastdds::dds::TopicDataType
{
public:

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object booster::msg::RobotStatusDdsMsg that will be copied.
     */
    eProsima_user_DllExport RobotStatusDdsMsg(
            const RobotStatusDdsMsg& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object booster::msg::RobotStatusDdsMsg that will be copied.
     */
    eProsima_user_DllExport RobotStatusDdsMsg(
            RobotStatusDdsMsg&& x) noexcept;

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object booster::msg::RobotStatusDdsMsg that will be copied.
     */
    eProsima_user_DllExport RobotStatusDdsMsg& operator =(
            const RobotStatusDdsMsg& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object booster::msg::RobotStatusDdsMsg that will be copied.
     */
    eProsima_user_DllExport RobotStatusDdsMsg& operator =(
            RobotStatusDdsMsg&& x) noexcept;

    /*!
     * @brief Comparison operator.
     * @param x booster::msg::RobotStatusDdsMsg object to compare.
     */
    eProsima_user_DllExport bool operator ==(
            const RobotStatusDdsMsg& x) const;

    /*!
     * @brief Comparison operator.
     * @param x booster::msg::RobotStatusDdsMsg object to compare.
     */
    eProsima_user_DllExport bool operator !=(
            const RobotStatusDdsMsg& x) const;

    /*!
     * @brief This function copies the value in member joint_vec
     * @param _joint_vec New value to be copied in member joint_vec
     */
    eProsima_user_DllExport void joint_vec(
            const std::vector<booster::msg::RobotDdsJointStatus>& _joint_vec);

    /*!
     * @brief This function moves the value in member joint_vec
     * @param _joint_vec New value to be moved in member joint_vec
     */
    eProsima_user_DllExport void joint_vec(
            std::vector<booster::msg::RobotDdsJointStatus>&& _joint_vec);

    /*!
     * @brief This function returns a constant reference to member joint_vec
     * @return Constant reference to member joint_vec
     */
    eProsima_user_DllExport const std::vector<booster::msg::RobotDdsJointStatus>& joint_vec() const;

    /*!
     * @brief This function returns a reference to member joint_vec
     * @return Reference to member joint_vec
     */
    eProsima_user_DllExport std::vector<booster::msg::RobotDdsJointStatus>& joint_vec();


    /*!
     * @brief This function copies the value in member imu_vec
     * @param _imu_vec New value to be copied in member imu_vec
     */
    eProsima_user_DllExport void imu_vec(
            const std::vector<booster::msg::RobotDdsImuStatus>& _imu_vec);

    /*!
     * @brief This function moves the value in member imu_vec
     * @param _imu_vec New value to be moved in member imu_vec
     */
    eProsima_user_DllExport void imu_vec(
            std::vector<booster::msg::RobotDdsImuStatus>&& _imu_vec);

    /*!
     * @brief This function returns a constant reference to member imu_vec
     * @return Constant reference to member imu_vec
     */
    eProsima_user_DllExport const std::vector<booster::msg::RobotDdsImuStatus>& imu_vec() const;

    /*!
     * @brief This function returns a reference to member imu_vec
     * @return Reference to member imu_vec
     */
    eProsima_user_DllExport std::vector<booster::msg::RobotDdsImuStatus>& imu_vec();


    /*!
     * @brief This function copies the value in member battery_vec
     * @param _battery_vec New value to be copied in member battery_vec
     */
    eProsima_user_DllExport void battery_vec(
            const std::vector<booster::msg::RobotDdsBatteryStatus>& _battery_vec);

    /*!
     * @brief This function moves the value in member battery_vec
     * @param _battery_vec New value to be moved in member battery_vec
     */
    eProsima_user_DllExport void battery_vec(
            std::vector<booster::msg::RobotDdsBatteryStatus>&& _battery_vec);

    /*!
     * @brief This function returns a constant reference to member battery_vec
     * @return Constant reference to member battery_vec
     */
    eProsima_user_DllExport const std::vector<booster::msg::RobotDdsBatteryStatus>& battery_vec() const;

    /*!
     * @brief This function returns a reference to member battery_vec
     * @return Reference to member battery_vec
     */
    eProsima_user_DllExport std::vector<booster::msg::RobotDdsBatteryStatus>& battery_vec();

/*--------------------pub sub ------------------*/
    typedef RobotStatusDdsMsg type;

    eProsima_user_DllExport RobotStatusDdsMsg();

    eProsima_user_DllExport ~RobotStatusDdsMsg() override;

    eProsima_user_DllExport bool serialize(
            void* data,
            eprosima::fastrtps::rtps::SerializedPayload_t* payload) override
    {
        return serialize(data, payload, eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
    }

    eProsima_user_DllExport bool serialize(
            void* data,
            eprosima::fastrtps::rtps::SerializedPayload_t* payload,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation) override;

    eProsima_user_DllExport bool deserialize(
            eprosima::fastrtps::rtps::SerializedPayload_t* payload,
            void* data) override;

    eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
            void* data) override
    {
        return getSerializedSizeProvider(data, eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
    }

    eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
            void* data,
            eprosima::fastdds::dds::DataRepresentationId_t data_representation) override;

    eProsima_user_DllExport bool getKey(
            void* data,
            eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
            bool force_md5 = false) override;

    eProsima_user_DllExport void* createData() override;

    eProsima_user_DllExport void deleteData(
            void* data) override;

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
    eProsima_user_DllExport inline bool is_bounded() const override
    {
        return false;
    }

#endif  // TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
    eProsima_user_DllExport inline bool is_plain() const override
    {
        return false;
    }

    eProsima_user_DllExport inline bool is_plain(
        eprosima::fastdds::dds::DataRepresentationId_t data_representation) const override
    {
        static_cast<void>(data_representation);
        return false;
    }

#endif  // TOPIC_DATA_TYPE_API_HAS_IS_PLAIN

#ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
    eProsima_user_DllExport inline bool construct_sample(
            void* memory) const override
    {
        static_cast<void>(memory);
        return false;
    }

#endif  // TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE

    MD5 m_md5;
    unsigned char* m_keyBuffer;

/*--------------------pub sub ------------------*/

private:

    std::vector<booster::msg::RobotDdsJointStatus> m_joint_vec;
    std::vector<booster::msg::RobotDdsImuStatus> m_imu_vec;
    std::vector<booster::msg::RobotDdsBatteryStatus> m_battery_vec;

};

} // namespace msg

} // namespace booster

#endif // _FAST_DDS_GENERATED_BOOSTER_MSG_DEVICEGATEWAY_H_


