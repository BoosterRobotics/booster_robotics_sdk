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
 * @file ButtonEvent.h
 * This header file contains the declaration of the described types in the IDL
 * file.
 *
 * This file was generated by the tool fastddsgen.
 */

#ifndef _FAST_DDS_GENERATED_BOOSTER_INTERFACE_MSG_BUTTONEVENT_H_
#define _FAST_DDS_GENERATED_BOOSTER_INTERFACE_MSG_BUTTONEVENT_H_

#include <array>
#include <bitset>
#include <cstdint>
#include <fastcdr/cdr/fixed_size_string.hpp>
#include <fastcdr/xcdr/external.hpp>
#include <fastcdr/xcdr/optional.hpp>
#include <map>
#include <string>
#include <vector>

// ------------------------------ Pub Sub Type Start
// ----------------------------
#include <fastdds/rtps/common/InstanceHandle.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastrtps/utils/md5.h>

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>

#include "ButtonEvent.h"

#if !defined(GEN_API_VER) || (GEN_API_VER != 2)
#error \
    Generated ButtonEvent is not compatible with current installed Fast DDS. Please, regenerate it with fastddsgen.
#endif  // GEN_API_VER

// ------------------------------ Pub Sub Type End ----------------------------

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec(dllexport)
#else
#define eProsima_user_DllExport
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define eProsima_user_DllExport
#endif  // _WIN32

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(BUTTONEVENT_SOURCE)
#define BUTTONEVENT_DllAPI __declspec(dllexport)
#else
#define BUTTONEVENT_DllAPI __declspec(dllimport)
#endif  // BUTTONEVENT_SOURCE
#else
#define BUTTONEVENT_DllAPI
#endif  // EPROSIMA_USER_DLL_EXPORT
#else
#define BUTTONEVENT_DllAPI
#endif  // _WIN32

namespace eprosima {
namespace fastcdr {
class Cdr;
class CdrSizeCalculator;
}  // namespace fastcdr
}  // namespace eprosima

namespace booster_interface {

namespace msg {

/*!
 * @brief This class represents the enumeration ButtonEventType defined by the
 * user in the IDL file.
 * @ingroup ButtonEvent
 */
enum ButtonEventType : uint32_t {
  PRESS_DOWN,
  PRESS_UP,
  SINGLE_CLICK,
  DOUBLE_CLICK,
  TRIPLE_CLICK,
  LONG_PRESS_START,
  LONG_PRESS_HOLD,
  LONG_PRESS_END
};

/*!
 * @brief This class represents the structure ButtonEventMsg defined by the user
 * in the IDL file.
 * @ingroup ButtonEvent
 */
class ButtonEventMsg : public eprosima::fastdds::dds::TopicDataType {
 public:
  /*!
   * @brief Copy constructor.
   * @param x Reference to the object booster_interface::msg::ButtonEventMsg
   * that will be copied.
   */
  eProsima_user_DllExport ButtonEventMsg(const ButtonEventMsg& x);

  /*!
   * @brief Move constructor.
   * @param x Reference to the object booster_interface::msg::ButtonEventMsg
   * that will be copied.
   */
  eProsima_user_DllExport ButtonEventMsg(ButtonEventMsg&& x) noexcept;

  /*!
   * @brief Copy assignment.
   * @param x Reference to the object booster_interface::msg::ButtonEventMsg
   * that will be copied.
   */
  eProsima_user_DllExport ButtonEventMsg& operator=(const ButtonEventMsg& x);

  /*!
   * @brief Move assignment.
   * @param x Reference to the object booster_interface::msg::ButtonEventMsg
   * that will be copied.
   */
  eProsima_user_DllExport ButtonEventMsg& operator=(
      ButtonEventMsg&& x) noexcept;

  /*!
   * @brief Comparison operator.
   * @param x booster_interface::msg::ButtonEventMsg object to compare.
   */
  eProsima_user_DllExport bool operator==(const ButtonEventMsg& x) const;

  /*!
   * @brief Comparison operator.
   * @param x booster_interface::msg::ButtonEventMsg object to compare.
   */
  eProsima_user_DllExport bool operator!=(const ButtonEventMsg& x) const;

  /*!
   * @brief This function sets a value in member button
   * @param _button New value for member button
   */
  eProsima_user_DllExport void button(int32_t _button);

  /*!
   * @brief This function returns the value of member button
   * @return Value of member button
   */
  eProsima_user_DllExport int32_t button() const;

  /*!
   * @brief This function returns a reference to member button
   * @return Reference to member button
   */
  eProsima_user_DllExport int32_t& button();

  /*!
   * @brief This function sets a value in member event
   * @param _event New value for member event
   */
  eProsima_user_DllExport void event(
      booster_interface::msg::ButtonEventType _event);

  /*!
   * @brief This function returns the value of member event
   * @return Value of member event
   */
  eProsima_user_DllExport booster_interface::msg::ButtonEventType event() const;

  /*!
   * @brief This function returns a reference to member event
   * @return Reference to member event
   */
  eProsima_user_DllExport booster_interface::msg::ButtonEventType& event();

 private:
  int32_t m_button{0};
  booster_interface::msg::ButtonEventType m_event{
      booster_interface::msg::PRESS_DOWN};

 public:
  typedef ButtonEventMsg type;

  eProsima_user_DllExport ButtonEventMsg();

  eProsima_user_DllExport ~ButtonEventMsg() override;

  eProsima_user_DllExport bool serialize(
      void* data,
      eprosima::fastrtps::rtps::SerializedPayload_t* payload) override {
    return serialize(data, payload,
                     eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
  }

  eProsima_user_DllExport bool serialize(
      void* data, eprosima::fastrtps::rtps::SerializedPayload_t* payload,
      eprosima::fastdds::dds::DataRepresentationId_t data_representation)
      override;

  eProsima_user_DllExport bool deserialize(
      eprosima::fastrtps::rtps::SerializedPayload_t* payload,
      void* data) override;

  eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
      void* data) override {
    return getSerializedSizeProvider(
        data, eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
  }

  eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
      void* data,
      eprosima::fastdds::dds::DataRepresentationId_t data_representation)
      override;

  eProsima_user_DllExport bool getKey(
      void* data, eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
      bool force_md5 = false) override;

  eProsima_user_DllExport void* createData() override;

  eProsima_user_DllExport void deleteData(void* data) override;

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
  eProsima_user_DllExport inline bool is_bounded() const override {
    return false;
  }

#endif  // TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
  eProsima_user_DllExport inline bool is_plain() const override {
    return false;
  }

  eProsima_user_DllExport inline bool is_plain(
      eprosima::fastdds::dds::DataRepresentationId_t data_representation)
      const override {
    static_cast<void>(data_representation);
    return false;
  }

#endif  // TOPIC_DATA_TYPE_API_HAS_IS_PLAIN

#ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
  eProsima_user_DllExport inline bool construct_sample(
      void* memory) const override {
    static_cast<void>(memory);
    return false;
  }

#endif  // TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE

  MD5 m_md5;
  unsigned char* m_keyBuffer;
};

}  // namespace msg

}  // namespace booster_interface

#endif  // _FAST_DDS_GENERATED_BOOSTER_INTERFACE_MSG_BUTTONEVENT_H_
