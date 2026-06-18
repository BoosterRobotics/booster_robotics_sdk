cmake_minimum_required(VERSION 3.15...3.27)
cmake_policy(SET CMP0048 NEW)

if(NOT DEFINED SKBUILD_PROJECT_NAME)
    set(SKBUILD_PROJECT_NAME booster_robotics_sdk_release)
endif()

if(NOT DEFINED SKBUILD_PROJECT_VERSION)
    set(SKBUILD_PROJECT_VERSION 0.1.0)
endif()

project(
  ${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

set(BOOSTER_SDK_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
if(NOT BOOSTER_SDK_ARCH)
    execute_process(
        COMMAND uname -m
        OUTPUT_VARIABLE BOOSTER_SDK_ARCH
        OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

if(BOOSTER_SDK_ARCH MATCHES "^(AMD64|amd64)$")
    set(BOOSTER_SDK_ARCH x86_64)
endif()

set(BOOSTER_SDK_LIBRARY "${PROJECT_SOURCE_DIR}/lib/${BOOSTER_SDK_ARCH}/libbooster_robotics_sdk.a")
if(NOT EXISTS "${BOOSTER_SDK_LIBRARY}")
    message(FATAL_ERROR "Booster Robotics SDK library not found: ${BOOSTER_SDK_LIBRARY}")
endif()

include_directories(BEFORE ${PROJECT_SOURCE_DIR}/include)
link_libraries("${BOOSTER_SDK_LIBRARY}" Threads::Threads ${CMAKE_DL_LIBS} rt)

add_executable(b1_loco_example_client example/high_level/b1_loco_example_client.cpp)
add_executable(b1_arm_sdk_example_client example/high_level/b1_arm_sdk_example.cpp)
add_executable(b1_7dof_arm_sdk_example_client example/high_level/b1_7dof_arm_sdk_example.cpp)
add_executable(b1_low_level_publisher example/low_level/low_level_publisher.cpp)
add_executable(b1_low_level_subscriber example/low_level/low_level_subscriber.cpp)
add_executable(low_level_hand_data_subscriber example/low_level/low_level_hand_data_subscriber.cpp)
add_executable(b1_low_sdk_example example/low_level/b1_low_sdk_example.cpp)
add_executable(b1_7dof_arm_low_sdk_example example/low_level/b1_7dof_arm_low_sdk_example.cpp)
add_executable(odometer_example example/low_level/odometer_example.cpp)
add_executable(battery_state_subscriber example/low_level/battery_state_subscriber.cpp)
