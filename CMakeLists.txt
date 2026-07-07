cmake_policy(SET CMP0048 NEW)
project(
  ${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

cmake_minimum_required(VERSION 3.16...3.27)

option(BUILD_PYTHON_BINDING "Build Python Binding" off)


set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Detect target architecture to select the correct bundled library directory
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ARCH_DIR "aarch64")
else()
    set(ARCH_DIR "x86_64")
endif()

set(SDK_LIB_DIR      "${CMAKE_SOURCE_DIR}/lib/${ARCH_DIR}")
set(THIRD_PARTY_INC  "${CMAKE_SOURCE_DIR}/third_party/include")
set(THIRD_PARTY_LIB  "${CMAKE_SOURCE_DIR}/third_party/lib/${ARCH_DIR}")

include_directories(BEFORE
    "${CMAKE_SOURCE_DIR}/include"
    "${THIRD_PARTY_INC}"
)

link_directories("${SDK_LIB_DIR}" "${THIRD_PARTY_LIB}" "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu")

# tinyxml2 is a transitive dependency of fastrtps; link by full path as only
# the versioned .so is present (dev package not installed on this host).
find_library(TINYXML2_LIB
    NAMES tinyxml2
    HINTS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu"
    NO_DEFAULT_PATH
)
if(NOT TINYXML2_LIB)
    # Fall back to the versioned shared object directly
    file(GLOB _tinyxml2_candidates
        "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/libtinyxml2.so*")
    if(_tinyxml2_candidates)
        list(GET _tinyxml2_candidates 0 TINYXML2_LIB)
    else()
        message(FATAL_ERROR
            "libtinyxml2 not found. Install libtinyxml2-dev (or equivalent) "
            "to satisfy the transitive dependency of libfastrtps.")
    endif()
endif()
message(STATUS "tinyxml2: ${TINYXML2_LIB}")

link_libraries(booster_robotics_sdk.a fastrtps fastcdr libfoonathan_memory-0.7.3.a "${TINYXML2_LIB}")

add_executable(b1_loco_example_client example/high_level/b1_loco_example_client.cpp)
add_executable(b1_arm_sdk_example_client example/high_level/b1_arm_sdk_example.cpp)
add_executable(b1_7dof_arm_sdk_example_client example/high_level/b1_7dof_arm_sdk_example.cpp)

# visual_kick_no_ros – added per snippet request
add_executable(visual_kick_no_ros example/high_level/visual_kick_no_ros.cpp)
target_include_directories(visual_kick_no_ros PRIVATE "${CMAKE_SOURCE_DIR}/include" "${THIRD_PARTY_INC}")
target_link_directories(visual_kick_no_ros PRIVATE "${SDK_LIB_DIR}" "${THIRD_PARTY_LIB}")
target_link_libraries(visual_kick_no_ros booster_robotics_sdk.a fastrtps fastcdr libfoonathan_memory-0.7.3.a "${TINYXML2_LIB}")

# visual_kick_ball_detect_dds_camera – reads RGB frames directly from DDS,
# detects the ball with YOLO (optional) + colour-based HSV fallback, then kicks.
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn highgui imgcodecs)
add_executable(visual_kick_ball_detect_dds_camera
    example/high_level/visual_kick_ball_detect_dds_camera.cpp)
target_include_directories(visual_kick_ball_detect_dds_camera PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${THIRD_PARTY_INC}"
    ${OpenCV_INCLUDE_DIRS})
target_link_directories(visual_kick_ball_detect_dds_camera PRIVATE
    "${SDK_LIB_DIR}"
    "${THIRD_PARTY_LIB}")
target_link_libraries(visual_kick_ball_detect_dds_camera
    booster_robotics_sdk.a
    fastrtps
    fastcdr
    libfoonathan_memory-0.7.3.a
    "${TINYXML2_LIB}"
    ${OpenCV_LIBS})

add_executable(b1_low_level_publisher example/low_level/low_level_publisher.cpp)
add_executable(b1_low_level_subscriber example/low_level/low_level_subscriber.cpp)
add_executable(low_level_hand_data_subscriber example/low_level/low_level_hand_data_subscriber.cpp)
add_executable(b1_low_sdk_example example/low_level/b1_low_sdk_example.cpp)
add_executable(b1_7dof_arm_low_sdk_example example/low_level/b1_7dof_arm_low_sdk_example.cpp)
add_executable(odometer_example example/low_level/odometer_example.cpp)
add_executable(battery_state_subscriber example/low_level/battery_state_subscriber.cpp)

if(BUILD_PYTHON_BINDING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
    message(STATUS "Python3 executable: ${Python3_EXECUTABLE}")

    execute_process(
        COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_paths()['purelib'])"
        OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    include_directories(${Python3_INCLUDE_DIRS})
    find_package(pybind11 CONFIG REQUIRED)

    python3_add_library(booster_robotics_sdk_python MODULE python/binding.cpp WITH_SOABI)

    target_compile_definitions(booster_robotics_sdk_python PRIVATE VERSION_INFO=${PROJECT_VERSION})

    find_program(PYBIND11_STUBGEN_EXECUTABLE pybind11-stubgen)
    if(NOT PYBIND11_STUBGEN_EXECUTABLE)
        message(FATAL_ERROR "pybind11-stubgen not found")
    endif()

    add_custom_command(
        TARGET booster_robotics_sdk_python
        POST_BUILD
        COMMAND PYTHONPATH=${CMAKE_SOURCE_DIR}/build:/${PYTHONPATH} pybind11-stubgen -o ${CMAKE_SOURCE_DIR}/build booster_robotics_sdk_python
    )

    install(TARGETS booster_robotics_sdk_python LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES})
    install(FILES ${CMAKE_SOURCE_DIR}/build/booster_robotics_sdk_python.pyi DESTINATION ${PYTHON_SITE_PACKAGES})    
endif()
