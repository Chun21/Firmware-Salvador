cmake_minimum_required(VERSION 3.10)
project(g1_comp_servo_service)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    link_directories(lib/arm)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    link_directories(lib/x86)
else()
    message(STATUS "Architecture not supported")
endif()

if(EXISTS "$ENV{HOME}/.local/lib")
    link_directories("$ENV{HOME}/.local/lib")
endif()

find_package(Boost COMPONENTS program_options REQUIRED)
find_package(yaml-cpp REQUIRED)

set(UNITREE_SDK2_ROOT "" CACHE PATH "Path to vendored Unitree SDK2")
if(NOT UNITREE_SDK2_ROOT AND DEFINED ENV{UNITREE_SDK2_ROOT})
    set(UNITREE_SDK2_ROOT "$ENV{UNITREE_SDK2_ROOT}")
endif()

if(UNITREE_SDK2_ROOT)
    message(STATUS "Using vendored Unitree SDK2: ${UNITREE_SDK2_ROOT}")
    include_directories(
      ${UNITREE_SDK2_ROOT}/include
      ${UNITREE_SDK2_ROOT}/thirdparty/include
      ${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx
    )
    link_directories(
      ${UNITREE_SDK2_ROOT}/lib/aarch64
      ${UNITREE_SDK2_ROOT}/thirdparty/lib/aarch64
    )
    set(UNITREE_SDK2_LIB ${UNITREE_SDK2_ROOT}/lib/aarch64/libunitree_sdk2.a)
else()
    find_package(unitree_sdk2 REQUIRED)
    set(UNITREE_SDK2_LIB unitree_sdk2)
endif()

include_directories(
  include
  $ENV{HOME}/.local/include
  $ENV{HOME}/.local/include/ddscxx
  /usr/local/include/ddscxx
  /usr/local/include/iceoryx/v2.0.2
  thirdparty/DynamixelSDK/c++/include
)

add_executable(main main.cpp)
target_link_libraries(main
  PRIVATE
    ${UNITREE_SDK2_LIB}
    ddsc
    ddscxx
    rt
    pthread
    Boost::program_options
    yaml-cpp
    dxl_x64_cpp
)

option(G1_SERVO_BUILD_TESTS "Build G1 servo service test utilities" OFF)
if(G1_SERVO_BUILD_TESTS)
  add_executable(test_read_yaml test/test_read_yaml.cpp)
  add_executable(test_calibration test/test_calibration.cpp)
  add_executable(test_joint0_control test/test_joint0_control.cpp)
  add_executable(test_joint1_control test/test_joint1_control.cpp)
  add_executable(test_servo_control test/test_servo_control.cpp)
  add_executable(test_read_angle test/test_read_angle.cpp)
  add_executable(test_servo_homePostion test/test_servo_homePostion.cpp)

  foreach(test_target
      test_read_yaml
      test_calibration
      test_joint0_control
      test_joint1_control
      test_servo_control
      test_read_angle
      test_servo_homePostion)
    target_link_libraries(${test_target}
      PRIVATE
        ${UNITREE_SDK2_LIB}
        ddsc
        ddscxx
        rt
        pthread
        Boost::program_options
        yaml-cpp
        dxl_x64_cpp
    )
  endforeach()
endif()
