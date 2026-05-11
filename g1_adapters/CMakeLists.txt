cmake_minimum_required(VERSION 3.22)

project(g1_adapters)

set(G1_ROBOCUP_DDS_ROOT
    "$ENV{G1_ROBOCUP_DDS_ROOT}"
    CACHE PATH
    "Root of vendored G1 RoboCup DDS types.")

if(NOT G1_ROBOCUP_DDS_ROOT)
  set(G1_ROBOCUP_DDS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../vendor/g1/robocup_dds")
endif()

set(G1_DETECTION_DDS_DIR "${G1_ROBOCUP_DDS_ROOT}/detection")
set(G1_LOCATION_DDS_DIR "${G1_ROBOCUP_DDS_ROOT}/location")

if(NOT EXISTS "${G1_DETECTION_DDS_DIR}/DetectionModule.hpp")
  message(FATAL_ERROR "G1 DetectionModule.hpp not found under ${G1_DETECTION_DDS_DIR}")
endif()

if(NOT EXISTS "${G1_LOCATION_DDS_DIR}/LocationModule.hpp")
  message(FATAL_ERROR "G1 LocationModule.hpp not found under ${G1_LOCATION_DDS_DIR}")
endif()

message(STATUS "Using G1 RoboCup DDS root: ${G1_ROBOCUP_DDS_ROOT}")

add_library(${PROJECT_NAME}
    g1_external_adapters.cpp
    "${G1_DETECTION_DDS_DIR}/DetectionModule.cpp"
    "${G1_LOCATION_DDS_DIR}/LocationModule.cpp"
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        "${G1_DETECTION_DDS_DIR}"
        "${G1_LOCATION_DDS_DIR}"
    PRIVATE
        "${G1_DETECTION_DDS_DIR}"
        "${G1_LOCATION_DDS_DIR}"
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        unitree_sdk2
        utils
        vision_pub_sub
        localization_pub_sub
        multi_target_tracker_pub_sub
        near_obstacle_tracker_pub_sub
        fluxlog
        time
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
