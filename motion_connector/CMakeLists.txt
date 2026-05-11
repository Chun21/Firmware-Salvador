cmake_minimum_required(VERSION 3.22)

project(motion_connector)

add_library(motion_connector)

if (${ROBOT_MODEL} STREQUAL "g1")
target_sources(motion_connector PRIVATE
    motion_connector_g1.h
    motion_connector_g1.cpp
)
else()
target_sources(motion_connector PRIVATE
    motion_connector_t1.h
    motion_connector_t1.cpp
    motion_connector_k1.h
    motion_connector_k1.cpp
)
endif()
target_include_directories(motion_connector
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(motion_connector
    PUBLIC motion_pub_sub utils fluxlog sensor_pub_sub time parameter_tuner)

if (${ROBOT_MODEL} STREQUAL "g1")
target_link_libraries(motion_connector PUBLIC unitree_sdk2)
else()
target_link_libraries(motion_connector PUBLIC booster_robotics_sdk policy_executor)
endif()

target_compile_features(motion_connector INTERFACE cxx_std_17)
