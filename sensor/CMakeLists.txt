cmake_minimum_required(VERSION 3.22)

project(sensor)

add_library(${PROJECT_NAME}
    sensor.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

target_link_libraries(${PROJECT_NAME}
    PUBLIC  
    sensor_pub_sub 
    utils 
    fluxlog
    time
)

if (${ROBOT_MODEL} STREQUAL "g1")
target_link_libraries(${PROJECT_NAME} PUBLIC unitree_sdk2)
else()
target_link_libraries(${PROJECT_NAME} PUBLIC booster_robotics_sdk)
endif()

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
