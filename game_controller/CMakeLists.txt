cmake_minimum_required(VERSION 3.22)

project(game_controller)

add_library(${PROJECT_NAME}
    game_controller.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC game_controller_pub_sub gc_humanoid Boost::system fluxlog localization_pub_sub
    multi_target_tracker_pub_sub sensor_pub_sub
    PRIVATE utils)
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/localization/pub_sub
        ${CMAKE_SOURCE_DIR}/multi_target_tracker/pub_sub
        ${CMAKE_SOURCE_DIR}/sensor/pub_sub
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
