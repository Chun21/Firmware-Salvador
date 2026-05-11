project(Agents)

add_library(${PROJECT_NAME}
    agent_runner.cpp
    walk_relative.cpp
    walk_to_pos.cpp
    search_ball.cpp
    localize.cpp
    goalie_positioning.cpp
    goalie_dive.cpp
)

if (NOT ROBOT_MODEL STREQUAL "g1")
target_sources(${PROJECT_NAME} PRIVATE
    shoot.cpp
)
endif()

if (${ROBOT_MODEL} STREQUAL "k1")
target_sources(${PROJECT_NAME} PRIVATE
    dribble_k1.cpp
)
else()
target_sources(${PROJECT_NAME} PRIVATE
    dribble.cpp
)
endif()

target_link_libraries(${PROJECT_NAME} utils Orders SoccerField motion_pub_sub fluxlog
 localization_pub_sub multi_target_tracker_pub_sub head_control time
 game_controller_pub_sub team_strategy_pub_sub
 near_obstacle_tracker_pub_sub
 )

if (NOT ROBOT_MODEL STREQUAL "g1")
target_link_libraries(${PROJECT_NAME} policy_executor)
endif()
 
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
