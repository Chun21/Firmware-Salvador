cmake_minimum_required(VERSION 3.12)
project(robocup_locator)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)                                                                                                                                                                
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

option(G1_LOCATOR_BUILD_TEST_SUBSCRIBER "Build test_subscribe_location helper binary" OFF)

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
    set(UNITREE_SDK2_DEPS ddsc ddscxx Threads::Threads rt dl)
else()
    find_package(unitree_sdk2 REQUIRED)
    set(UNITREE_SDK2_LIB unitree_sdk2)
    set(UNITREE_SDK2_DEPS Threads::Threads rt dl)
endif()

include_directories("include")

file(GLOB SOURCES "src/*.cpp")

add_executable(test_location ${SOURCES} test_location.cpp)
target_link_libraries(test_location 
                    ${UNITREE_SDK2_LIB}
                    ${UNITREE_SDK2_DEPS}
                    Eigen3::Eigen 
                    ${OpenCV_LIBS}
                    yaml-cpp
                    )

if(G1_LOCATOR_BUILD_TEST_SUBSCRIBER)
    add_executable(test_subscribe_location src/LocationModule.cpp test_subscribe_location.cpp)
    target_link_libraries(test_subscribe_location ${UNITREE_SDK2_LIB} ${UNITREE_SDK2_DEPS})
endif()

add_executable(location_fusion src/LocationModule.cpp src/DetectionModule.cpp src/TeamBallModule.cpp location_fusion.cpp)
target_link_libraries(location_fusion ${UNITREE_SDK2_LIB} ${UNITREE_SDK2_DEPS})
