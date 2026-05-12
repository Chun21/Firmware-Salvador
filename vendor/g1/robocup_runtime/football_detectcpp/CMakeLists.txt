cmake_minimum_required(VERSION 3.12)
project(football_detect)

find_package(CUDA REQUIRED)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
add_definitions(-DAPI_EXPORTS)

# Include CUDA directories
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(src/)
include_directories(../roboCup_sdk/include)

# Add source files
set(SOURCES
    main.cpp
    src/RgbdLocalizer.cpp
    src/YOLO.cpp
    src/DetectionModule.cpp
    ../roboCup_sdk/src/common/LocationModule.cpp
)

# Add CUDA source files
set(CUDA_SOURCES src/preprocess.cu)

# Add headers
set(HEADERS
    src/RgbdLocalizer.h
    src/YOLO.h
    src/macros.h
    src/logging.h
    src/cuda_utils.h
    src/preprocess.h
    src/common.h
    src/DetectionModule.hpp
)






# Find and include OpenCV
#set(OpenCV_DIR "your OpenCV build directory path")
find_package(OpenCV REQUIRED)
include_directories(
${OpenCV_INCLUDE_DIRS}
)


find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)

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

# Include TensorRT
# Set TensorRT path if not set in environment variables
# set(TENSORRT_DIR "/home/unitree/Toots/TensorRT-10.9.0.34/")
# include_directories(${TENSORRT_DIR}/include)
# link_directories(${TENSORRT_DIR}/lib)

set(TENSORRT_LIBS nvinfer nvinfer_plugin  nvonnxparser)

# Create an executable
cuda_add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${CUDA_SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIBS}
    realsense2::realsense2
    ${UNITREE_SDK2_LIB}
    ${UNITREE_SDK2_DEPS}
)

cuda_add_executable(sub sub.cpp src/DetectionModule.cpp src/DetectionModule.hpp )

target_link_libraries(sub
    ${UNITREE_SDK2_LIB}
    ${UNITREE_SDK2_DEPS}
)
