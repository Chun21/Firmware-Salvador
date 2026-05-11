set(UNITREE_SDK2_ROOT
    "$ENV{UNITREE_SDK2_ROOT}"
    CACHE PATH
    "Root of Unitree SDK2. Defaults to vendored vendor/g1/unitree_sdk2 if unset.")

if(NOT UNITREE_SDK2_ROOT)
  set(UNITREE_SDK2_ROOT "${CMAKE_CURRENT_LIST_DIR}/vendor/g1/unitree_sdk2")
endif()

find_package(Threads REQUIRED)

find_path(UNITREE_SDK2_INCLUDE_DIR
  NAMES unitree/robot/g1/loco/g1_loco_client.hpp
  PATHS
    "${UNITREE_SDK2_ROOT}/include"
    "/home/unitree/.local/include"
    "/usr/local/include"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH)

find_path(UNITREE_SDK2_DDSCXX_INCLUDE_DIR
  NAMES dds/topic/TopicTraits.hpp
  PATHS
    "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx"
    "/home/unitree/.local/include/ddscxx"
    "/usr/local/include/ddscxx"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH)

find_path(UNITREE_SDK2_DDSC_INCLUDE_DIR
  NAMES dds/ddsc/dds_public_qos.h
  PATHS
    "${UNITREE_SDK2_ROOT}/thirdparty/include"
    "/home/unitree/.local/include"
    "/usr/local/include"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH)

find_library(UNITREE_SDK2_LIBRARY
  NAMES unitree_sdk2
  PATHS
    "${UNITREE_SDK2_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
    "/home/unitree/.local/lib"
    "/usr/local/lib"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH)

find_library(UNITREE_DDSC_LIBRARY
  NAMES ddsc
  PATHS
    "${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
    "/home/unitree/.local/lib"
    "/usr/local/lib"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH)

find_library(UNITREE_DDSCXX_LIBRARY
  NAMES ddscxx
  PATHS
    "${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
    "/home/unitree/.local/lib"
    "/usr/local/lib"
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH)

if(NOT UNITREE_SDK2_INCLUDE_DIR OR NOT UNITREE_SDK2_LIBRARY)
  message(FATAL_ERROR
      "Unitree SDK2 not found. Set UNITREE_SDK2_ROOT to a valid SDK2 root. "
      "Tried: ${UNITREE_SDK2_ROOT}")
endif()

if(NOT TARGET unitree_sdk2)
  set(UNITREE_SDK2_DDS_LIBS)
  if(UNITREE_DDSC_LIBRARY)
    list(APPEND UNITREE_SDK2_DDS_LIBS "${UNITREE_DDSC_LIBRARY}")
  else()
    list(APPEND UNITREE_SDK2_DDS_LIBS ddsc)
  endif()
  if(UNITREE_DDSCXX_LIBRARY)
    list(APPEND UNITREE_SDK2_DDS_LIBS "${UNITREE_DDSCXX_LIBRARY}")
  else()
    list(APPEND UNITREE_SDK2_DDS_LIBS ddscxx)
  endif()

  add_library(unitree_sdk2 STATIC IMPORTED GLOBAL)
  set_target_properties(unitree_sdk2 PROPERTIES IMPORTED_LOCATION "${UNITREE_SDK2_LIBRARY}")
  target_include_directories(unitree_sdk2 INTERFACE "${UNITREE_SDK2_INCLUDE_DIR}")
  if(UNITREE_SDK2_DDSCXX_INCLUDE_DIR)
    target_include_directories(unitree_sdk2 INTERFACE "${UNITREE_SDK2_DDSCXX_INCLUDE_DIR}")
  endif()
  if(UNITREE_SDK2_DDSC_INCLUDE_DIR)
    target_include_directories(unitree_sdk2 INTERFACE "${UNITREE_SDK2_DDSC_INCLUDE_DIR}")
  endif()
  target_link_libraries(unitree_sdk2 INTERFACE
      ${UNITREE_SDK2_DDS_LIBS}
      Threads::Threads
      rt)
endif()

message(STATUS "Using Unitree SDK2 include: ${UNITREE_SDK2_INCLUDE_DIR}")
message(STATUS "Using Unitree SDK2 library: ${UNITREE_SDK2_LIBRARY}")

set(UNITREE_SDK2_THIRDPARTY_LIB_DIR
    "${UNITREE_SDK2_ROOT}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}")
if(EXISTS "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}")
  install(DIRECTORY "${UNITREE_SDK2_THIRDPARTY_LIB_DIR}/"
          DESTINATION lib
          FILES_MATCHING PATTERN "*.so*")
endif()
