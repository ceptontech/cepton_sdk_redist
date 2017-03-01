cmake_minimum_required(VERSION 2.8)
project(cepton_sdk_redist)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")


# Definitions
# -----------------
if (WIN32)
  add_definitions(-D_WIN32_WINNT=0x0600)  # Support Vista and above
  set(CEPTON_RELEASE_LIB_PATH win64/cepton_sdk.lib)
  set(CEPTON_DEBUG_LIB_PATH win64_debug/cepton_sdk.lib)
endif()

if (UNIX)
  set(CEPTON_RELEASE_LIB_PATH linux64/libcepton_sdk.a)
  set(CEPTON_DEBUG_LIB_PATH linux64/libcepton_sdk.a)
endif()

# Project files
# -------------
set(CEPTON_SDK_INCLUDE ${PROJECT_SOURCE_DIR}/include)
set(CEPTON_SDK_LIBRARY_DEBUG ${PROJECT_SOURCE_DIR}/lib/${CEPTON_DEBUG_LIB_PATH})
set(CEPTON_SDK_LIBRARY_RELEASE ${PROJECT_SOURCE_DIR}/lib/${CEPTON_RELEASE_LIB_PATH})

file(GLOB CEPTON_SDK_HEADERS ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB CEPTON_SDK_DOCS ${PROJECT_SOURCE_DIR}/*.md)
include_directories(${CEPTON_SDK_INCLUDE})

if(WIN32)
  set(OS_LIBRARIES "Iphlpapi.lib")
endif()

if(UNIX)
  set(OS_LIBRARIES "stdc++" "m" "pthread")
endif()


add_subdirectory(${PROJECT_SOURCE_DIR}/samples/basic_interaction)
add_subdirectory(${PROJECT_SOURCE_DIR}/samples/capture_replay)
