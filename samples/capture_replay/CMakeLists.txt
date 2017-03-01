cmake_minimum_required(VERSION 3.0)
project(capture_replay)

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/*.cpp ${PROJECT_SOURCE_DIR}/*.hpp)
include_directories(SYSTEM ${CEPTON_SDK_INCLUDE_REDIST})

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  debug ${CEPTON_SDK_LIBRARY_DEBUG}
  optimized  ${CEPTON_SDK_LIBRARY_RELEASE}
  general ${OS_LIBRARIES})