cmake_minimum_required(VERSION 2.8)
project(cepton_driveworks)

# Setup cross compilation from x86_64 to aarch64
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-gcc)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/*.c)

include_directories(SYSTEM ${Driveworks_INCLUDE_DIRS} "/usr/local/cuda/include"
                    ${PROJECT_SOURCE_DIR}/../include/)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

# This is a hack to make AARCH64 lib available for cross build.
target_link_libraries(cepton_driveworks ${PROJECT_SOURCE_DIR}/libcepton_sdk.a)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
