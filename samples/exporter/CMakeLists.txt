cmake_minimum_required(VERSION 3.1)

project(cepton_exporter VERSION 0.9.6 LANGUAGES C CXX)

add_definitions(-DVERSION_MAJOR=${PROJECT_VERSION_MAJOR})
add_definitions(-DVERSION_MINOR=${PROJECT_VERSION_MINOR})
add_definitions(-DVERSION_BUILD=${PROJECT_VERSION_PATCH})

file(GLOB HEADERS "*.h*")
file(GLOB SOURCES "*.cpp")

set(LIBS cepton_sdk_static)

if (WIN32)
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
endif()

add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE "${CEPTON_SDK_SOURCE_DIR}/include" "filesystem/include")
target_link_libraries(${PROJECT_NAME} ${LIBS})
