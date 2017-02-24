cmake_minimum_required(VERSION 3.0)
project(basic_interaction)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/*.c)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${CEPTON_SDK_LIBRARY} ${OS_LIBRARIES})
