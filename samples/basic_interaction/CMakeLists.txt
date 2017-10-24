cmake_minimum_required(VERSION 2.8)
project(basic_interaction)

file(GLOB SOURCES ${PROJECT_SOURCE_DIR}/*.c)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME}
  debug ${CEPTON_SDK_LIBRARY_DEBUG}
  optimized  ${CEPTON_SDK_LIBRARY_RELEASE}
  general ${OS_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
