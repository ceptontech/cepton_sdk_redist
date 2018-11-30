#[[
CMake file for building samples.
]]
cmake_minimum_required(VERSION 2.8)
project(cepton_sdk_samples)

# set(CMAKE_VERBOSE_MAKEFILE FALSE)

get_filename_component(CEPTON_SDK_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../" ABSOLUTE)
include("${CEPTON_SDK_SOURCE_DIR}/cmake/CommonRedist.cmake")

# cepton_sdk
add_subdirectory("${CEPTON_SDK_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/third_party/cepton_sdk")
include_directories(${CEPTON_SDK_INCLUDE_DIRS})
add_definitions(${CEPTON_SDK_DEFINITIONS})

# Samples
# foreach(name "c_basic")
#   add_executable(cepton_sdk_sample_${name}
#     "${PROJECT_SOURCE_DIR}/${name}.c")
#   target_link_libraries(cepton_sdk_sample_${name}
#     ${CEPTON_SDK_LIBRARIES})
# endforeach(name)

foreach(name "basic" "callback" "error" "frame" "process_single" 
        "process_multi" "sensor" "stray" "transform")
  add_executable(cepton_sdk_sample_${name}
    "${PROJECT_SOURCE_DIR}/${name}.cpp")
  target_link_libraries(cepton_sdk_sample_${name}
    ${CEPTON_SDK_LIBRARIES})
endforeach(name)