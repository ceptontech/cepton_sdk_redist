# Sample CMake file for building samples
cmake_minimum_required(VERSION 2.8)
project(cepton_sdk_sample)

# cepton_sdk
add_subdirectory("${PROJECT_SOURCE_DIR}/../" "${CMAKE_BINARY_DIR}/third_party/cepton_sdk")
include_directories(${CEPTON_SDK_INCLUDE_DIRS})
add_definitions(${CEPTON_SDK_DEFINITIONS})

# C
foreach(name "c_basic")
    add_executable(cepton_sdk_sample_${name}
        "${PROJECT_SOURCE_DIR}/${name}.c")
    target_link_libraries(cepton_sdk_sample_${name}
        ${CEPTON_SDK_LIBRARIES})
endforeach(name)

# C++
foreach(name "basic" "callback" "error" "frame" 
        "network" "process_single" "process_multi" 
        "replay" "sensor" "stray" "transform")
    add_executable(cepton_sdk_sample_${name}
        "${PROJECT_SOURCE_DIR}/${name}.cpp")
    target_link_libraries(cepton_sdk_sample_${name}
        ${CEPTON_SDK_LIBRARIES})
endforeach(name)