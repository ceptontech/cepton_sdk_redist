#[[
CMake file for building samples.
]]
cmake_minimum_required(VERSION 3.1)

set(CEPTON_SDK_SAMPLES_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")
get_filename_component(CEPTON_SDK_SOURCE_DIR
                       "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/../" ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH "${CEPTON_SDK_SOURCE_DIR}/cmake")

include("CeptonVersion")
project(cepton_sdk_samples VERSION ${CEPTON_VERSION} LANGUAGES C CXX)

include("CeptonCommon")

if(GCC OR CLANG)
  add_flags(-Wall)
  add_linker_flags(-pthread)
endif()

# cepton_sdk
add_subdirectory("${CEPTON_SDK_SOURCE_DIR}"
                 "${PROJECT_BINARY_DIR}/third_party/cepton_sdk")

set(CEPTON_SDK_SAMPLE_SOURCES
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/advanced/frame_accumulator.cpp"
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/advanced/frame_detector.cpp"
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/advanced/process_multi.cpp"
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/advanced/process_single.cpp"
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/advanced/stray_filter.cpp"
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/basic.cpp"
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/callback.cpp"
    "${CEPTON_SDK_SAMPLES_SOURCE_DIR}/error.cpp")
foreach(path ${CEPTON_SDK_SAMPLE_SOURCES})
  get_filename_component(name "${path}" NAME_WE)
  add_executable(cepton_sdk_sample_${name} "${path}")
  target_include_directories(cepton_sdk_sample_${name}
                             PRIVATE "${CEPTON_SDK_SAMPLES_SOURCE_DIR}")
  target_link_libraries(cepton_sdk_sample_${name} cepton_sdk::cepton_sdk)
endforeach()
