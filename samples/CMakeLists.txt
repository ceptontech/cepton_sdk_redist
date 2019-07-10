#[[
CMake file for building samples.
]]
cmake_minimum_required(VERSION 3.1)
project(cepton_sdk_samples
  VERSION 1.0
  LANGUAGES C CXX)

get_filename_component(CEPTON_SDK_SOURCE_DIR "${PROJECT_SOURCE_DIR}/../" ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH "${CEPTON_SDK_SOURCE_DIR}/cmake")
include("CeptonCommon")

if(GCC OR CLANG)
  add_flags(-Wall)
  add_linker_flags(-pthread)
endif()

# cepton_sdk
add_subdirectory("${CEPTON_SDK_SOURCE_DIR}" "${PROJECT_BINARY_DIR}/third_party/cepton_sdk")

set(CEPTON_SDK_SAMPLE_SOURCES
  "${PROJECT_SOURCE_DIR}/advanced/frame.cpp" 
  "${PROJECT_SOURCE_DIR}/advanced/get_timestamp_offset.cpp" 
  "${PROJECT_SOURCE_DIR}/advanced/process_multi.cpp"
  "${PROJECT_SOURCE_DIR}/advanced/process_single.cpp"     
  "${PROJECT_SOURCE_DIR}/advanced/stray.cpp" 
  "${PROJECT_SOURCE_DIR}/advanced/transform.cpp"
  "${PROJECT_SOURCE_DIR}/basic.cpp" 
  "${PROJECT_SOURCE_DIR}/callback.cpp"
  "${PROJECT_SOURCE_DIR}/error.cpp" 
  "${PROJECT_SOURCE_DIR}/sensor.cpp" 
)
foreach(path ${CEPTON_SDK_SAMPLE_SOURCES})
  get_filename_component(name "${path}" NAME_WE)
  add_executable(cepton_sdk_sample_${name} "${path}")
  target_link_libraries(cepton_sdk_sample_${name}
    cepton_sdk::cepton_sdk)
endforeach()