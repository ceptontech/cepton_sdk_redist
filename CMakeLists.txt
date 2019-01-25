#[[
CMake file for including cepton_sdk in a CMake project.

See `samples/CMakeLists.txt` for example usage.
]]
cmake_minimum_required(VERSION 3.1)
project(cepton_sdk
  VERSION 0.1
  LANGUAGES C CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("CeptonCommon")

create_option(BOOL CEPTON_SDK_USE_STATIC TRUE
    "Use static cepton sdk libraries.")

if(CEPTON_SDK_USE_STATIC)
    cepton_get_static_library(CEPTON_SDK_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}" "cepton_sdk")
    add_library(cepton_sdk::cepton_sdk STATIC IMPORTED GLOBAL)
    set_target_properties(cepton_sdk::cepton_sdk PROPERTIES
      INTERFACE_COMPILE_DEFINITIONS CEPTON_SDK_STATIC
    )
else()
    cepton_get_shared_library(CEPTON_SDK_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}" "cepton_sdk")
    add_library(cepton_sdk::cepton_sdk SHARED IMPORTED GLOBAL)
endif()
set_target_properties(cepton_sdk::cepton_sdk PROPERTIES
  IMPORTED_LOCATION ${CEPTON_SDK_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
)