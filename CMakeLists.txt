#[[
CMake file for including cepton_sdk in a CMake project.

See `samples/CMakeLists.txt` for example usage.
]]
set(CEPTON_SDK_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CEPTON_SDK_SOURCE_DIR}/cmake")

include("CeptonVersion")

cmake_minimum_required(VERSION 3.1)
project(cepton_sdk VERSION ${CEPTON_VERSION} LANGUAGES C CXX)

include("CeptonCommon")

create_option(BOOL
              CEPTON_SDK_USE_STATIC
              FALSE
              "Use static cepton sdk libraries.")

if(NOT TARGET cepton_sdk::cepton_sdk)
  if(CEPTON_SDK_USE_STATIC)
    add_library(cepton_sdk::cepton_sdk
                STATIC
                IMPORTED
                GLOBAL)
    cepton_import_static_library(cepton_sdk::cepton_sdk
                                 "${CMAKE_CURRENT_SOURCE_DIR}" "cepton_sdk")
    set_target_properties(cepton_sdk::cepton_sdk
                          PROPERTIES INTERFACE_COMPILE_DEFINITIONS
                                     CEPTON_SDK_STATIC)
  else()
    add_library(cepton_sdk::cepton_sdk
                SHARED
                IMPORTED
                GLOBAL)
    cepton_import_shared_library(cepton_sdk::cepton_sdk
                                 "${CMAKE_CURRENT_SOURCE_DIR}" "cepton_sdk")
  endif()
  set_target_properties(cepton_sdk::cepton_sdk
                        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                   "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()
